/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include "esp_rom_sys.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "hal/clk_tree_ll.h"
#include "hal/rtc_cntl_ll.h"
#include "hal/timer_ll.h"
#include "soc/timer_group_reg.h"
#include "esp_private/periph_ctrl.h"

/* Calibration of RTC_SLOW_CLK is performed using a special feature of TIMG0.
 * This feature counts the number of XTAL clock cycles within a given number of
 * RTC_SLOW_CLK cycles.
 *
 * Slow clock calibration feature has two modes of operation: one-off and cycling.
 * In cycling mode (which is enabled by default on SoC reset), counting of XTAL
 * cycles within RTC_SLOW_CLK cycle is done continuously. Cycling mode is enabled
 * using TIMG_RTC_CALI_START_CYCLING bit. In one-off mode counting is performed
 * once, and TIMG_RTC_CALI_RDY bit is set when counting is done. One-off mode is
 * enabled using TIMG_RTC_CALI_START bit.
 */

/**
 * @brief One-off clock calibration function used by rtc_clk_cal_internal
 * @param cal_clk which clock to calibrate
 * @param slowclk_cycles number of slow clock cycles to count
 * @return number of XTAL clock cycles within the given number of slow clock cycles
 */
static uint32_t rtc_clk_cal_internal_oneoff(rtc_cal_sel_t cal_clk, uint32_t slowclk_cycles)
{
    /* There may be another calibration process already running during we call this function,
     * so we should wait the last process is done.
     */
    if (GET_PERI_REG_MASK(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START_CYCLING)) {
        /**
         * Set a small timeout threshold to accelerate the generation of timeout.
         * The internal circuit will be reset when the timeout occurs and will not affect the next calibration.
         */
        REG_SET_FIELD(TIMG_RTCCALICFG2_REG(0), TIMG_RTC_CALI_TIMEOUT_THRES, 1);
        while (!GET_PERI_REG_MASK(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_RDY)
               && !GET_PERI_REG_MASK(TIMG_RTCCALICFG2_REG(0), TIMG_RTC_CALI_TIMEOUT));
    }

    /* Prepare calibration */
    REG_SET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_CLK_SEL, cal_clk);
    CLEAR_PERI_REG_MASK(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START_CYCLING);
    REG_SET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_MAX, slowclk_cycles);
    /* Figure out how long to wait for calibration to finish */

    /* Set timeout reg and expect time delay*/
    uint32_t expected_freq;
    if (cal_clk == RTC_CAL_32K_XTAL) {
        REG_SET_FIELD(TIMG_RTCCALICFG2_REG(0), TIMG_RTC_CALI_TIMEOUT_THRES, RTC_SLOW_CLK_X32K_CAL_TIMEOUT_THRES(slowclk_cycles));
        expected_freq = SOC_CLK_XTAL32K_FREQ_APPROX;
    } else if (cal_clk == RTC_CAL_8MD256) {
        REG_SET_FIELD(TIMG_RTCCALICFG2_REG(0), TIMG_RTC_CALI_TIMEOUT_THRES, RTC_SLOW_CLK_8MD256_CAL_TIMEOUT_THRES(slowclk_cycles));
        expected_freq = SOC_CLK_RC_FAST_D256_FREQ_APPROX;
    } else {
        REG_SET_FIELD(TIMG_RTCCALICFG2_REG(0), TIMG_RTC_CALI_TIMEOUT_THRES, RTC_SLOW_CLK_90K_CAL_TIMEOUT_THRES(slowclk_cycles));
        expected_freq = SOC_CLK_RC_SLOW_FREQ_APPROX;
    }
    uint32_t us_time_estimate = (uint32_t) (((uint64_t) slowclk_cycles) * MHZ / expected_freq);
    /* Start calibration */
    CLEAR_PERI_REG_MASK(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START);
    SET_PERI_REG_MASK(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START);

    /* Wait for calibration to finish up to another us_time_estimate */
    esp_rom_delay_us(us_time_estimate);
    uint32_t cal_val;
    while (true) {
        if (GET_PERI_REG_MASK(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_RDY)) {
            cal_val = REG_GET_FIELD(TIMG_RTCCALICFG1_REG(0), TIMG_RTC_CALI_VALUE);
            break;
        }
        if (GET_PERI_REG_MASK(TIMG_RTCCALICFG2_REG(0), TIMG_RTC_CALI_TIMEOUT)) {
            cal_val = 0;
            break;
        }
    }

    return cal_val;
}

/**
 * @brief Cycling clock calibration function used by rtc_clk_cal_internal
 * @param cal_clk which clock to calibrate
 * @param slowclk_cycles number of slow clock cycles to count
 * @return number of XTAL clock cycles within the given number of slow clock cycles
 */
static uint32_t rtc_clk_cal_internal_cycling(rtc_cal_sel_t cal_clk, uint32_t slowclk_cycles)
{
    /* Get which slowclk is in calibration and max cali cycles */
    rtc_cal_sel_t in_calibration_clk;
    in_calibration_clk = REG_GET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_CLK_SEL);
    uint32_t cali_slowclk_cycles = REG_GET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_MAX);
    /* If no calibration in process or calibration period equal to 0, use slowclk_cycles cycles to calibrate slowclk */
    if (cali_slowclk_cycles == 0 || !GET_PERI_REG_MASK(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START_CYCLING) || in_calibration_clk != cal_clk) {
        CLEAR_PERI_REG_MASK(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START_CYCLING);
        REG_SET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_CLK_SEL, cal_clk);
        REG_SET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_MAX, slowclk_cycles);
        SET_PERI_REG_MASK(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START_CYCLING);
        cali_slowclk_cycles = slowclk_cycles;
    }

    /* Wait for calibration finished */
    while (!GET_PERI_REG_MASK(TIMG_RTCCALICFG1_REG(0), TIMG_RTC_CALI_CYCLING_DATA_VLD));
    uint32_t cal_val = REG_GET_FIELD(TIMG_RTCCALICFG1_REG(0), TIMG_RTC_CALI_VALUE);

    return cal_val;
}

/**
 * @brief Slowclk period calculating function used by rtc_clk_cal and rtc_clk_cal_cycling
 * @param xtal_cycles number of xtal cycles count
 * @param slowclk_cycles number of slow clock cycles to count
 * @return slow clock period
 */
static uint32_t rtc_clk_xtal_to_slowclk(uint64_t xtal_cycles, uint32_t slowclk_cycles)
{
    assert(slowclk_cycles);
    soc_xtal_freq_t xtal_freq = rtc_clk_xtal_freq_get();
    uint64_t divider = ((uint64_t)xtal_freq) * slowclk_cycles;
    uint64_t period_64 = ((xtal_cycles << RTC_CLK_CAL_FRACT) + divider / 2 - 1) / divider;
    uint32_t period = (uint32_t)(period_64 & UINT32_MAX);
    return period;
}

/**
 * @brief Clock calibration function used by rtc_clk_cal and rtc_clk_cal_ratio
 * @param cal_clk which clock to calibrate
 * @param slowclk_cycles number of slow clock cycles to count
 * @return number of XTAL clock cycles within the given number of slow clock cycles
 */
uint32_t rtc_clk_cal_internal(rtc_cal_sel_t cal_clk, uint32_t slowclk_cycles, uint32_t cal_mode)
{
    /* On ESP32S2, choosing RTC_CAL_RTC_MUX results in calibration of
     * the 90k RTC clock regardless of the currently selected SLOW_CLK.
     * On the ESP32, it used the currently selected SLOW_CLK.
     * The following code emulates ESP32 behavior:
     */
    if (cal_clk == RTC_CAL_RTC_MUX) {
        soc_rtc_slow_clk_src_t slow_clk_src = rtc_clk_slow_src_get();
        if (slow_clk_src == SOC_RTC_SLOW_CLK_SRC_XTAL32K) {
            cal_clk = RTC_CAL_32K_XTAL;
        } else if (slow_clk_src == SOC_RTC_SLOW_CLK_SRC_RC_FAST_D256) {
            cal_clk = RTC_CAL_8MD256;
        }
    } else if (cal_clk == RTC_CAL_INTERNAL_OSC) {
        cal_clk = RTC_CAL_RTC_MUX;
    }

    /* Enable requested clock (90k clock is always on) */
    bool dig_32k_xtal_enabled = clk_ll_xtal32k_digi_is_enabled();
    if (cal_clk == RTC_CAL_32K_XTAL && !dig_32k_xtal_enabled) {
        clk_ll_xtal32k_digi_enable();
    }

    bool rc_fast_enabled = clk_ll_rc_fast_is_enabled();
    bool rc_fast_d256_enabled = clk_ll_rc_fast_d256_is_enabled();
    if (cal_clk == RTC_CAL_8MD256) {
        rtc_clk_8m_enable(true, true);
        clk_ll_rc_fast_d256_digi_enable();
    }

    uint32_t cal_val;
    if (cal_mode == RTC_TIME_CAL_ONEOFF_MODE) {
        cal_val = rtc_clk_cal_internal_oneoff(cal_clk, slowclk_cycles);
    } else {
        cal_val = rtc_clk_cal_internal_cycling(cal_clk, slowclk_cycles);
    }

    CLEAR_PERI_REG_MASK(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START);

    /* if dig_32k_xtal was originally off and enabled due to calibration, then set back to off state */
    if (cal_clk == RTC_CAL_32K_XTAL && !dig_32k_xtal_enabled) {
        clk_ll_xtal32k_digi_disable();
    }

    if (cal_clk == RTC_CAL_8MD256) {
        clk_ll_rc_fast_d256_digi_disable();
        rtc_clk_8m_enable(rc_fast_enabled, rc_fast_d256_enabled);
    }

    return cal_val;
}

uint32_t rtc_clk_cal_ratio(rtc_cal_sel_t cal_clk, uint32_t slowclk_cycles)
{
    assert(slowclk_cycles);
    uint64_t xtal_cycles = rtc_clk_cal_internal(cal_clk, slowclk_cycles, RTC_TIME_CAL_ONEOFF_MODE);
    uint64_t ratio_64 = ((xtal_cycles << RTC_CLK_CAL_FRACT)) / slowclk_cycles;
    uint32_t ratio = (uint32_t)(ratio_64 & UINT32_MAX);
    return ratio;
}

static inline bool rtc_clk_cal_32k_valid(uint32_t xtal_freq, uint32_t slowclk_cycles, uint64_t actual_xtal_cycles)
{
    uint64_t expected_xtal_cycles = (xtal_freq * 1000000ULL * slowclk_cycles) >> 15; // xtal_freq(hz) * slowclk_cycles / 32768
    uint64_t delta = expected_xtal_cycles / 2000;                                    // 5/10000
    return (actual_xtal_cycles >= (expected_xtal_cycles - delta)) && (actual_xtal_cycles <= (expected_xtal_cycles + delta));
}

uint32_t rtc_clk_cal(rtc_cal_sel_t cal_clk, uint32_t slowclk_cycles)
{
    uint64_t xtal_cycles = rtc_clk_cal_internal(cal_clk, slowclk_cycles, RTC_TIME_CAL_ONEOFF_MODE);

    if ((cal_clk == RTC_CAL_32K_XTAL) && !rtc_clk_cal_32k_valid((uint32_t)rtc_clk_xtal_freq_get(), slowclk_cycles, xtal_cycles)) {
        return 0;
    }

    return rtc_clk_xtal_to_slowclk(xtal_cycles, slowclk_cycles);
}

uint32_t rtc_clk_cal_cycling(rtc_cal_sel_t cal_clk, uint32_t slowclk_cycles)
{
    uint64_t xtal_cycles = rtc_clk_cal_internal(cal_clk, slowclk_cycles, RTC_TIME_CAL_CYCLING_MODE);
    uint32_t period = rtc_clk_xtal_to_slowclk(xtal_cycles, slowclk_cycles);
    return period;
}

uint64_t rtc_time_us_to_slowclk(uint64_t time_in_us, uint32_t period)
{
    assert(period);
    if (time_in_us > (UINT64_MAX >> RTC_CLK_CAL_FRACT)) {
        return ((time_in_us / period) << RTC_CLK_CAL_FRACT) + ((time_in_us % period) << RTC_CLK_CAL_FRACT) / period;
    }
    return (time_in_us << RTC_CLK_CAL_FRACT) / period;
}

uint64_t rtc_time_slowclk_to_us(uint64_t rtc_cycles, uint32_t period)
{
    return (rtc_cycles * period) >> RTC_CLK_CAL_FRACT;
}

uint64_t rtc_time_get(void)
{
    return rtc_cntl_ll_get_rtc_time();
}

void rtc_clk_wait_for_slow_cycle(void) //This function may not by useful any more
{
    SET_PERI_REG_MASK(RTC_CNTL_SLOW_CLK_CONF_REG, RTC_CNTL_SLOW_CLK_NEXT_EDGE);
    while (GET_PERI_REG_MASK(RTC_CNTL_SLOW_CLK_CONF_REG, RTC_CNTL_SLOW_CLK_NEXT_EDGE)) {
        esp_rom_delay_us(1);
    }
}

uint32_t rtc_clk_freq_cal(uint32_t cal_val)
{
    if (cal_val == 0) {
        return 0;   // cal_val will be denominator, return 0 as the symbol of failure.
    }
    return 1000000ULL * (1 << RTC_CLK_CAL_FRACT) / cal_val;
}

/// @brief if the calibration is used, we need to enable the timer group0 first
__attribute__((constructor))
static void enable_timer_group0_for_calibration(void)
{
#ifndef BOOTLOADER_BUILD
    PERIPH_RCC_ACQUIRE_ATOMIC(PERIPH_TIMG0_MODULE, ref_count) {
        if (ref_count == 0) {
            timer_ll_enable_bus_clock(0, true);
            timer_ll_reset_register(0);
        }
    }
#else
    _timer_ll_enable_bus_clock(0, true);
    _timer_ll_reset_register(0);
#endif
}
