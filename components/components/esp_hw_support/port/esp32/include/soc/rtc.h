/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "soc/soc.h"
#include "soc/rtc_periph.h"
#include "soc/clk_tree_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************************/
/***************** THIS FILE IS CONSIDERED AS A PRIVATE HEADER FILE *****************/
/*** IT IS NOT RECOMMENDED TO USE THE APIS IN THIS FILE DIRECTLY IN APPLICATIONS ****/
/************************************************************************************/

/**
 * @file rtc.h
 * @brief Low-level RTC power, clock, and sleep functions.
 *
 * Functions in this file facilitate configuration of ESP32's RTC_CNTL peripheral.
 * RTC_CNTL peripheral handles many functions:
 * - enables/disables clocks and power to various parts of the chip; this is
 *   done using direct register access (forcing power up or power down) or by
 *   allowing state machines to control power and clocks automatically
 * - handles sleep and wakeup functions
 * - maintains a 48-bit counter which can be used for timekeeping
 *
 * These functions are not thread safe, and should not be viewed as high level
 * APIs. For example, while this file provides a function which can switch
 * CPU frequency, this function is on its own is not sufficient to implement
 * frequency switching in ESP-IDF context: some coordination with RTOS,
 * peripheral drivers, and WiFi/BT stacks is also required.
 *
 * These functions will normally not be used in applications directly.
 * ESP-IDF provides, or will provide, drivers and other facilities to use
 * RTC subsystem functionality.
 *
 * The functions are loosely split into the following groups:
 * - rtc_clk: clock switching, calibration
 * - rtc_time: reading RTC counter, conversion between counter values and time
 * - rtc_sleep: entry into sleep modes
 * - rtc_init: initialization
 */

#define MHZ (1000000)

/* Delays for various clock sources to be enabled/switched.
 * All values are in microseconds.
 */
#define SOC_DELAY_PLL_DBIAS_RAISE           3
#define SOC_DELAY_PLL_ENABLE_WITH_150K      80
#define SOC_DELAY_PLL_ENABLE_WITH_32K       160
#define SOC_DELAY_RTC_FAST_CLK_SWITCH       3
#define SOC_DELAY_RTC_SLOW_CLK_SWITCH       300
#define SOC_DELAY_RC_FAST_ENABLE            50
#define SOC_DELAY_RC_FAST_DIGI_SWITCH       5

/* Core voltage needs to be increased in two cases:
 * 1. running at 240 MHz
 * 2. running with 80MHz Flash frequency
 *
 * There is a record in efuse which indicates the proper voltage for these two cases.
 */
#define RTC_CNTL_DBIAS_HP_VOLT         (RTC_CNTL_DBIAS_1V25 - efuse_ll_get_vol_level_hp_inv())
#ifdef CONFIG_ESPTOOLPY_FLASHFREQ_80M
#define DIG_DBIAS_80M_160M  RTC_CNTL_DBIAS_HP_VOLT
#else
#define DIG_DBIAS_80M_160M  RTC_CNTL_DBIAS_1V10
#endif
#define DIG_DBIAS_240M      RTC_CNTL_DBIAS_HP_VOLT
#define DIG_DBIAS_XTAL      RTC_CNTL_DBIAS_1V10
#define DIG_DBIAS_2M        RTC_CNTL_DBIAS_1V00

/**
 * @brief CPU clock configuration structure
 */
typedef struct rtc_cpu_freq_config_s {
    soc_cpu_clk_src_t source;       //!< The clock from which CPU clock is derived
    uint32_t source_freq_mhz;       //!< Source clock frequency
    uint32_t div;                   //!< Divider, freq_mhz = source_freq_mhz / div
    uint32_t freq_mhz;              //!< CPU clock frequency
} rtc_cpu_freq_config_t;

/**
 * @brief Clock source to be calibrated using rtc_clk_cal function
 */
typedef enum {
    RTC_CAL_RTC_MUX = 0,       //!< Currently selected RTC SLOW_CLK
    RTC_CAL_8MD256 = 1,        //!< Internal 8 MHz RC oscillator, divided by 256
    RTC_CAL_32K_XTAL = 2       //!< External 32 kHz XTAL
} rtc_cal_sel_t;

/**
 * Initialization parameters for rtc_clk_init
 */
typedef struct rtc_clk_config_s {
    soc_xtal_freq_t xtal_freq : 8;             //!< Main XTAL frequency
    uint32_t cpu_freq_mhz : 10;                //!< CPU frequency to set, in MHz
    soc_rtc_fast_clk_src_t fast_clk_src : 2;   //!< RTC_FAST_CLK clock source to choose
    soc_rtc_slow_clk_src_t slow_clk_src : 2;   //!< RTC_SLOW_CLK clock source to choose
    uint32_t clk_8m_div : 3;                   //!< RTC 8M clock divider (division is by clk_8m_div+1, i.e. 0 means 8MHz frequency)
    uint32_t slow_clk_dcap : 8;                //!< RTC 150k clock adjustment parameter (higher value leads to lower frequency)
    uint32_t clk_8m_dfreq : 8;                 //!< RTC 8m clock adjustment parameter (higher value leads to higher frequency)
} rtc_clk_config_t;

/**
 * Default initializer for rtc_clk_config_t
 */
#define RTC_CLK_CONFIG_DEFAULT() { \
    .xtal_freq = CONFIG_XTAL_FREQ, \
    .cpu_freq_mhz = 80, \
    .fast_clk_src = SOC_RTC_FAST_CLK_SRC_RC_FAST, \
    .slow_clk_src = SOC_RTC_SLOW_CLK_SRC_RC_SLOW, \
    .clk_8m_div = 0, \
    .slow_clk_dcap = RTC_CNTL_SCK_DCAP_DEFAULT, \
    .clk_8m_dfreq = RTC_CNTL_CK8M_DFREQ_DEFAULT, \
}

/**
 * Initialize clocks and set CPU frequency
 *
 * If cfg.xtal_freq is set to SOC_XTAL_FREQ_AUTO, this function will attempt
 * to auto detect XTAL frequency. Auto detection is performed by comparing
 * XTAL frequency with the frequency of internal 8MHz oscillator. Note that at
 * high temperatures the frequency of the internal 8MHz oscillator may drift
 * enough for auto detection to be unreliable.
 * Auto detection code will attempt to distinguish between 26MHz and 40MHz
 * crystals. 24 MHz crystals are not supported by auto detection code.
 * If XTAL frequency can not be auto detected, this 26MHz frequency will be used.
 *
 * @param cfg clock configuration as rtc_clk_config_t
 */
void rtc_clk_init(rtc_clk_config_t cfg);

/**
 * @brief Get main XTAL frequency
 *
 * This is the value stored in RTC register RTC_XTAL_FREQ_REG by the bootloader. As passed to
 * rtc_clk_init function, or if the value was SOC_XTAL_FREQ_AUTO, the detected
 * XTAL frequency.
 *
 * @return XTAL frequency, one of soc_xtal_freq_t
 */
soc_xtal_freq_t rtc_clk_xtal_freq_get(void);

/**
 * @brief Update XTAL frequency
 *
 * Updates the XTAL value stored in RTC_XTAL_FREQ_REG. Usually this value is ignored
 * after startup.
 *
 * @param xtal_freq New frequency value
 */
void rtc_clk_xtal_freq_update(soc_xtal_freq_t xtal_freq);

/**
 * @brief Enable or disable 32 kHz XTAL oscillator
 * @param en  true to enable, false to disable
 */
void rtc_clk_32k_enable(bool en);

/**
 * @brief Configure 32 kHz XTAL oscillator to accept external clock signal
 */
void rtc_clk_32k_enable_external(void);

/**
 * @brief Get the state of 32k XTAL oscillator
 * @return true if 32k XTAL oscillator has been enabled
 */
bool rtc_clk_32k_enabled(void);

/**
 * @brief Enable 32k oscillator, configuring it for fast startup time.
 * Note: to achieve higher frequency stability, rtc_clk_32k_enable function
 * must be called one the 32k XTAL oscillator has started up. This function
 * will initially disable the 32k XTAL oscillator, so it should not be called
 * when the system is using 32k XTAL as RTC_SLOW_CLK.
 *
 * @param cycle Number of 32kHz cycles to bootstrap external crystal.
 *              If 0, no square wave will be used to bootstrap crystal oscillation.
 */
void rtc_clk_32k_bootstrap(uint32_t cycle);

/**
 * @brief Enable or disable 8 MHz internal oscillator
 *
 * Output from 8 MHz internal oscillator is passed into a configurable
 * divider, which by default divides the input clock frequency by 256.
 * Output of the divider may be used as RTC_SLOW_CLK source.
 * Output of the divider is referred to in register descriptions and code as
 * 8md256 or simply d256. Divider values other than 256 may be configured, but
 * this facility is not currently needed, so is not exposed in the code.
 *
 * When 8MHz/256 divided output is not needed, the divider should be disabled
 * to reduce power consumption.
 *
 * @param clk_8m_en true to enable 8MHz generator
 * @param d256_en true to enable /256 divider
 */
void rtc_clk_8m_enable(bool clk_8m_en, bool d256_en);

/**
 * @brief Get the state of 8 MHz internal oscillator
 * @return true if the oscillator is enabled
 */
bool rtc_clk_8m_enabled(void);

/**
 * @brief Get the state of /256 divider which is applied to 8MHz clock
 * @return true if the divided output is enabled
 */
bool rtc_clk_8md256_enabled(void);

/**
 * @brief Enable or disable APLL
 *
 * Output frequency is given by the formula:
 * apll_freq = xtal_freq * (4 + sdm2 + sdm1/256 + sdm0/65536)/((o_div + 2) * 2)
 *
 * The dividend in this expression should be in the range of 240 - 600 MHz.
 *
 * In rev. 0 of ESP32, sdm0 and sdm1 are unused and always set to 0.
 *
 * @param enable  true to enable, false to disable
 */
void rtc_clk_apll_enable(bool enable);

/**
 * @brief Calculate APLL clock coeffifcients
 *
 * @param freq  expected APLL frequency
 * @param o_div  frequency divider, 0..31
 * @param sdm0  frequency adjustment parameter, 0..255
 * @param sdm1  frequency adjustment parameter, 0..255
 * @param sdm2  frequency adjustment parameter, 0..63
 *
 * @return
 *      - 0 Failed
 *      - else Success
 */
uint32_t rtc_clk_apll_coeff_calc(uint32_t freq, uint32_t *_o_div, uint32_t *_sdm0, uint32_t *_sdm1, uint32_t *_sdm2);

/**
 * @brief Set APLL clock coeffifcients
 *
 * @param o_div  frequency divider, 0..31
 * @param sdm0  frequency adjustment parameter, 0..255
 * @param sdm1  frequency adjustment parameter, 0..255
 * @param sdm2  frequency adjustment parameter, 0..63
 */
void rtc_clk_apll_coeff_set(uint32_t o_div, uint32_t sdm0, uint32_t sdm1, uint32_t sdm2);

/**
 * @brief Select source for RTC_SLOW_CLK
 * @param clk_src clock source (one of soc_rtc_slow_clk_src_t values)
 */
void rtc_clk_slow_src_set(soc_rtc_slow_clk_src_t clk_src);

/**
 * @brief Get the RTC_SLOW_CLK source
 * @return currently selected clock source (one of soc_rtc_slow_clk_src_t values)
 */
soc_rtc_slow_clk_src_t rtc_clk_slow_src_get(void);

/**
 * @brief Get the approximate frequency of RTC_SLOW_CLK, in Hz
 *
 * - if SOC_RTC_SLOW_CLK_SRC_RC_SLOW is selected, returns ~150000
 * - if SOC_RTC_SLOW_CLK_SRC_XTAL32K is selected, returns 32768
 * - if SOC_RTC_SLOW_CLK_SRC_RC_FAST_D256 is selected, returns ~33000
 *
 * rtc_clk_cal function can be used to get more precise value by comparing
 * RTC_SLOW_CLK frequency to the frequency of main XTAL.
 *
 * @return RTC_SLOW_CLK frequency, in Hz
 */
uint32_t rtc_clk_slow_freq_get_hz(void);

/**
 * @brief Select source for RTC_FAST_CLK
 * @param clk_src clock source (one of soc_rtc_fast_clk_src_t values)
 */
void rtc_clk_fast_src_set(soc_rtc_fast_clk_src_t clk_src);

/**
 * @brief Get the RTC_FAST_CLK source
 * @return currently selected clock source (one of soc_rtc_fast_clk_src_t values)
 */
soc_rtc_fast_clk_src_t rtc_clk_fast_src_get(void);

 /**
  * @brief Get CPU frequency config for a given frequency
  * @param freq_mhz  Frequency in MHz
  * @param[out] out_config Output, CPU frequency configuration structure
  * @return true if frequency can be obtained, false otherwise
  */
 bool rtc_clk_cpu_freq_mhz_to_config(uint32_t freq_mhz, rtc_cpu_freq_config_t* out_config);

 /**
  * @brief Switch CPU frequency
  *
  * This function sets CPU frequency according to the given configuration
  * structure. It enables PLLs, if necessary.
  *
  * @note This function in not intended to be called by applications in FreeRTOS
  * environment. This is because it does not adjust various timers based on the
  * new CPU frequency.
  *
  * @param config  CPU frequency configuration structure
  */
 void rtc_clk_cpu_freq_set_config(const rtc_cpu_freq_config_t* config);

 /**
  * @brief Switch CPU frequency (optimized for speed)
  *
  * This function is a faster equivalent of rtc_clk_cpu_freq_set_config.
  * It works faster because it does not disable PLLs when switching from PLL to
  * XTAL and does not enabled them when switching back. If PLL is not already
  * enabled when this function is called to switch from XTAL to PLL frequency,
  * or the PLL which is enabled is the wrong one, this function will fall back
  * to calling rtc_clk_cpu_freq_set_config.
  *
  * Unlike rtc_clk_cpu_freq_set_config, this function relies on static data,
  * so it is less safe to use it e.g. from a panic handler (when memory might
  * be corrupted).
  *
  * @note This function in not intended to be called by applications in FreeRTOS
  * environment. This is because it does not adjust various timers based on the
  * new CPU frequency.
  *
  * @param config  CPU frequency configuration structure
  */
 void rtc_clk_cpu_freq_set_config_fast(const rtc_cpu_freq_config_t* config);

 /**
  * @brief Get the currently used CPU frequency configuration
  * @param[out] out_config  Output, CPU frequency configuration structure
  */
 void rtc_clk_cpu_freq_get_config(rtc_cpu_freq_config_t* out_config);

 /**
  * @brief Switch CPU clock source to XTAL
  *
  * Short form for filling in rtc_cpu_freq_config_t structure and calling
  * rtc_clk_cpu_freq_set_config when a switch to XTAL is needed.
  * Assumes that XTAL frequency has been determined — don't call in startup code.
  */
 void rtc_clk_cpu_freq_set_xtal(void);


/**
 * @brief Store new APB frequency value into RTC_APB_FREQ_REG
 *
 * This function doesn't change any hardware clocks.
 *
 * Functions which perform frequency switching and change APB frequency call
 * this function to update the value of APB frequency stored in RTC_APB_FREQ_REG
 * (one of RTC general purpose retention registers). This should not normally
 * be called from application code.
 *
 * @param apb_freq  new APB frequency, in Hz
 */
void rtc_clk_apb_freq_update(uint32_t apb_freq);

/**
 * @brief Get the current stored APB frequency.
 * @return The APB frequency value as last set via rtc_clk_apb_freq_update(), in Hz.
 */
uint32_t rtc_clk_apb_freq_get(void);

#define RTC_CLK_CAL_FRACT  19  //!< Number of fractional bits in values returned by rtc_clk_cal

/**
 * @brief Measure RTC slow clock's period, based on main XTAL frequency
 *
 * This function will time out and return 0 if the time for the given number
 * of cycles to be counted exceeds the expected time twice. This may happen if
 * 32k XTAL is being calibrated, but the oscillator has not started up (due to
 * incorrect loading capacitance, board design issue, or lack of 32 XTAL on board).
 *
 * @note When 32k CLK is being calibrated, this function will check the accuracy
 * of the clock. Since the xtal 32k or ext osc 32k is generally very stable, if
 * the check fails, then consider this an invalid 32k clock and return 0. This
 * check can filter some jamming signal.
 *
 * @param cal_clk  clock to be measured
 * @param slow_clk_cycles  number of slow clock cycles to average
 * @return average slow clock period in microseconds, Q13.19 fixed point format,
 *         or 0 if calibration has timed out
 */
uint32_t rtc_clk_cal(rtc_cal_sel_t cal_clk, uint32_t slow_clk_cycles);

/**
 * @brief Measure ratio between XTAL frequency and RTC slow clock frequency
 * @param cal_clk slow clock to be measured
 * @param slow_clk_cycles number of slow clock cycles to average
 * @return average ratio between XTAL frequency and slow clock frequency,
 *         Q13.19 fixed point format, or 0 if calibration has timed out.
 */
uint32_t rtc_clk_cal_ratio(rtc_cal_sel_t cal_clk, uint32_t slow_clk_cycles);

/**
 * @brief Convert time interval from microseconds to RTC_SLOW_CLK cycles
 * @param time_in_us Time interval in microseconds
 * @param slow_clk_period  Period of slow clock in microseconds, Q13.19
 *                         fixed point format (as returned by rtc_slowck_cali).
 * @return number of slow clock cycles
 */
uint64_t rtc_time_us_to_slowclk(uint64_t time_in_us, uint32_t period);

/**
 * @brief Convert time interval from RTC_SLOW_CLK to microseconds
 * @param time_in_us Time interval in RTC_SLOW_CLK cycles
 * @param slow_clk_period  Period of slow clock in microseconds, Q13.19
 *                         fixed point format (as returned by rtc_slowck_cali).
 * @return time interval in microseconds
 */
uint64_t rtc_time_slowclk_to_us(uint64_t rtc_cycles, uint32_t period);

/**
 * @brief Get current value of RTC counter
 *
 * RTC has a 48-bit counter which is incremented by 2 every 2 RTC_SLOW_CLK
 * cycles. Counter value is not writable by software. The value is not adjusted
 * when switching to a different RTC_SLOW_CLK source.
 *
 * Note: this function may take up to 1 RTC_SLOW_CLK cycle to execute
 *
 * @return current value of RTC counter
 */
uint64_t rtc_time_get(void);

/**
 * @brief Busy loop until next RTC_SLOW_CLK cycle
 *
 * This function returns not earlier than the next RTC_SLOW_CLK clock cycle.
 * In some cases (e.g. when RTC_SLOW_CLK cycle is very close), it may return
 * one RTC_SLOW_CLK cycle later.
 */
void rtc_clk_wait_for_slow_cycle(void);

/**
 * @brief Enable the rtc digital 8M clock
 *
 * This function is used to enable the digital rtc 8M clock to support peripherals.
 * For enabling the analog 8M clock, using `rtc_clk_8M_enable` function above.
 */
void rtc_dig_clk8m_enable(void);

/**
 * @brief Disable the rtc digital 8M clock
 *
 * This function is used to disable the digital rtc 8M clock, which is only used to support peripherals.
 */
void rtc_dig_clk8m_disable(void);

/**
 * @brief Get whether the rtc digital 8M clock is enabled
 */
bool rtc_dig_8m_enabled(void);

/**
 * @brief Calculate the real clock value after the clock calibration
 *
 * @param cal_val Average slow clock period in microseconds, fixed point value as returned from `rtc_clk_cal`
 * @return Frequency of the clock in Hz
 */
uint32_t rtc_clk_freq_cal(uint32_t cal_val);

/**
 * @brief sleep configuration for rtc_sleep_init function
 */
typedef struct rtc_sleep_config_s {
    uint32_t lslp_mem_inf_fpu : 1;      //!< force normal voltage in sleep mode (digital domain memory)
    uint32_t rtc_mem_inf_fpu : 1;       //!< force normal voltage in sleep mode (RTC memory)
    uint32_t rtc_mem_inf_follow_cpu : 1;//!< keep low voltage in sleep mode (even if ULP/touch is used)
    uint32_t rtc_fastmem_pd_en : 1;     //!< power down RTC fast memory
    uint32_t rtc_slowmem_pd_en : 1;     //!< power down RTC slow memory
    uint32_t rtc_peri_pd_en : 1;        //!< power down RTC peripherals
    uint32_t modem_pd_en : 1;           //!< power down Modem(wifi and btdm)
    uint32_t int_8m_pd_en : 1;          //!< Power down Internal 8M oscillator
    uint32_t rom_mem_pd_en : 1;         //!< power down main RAM and ROM
    uint32_t deep_slp : 1;              //!< power down digital domain
    uint32_t wdt_flashboot_mod_en : 1;  //!< enable WDT flashboot mode
    uint32_t dig_dbias_wak : 3;         //!< set bias for digital domain, in active mode
    uint32_t dig_dbias_slp : 3;         //!< set bias for digital domain, in sleep mode
    uint32_t rtc_dbias_wak : 3;         //!< set bias for RTC domain, in active mode
    uint32_t rtc_dbias_slp : 3;         //!< set bias for RTC domain, in sleep mode
    uint32_t lslp_meminf_pd : 1;        //!< remove all peripheral force power up flags
    uint32_t vddsdio_pd_en : 1;         //!< power down VDDSDIO regulator
    uint32_t xtal_fpu : 1;              //!< keep main XTAL powered up in sleep
    uint32_t deep_slp_reject : 1;       //!< enable deep sleep reject
    uint32_t light_slp_reject : 1;      //!< enable light sleep reject
    uint32_t dbg_atten_slp : 2;             //!< voltage parameter
} rtc_sleep_config_t;

#define RTC_SLEEP_PD_DIG                BIT(0)  //!< Deep sleep (power down digital domain)
#define RTC_SLEEP_PD_RTC_PERIPH         BIT(1)  //!< Power down RTC peripherals
#define RTC_SLEEP_PD_RTC_SLOW_MEM       BIT(2)  //!< Power down RTC SLOW memory
#define RTC_SLEEP_PD_RTC_FAST_MEM       BIT(3)  //!< Power down RTC FAST memory
#define RTC_SLEEP_PD_RTC_MEM_FOLLOW_CPU BIT(4)  //!< RTC FAST and SLOW memories are automatically powered up and down along with the CPU
#define RTC_SLEEP_PD_VDDSDIO            BIT(5)  //!< Power down VDDSDIO regulator
#define RTC_SLEEP_PD_MODEM              BIT(6)  //!< Power down Modem(wifi and btdm)
#define RTC_SLEEP_PD_XTAL               BIT(7)  //!< Power down main XTAL
#define RTC_SLEEP_PD_INT_8M             BIT(8)  //!< Power down Internal 8M oscillator

//These flags are not power domains, but will affect some sleep parameters
#define RTC_SLEEP_DIG_USE_8M            BIT(16)
#define RTC_SLEEP_USE_ADC_TESEN_MONITOR BIT(17)
#define RTC_SLEEP_NO_ULTRA_LOW          BIT(18) //!< Avoid using ultra low power in deep sleep, in which RTCIO cannot be used as input, and RTCMEM can't work under high temperature
#define RTC_SLEEP_XTAL_AS_RTC_FAST      BIT(19)
/**
 * Default initializer for rtc_sleep_config_t
 *
 * This initializer sets all fields to "reasonable" values (e.g. suggested for
 * production use) based on a combination of RTC_SLEEP_PD_x flags.
 *
 * @param RTC_SLEEP_PD_x flags combined using bitwise OR
 */
void rtc_sleep_get_default_config(uint32_t sleep_flags, rtc_sleep_config_t *out_config);

/* Various delays to be programmed into power control state machines */
#define RTC_CNTL_XTL_BUF_WAIT_SLP_US        (1000)
#define RTC_CNTL_PLL_BUF_WAIT_SLP_CYCLES    (1)
#define RTC_CNTL_CK8M_WAIT_SLP_CYCLES       (4)
#define RTC_CNTL_WAKEUP_DELAY_CYCLES        (7)
#define RTC_CNTL_OTHER_BLOCKS_POWERUP_CYCLES    (1)
#define RTC_CNTL_OTHER_BLOCKS_WAIT_CYCLES       (1)
#define RTC_CNTL_MIN_SLP_VAL_MIN            (128)

#define RTC_CNTL_CK8M_WAIT_DEFAULT          20      // Equivalent macro as `CLK_LL_RC_FAST_WAIT_DEFAULT`
#define RTC_CK8M_ENABLE_WAIT_DEFAULT        5       // Equivalent macro as `CLK_LL_RC_FAST_ENABLE_WAIT_DEFAULT`

/**
 * @brief Prepare the chip to enter sleep mode
 *
 * This function configures various power control state machines to handle
 * entry into light sleep or deep sleep mode, switches APB and CPU clock source
 * (usually to XTAL), and sets bias voltages for digital and RTC power domains.
 *
 * This function does not actually enter sleep mode; this is done using
 * rtc_sleep_start function. Software may do some other actions between
 * rtc_sleep_init and rtc_sleep_start, such as set wakeup timer and configure
 * wakeup sources.
 * @param cfg sleep mode configuration
 */
void rtc_sleep_init(rtc_sleep_config_t cfg);

/**
 * @brief Low level initialize for rtc state machine waiting cycles after waking up
 *
 * This function configures the cycles chip need to wait for internal 8MHz
 * oscillator and external 40MHz crystal. As we configure fixed time for waiting
 * crystal, we need to pass period to calculate cycles. Now this function only
 * used in lightsleep mode.
 *
 * @param slowclk_period re-calibrated slow clock period
 * @param dslp true if initialize for deepsleep request
 */
void rtc_sleep_low_init(uint32_t slowclk_period, bool dslp);

#define RTC_EXT0_TRIG_EN    BIT(0)  //!< EXT0 GPIO wakeup
#define RTC_EXT1_TRIG_EN    BIT(1)  //!< EXT1 GPIO wakeup
#define RTC_GPIO_TRIG_EN    BIT(2)  //!< GPIO wakeup (light sleep only)
#define RTC_TIMER_TRIG_EN   BIT(3)  //!< Timer wakeup
#define RTC_SDIO_TRIG_EN    BIT(4)  //!< SDIO wakeup (light sleep only)
#define RTC_MAC_TRIG_EN     BIT(5)  //!< MAC wakeup (light sleep only)
#define RTC_UART0_TRIG_EN   BIT(6)  //!< UART0 wakeup (light sleep only)
#define RTC_UART1_TRIG_EN   BIT(7)  //!< UART1 wakeup (light sleep only)
#define RTC_TOUCH_TRIG_EN   BIT(8)  //!< Touch wakeup
#define RTC_ULP_TRIG_EN     BIT(9)  //!< ULP wakeup
#define RTC_BT_TRIG_EN      BIT(10) //!< BT wakeup (light sleep only)

/**
 * RTC_SLEEP_REJECT_MASK records sleep reject sources supported by chip
 * esp32 only supports GPIO and SDIO sleep reject sources
 */
#define RTC_SLEEP_REJECT_MASK (RTC_GPIO_TRIG_EN | RTC_SDIO_TRIG_EN)

/**
 * @brief Enter deep or light sleep mode
 *
 * This function enters the sleep mode previously configured using rtc_sleep_init
 * function. Before entering sleep, software should configure wake up sources
 * appropriately (set up GPIO wakeup registers, timer wakeup registers,
 * and so on).
 *
 * If deep sleep mode was configured using rtc_sleep_init, and sleep is not
 * rejected by hardware (based on reject_opt flags), this function never returns.
 * When the chip wakes up from deep sleep, CPU is reset and execution starts
 * from ROM bootloader.
 *
 * If light sleep mode was configured using rtc_sleep_init, this function
 * returns on wakeup, or if sleep is rejected by hardware.
 *
 * @param wakeup_opt  bit mask wake up reasons to enable (RTC_xxx_TRIG_EN flags
 *                    combined with OR)
 * @param reject_opt  bit mask of sleep reject reasons:
 *                      - RTC_CNTL_GPIO_REJECT_EN
 *                      - RTC_CNTL_SDIO_REJECT_EN
 *                    These flags are used to prevent entering sleep when e.g.
 *                    an external host is communicating via SDIO slave
 * @return non-zero if sleep was rejected by hardware
 */
uint32_t rtc_sleep_start(uint32_t wakeup_opt, uint32_t reject_opt);

/**
 * @brief Enter deep sleep mode
 *
 * Similar to rtc_sleep_start(), but additionally uses hardware to calculate the CRC value
 * of RTC FAST memory. On wake, this CRC is used to determine if a deep sleep wake
 * stub is valid to execute (if a wake address is set).
 *
 * No RAM is accessed while calculating the CRC and going into deep sleep, which makes
 * this function safe to use even if the caller's stack is in RTC FAST memory.
 *
 * @note If no deep sleep wake stub address is set then calling rtc_sleep_start() will
 * have the same effect and takes less time as CRC calculation is skipped.
 *
 * @note This function should only be called after rtc_sleep_init() has been called to
 * configure the system for deep sleep.
 *
 * @param wakeup_opt - same as for rtc_sleep_start
 * @param reject_opt - same as for rtc_sleep_start
 *
 * @return non-zero if sleep was rejected by hardware
 */
uint32_t rtc_deep_sleep_start(uint32_t wakeup_opt, uint32_t reject_opt);

/**
 * RTC power and clock control initialization settings
 */
typedef struct rtc_config_s {
    uint32_t ck8m_wait : 8;         //!< Number of rtc_fast_clk cycles to wait for 8M clock to be ready
    uint32_t xtal_wait : 8;         //!< Number of rtc_fast_clk cycles to wait for XTAL clock to be ready
    uint32_t pll_wait : 8;          //!< Number of rtc_fast_clk cycles to wait for PLL to be ready
    uint32_t clkctl_init : 1;       //!< Perform clock control related initialization
    uint32_t pwrctl_init : 1;       //!< Perform power control related initialization
    uint32_t rtc_dboost_fpd : 1;    //!< Force power down RTC_DBOOST
} rtc_config_t;

/**
 * Default initializer of rtc_config_t.
 *
 * This initializer sets all fields to "reasonable" values (e.g. suggested for
 * production use).
 */
#define RTC_CONFIG_DEFAULT() {\
    .ck8m_wait = RTC_CNTL_CK8M_WAIT_DEFAULT, \
    .xtal_wait = RTC_CNTL_XTL_BUF_WAIT_DEFAULT, \
    .pll_wait  = RTC_CNTL_PLL_BUF_WAIT_DEFAULT, \
    .clkctl_init = 1, \
    .pwrctl_init = 1, \
    .rtc_dboost_fpd = 1 \
}

/**
 * Initialize RTC clock and power control related functions
 * @param cfg configuration options as rtc_config_t
 */
void rtc_init(rtc_config_t cfg);

#define RTC_VDDSDIO_TIEH_1_8V 0 //!< TIEH field value for 1.8V VDDSDIO
#define RTC_VDDSDIO_TIEH_3_3V 1 //!< TIEH field value for 3.3V VDDSDIO

/**
 * Structure describing vddsdio configuration
 */
typedef struct rtc_vddsdio_config_s {
    uint32_t force : 1;     //!< If 1, use configuration from RTC registers; if 0, use EFUSE/bootstrapping pins.
    uint32_t enable : 1;    //!< Enable VDDSDIO regulator
    uint32_t tieh  : 1;     //!< Select VDDSDIO voltage. One of RTC_VDDSDIO_TIEH_1_8V, RTC_VDDSDIO_TIEH_3_3V
    uint32_t drefh : 2;     //!< Tuning parameter for VDDSDIO regulator
    uint32_t drefm : 2;     //!< Tuning parameter for VDDSDIO regulator
    uint32_t drefl : 2;     //!< Tuning parameter for VDDSDIO regulator
} rtc_vddsdio_config_t;

/**
 * Get current VDDSDIO configuration
 * If VDDSDIO configuration is overridden by RTC, get values from RTC
 * Otherwise, if VDDSDIO is configured by EFUSE, get values from EFUSE
 * Otherwise, use default values and the level of MTDI bootstrapping pin.
 * @return currently used VDDSDIO configuration
 */
rtc_vddsdio_config_t rtc_vddsdio_get_config(void);

/**
 * Set new VDDSDIO configuration using RTC registers.
 * If config.force == 1, this overrides configuration done using bootstrapping
 * pins and EFUSE.
 *
 * @param config new VDDSDIO configuration
 */
void rtc_vddsdio_set_config(rtc_vddsdio_config_t config);


// -------------------------- CLOCK TREE DEFS ALIAS ----------------------------
// **WARNING**: The following are only for backwards compatibility.
// Please use the declarations in soc/clk_tree_defs.h instead.
/**
 * @brief CPU clock source
 */
typedef soc_cpu_clk_src_t rtc_cpu_freq_src_t;
#define RTC_CPU_FREQ_SRC_XTAL SOC_CPU_CLK_SRC_XTAL  //!< XTAL
#define RTC_CPU_FREQ_SRC_PLL SOC_CPU_CLK_SRC_PLL    //!< PLL (480M or 320M)
#define RTC_CPU_FREQ_SRC_8M SOC_CPU_CLK_SRC_RC_FAST //!< Internal 8M RTC oscillator
#define RTC_CPU_FREQ_SRC_APLL SOC_CPU_CLK_SRC_APLL  //!< APLL

/**
 * @brief RTC SLOW_CLK frequency values
 */
typedef soc_rtc_slow_clk_src_t rtc_slow_freq_t;
#define RTC_SLOW_FREQ_RTC SOC_RTC_SLOW_CLK_SRC_RC_SLOW         //!< Internal 150 kHz RC oscillator
#define RTC_SLOW_FREQ_32K_XTAL SOC_RTC_SLOW_CLK_SRC_XTAL32K    //!< External 32 kHz XTAL
#define RTC_SLOW_FREQ_8MD256 SOC_RTC_SLOW_CLK_SRC_RC_FAST_D256 //!< Internal 8 MHz RC oscillator, divided by 256

/**
 * @brief RTC FAST_CLK frequency values
 */
typedef soc_rtc_fast_clk_src_t rtc_fast_freq_t;
#define RTC_FAST_FREQ_XTALD4 SOC_RTC_FAST_CLK_SRC_XTAL_DIV  //!< Main XTAL, divided by 4
#define RTC_FAST_FREQ_8M SOC_RTC_FAST_CLK_SRC_RC_FAST       //!< Internal 8 MHz RC oscillator

/**
 * @brief Possible main XTAL frequency values.
 */
typedef soc_xtal_freq_t rtc_xtal_freq_t;
#define RTC_XTAL_FREQ_AUTO SOC_XTAL_FREQ_AUTO               //!< Automatic XTAL frequency detection
#define RTC_XTAL_FREQ_40M SOC_XTAL_FREQ_40M                 //!< 40 MHz XTAL
#define RTC_XTAL_FREQ_26M SOC_XTAL_FREQ_26M                 //!< 26 MHz XTAL
#define RTC_XTAL_FREQ_24M SOC_XTAL_FREQ_24M                 //!< 24 MHz XTAL

/* Alias of frequency related macros */
#define RTC_FAST_CLK_FREQ_APPROX    SOC_CLK_RC_FAST_FREQ_APPROX
#define RTC_FAST_CLK_FREQ_8M        SOC_CLK_RC_FAST_FREQ_APPROX
#define RTC_SLOW_CLK_FREQ_150K      SOC_CLK_RC_SLOW_FREQ_APPROX
#define RTC_SLOW_CLK_FREQ_8MD256    SOC_CLK_RC_FAST_D256_FREQ_APPROX
#define RTC_SLOW_CLK_FREQ_32K       SOC_CLK_XTAL32K_FREQ_APPROX

/* Alias of deprecated function names */
#define rtc_clk_slow_freq_set(slow_freq) rtc_clk_slow_src_set(slow_freq)
#define rtc_clk_slow_freq_get() rtc_clk_slow_src_get()
#define rtc_clk_fast_freq_set(fast_freq) rtc_clk_fast_src_set(fast_freq)
#define rtc_clk_fast_freq_get() rtc_clk_fast_src_get()

#ifdef __cplusplus
}
#endif
