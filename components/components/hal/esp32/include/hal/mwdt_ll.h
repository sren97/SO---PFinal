/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// The LL layer for Timer Group register operations.
// Note that most of the register operations in this layer are non-atomic operations.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "hal/misc.h"
#include "hal/assert.h"
#include "soc/wdt_periph.h"
#include "soc/timer_group_struct.h"
#include "hal/wdt_types.h"
#include "esp_attr.h"
#include "esp_assert.h"

/* Pre-calculated prescaler to achieve 500 ticks/us (MWDT1_TICKS_PER_US) when using default clock (MWDT_CLK_SRC_DEFAULT ) */
#define MWDT_LL_DEFAULT_CLK_PRESCALER 40000

/* Possible values for TIMG_WDT_STGx */
#define TIMG_WDT_STG_SEL_OFF 0
#define TIMG_WDT_STG_SEL_INT 1
#define TIMG_WDT_STG_SEL_RESET_CPU 2
#define TIMG_WDT_STG_SEL_RESET_SYSTEM 3

/* Possible values for TIMG_WDT_CPU_RESET_LENGTH and TIMG_WDT_SYS_RESET_LENGTH */
#define TIMG_WDT_RESET_LENGTH_100_NS    0
#define TIMG_WDT_RESET_LENGTH_200_NS    1
#define TIMG_WDT_RESET_LENGTH_300_NS    2
#define TIMG_WDT_RESET_LENGTH_400_NS    3
#define TIMG_WDT_RESET_LENGTH_500_NS    4
#define TIMG_WDT_RESET_LENGTH_800_NS    5
#define TIMG_WDT_RESET_LENGTH_1600_NS   6
#define TIMG_WDT_RESET_LENGTH_3200_NS   7

//Type check wdt_stage_action_t
ESP_STATIC_ASSERT(WDT_STAGE_ACTION_OFF == TIMG_WDT_STG_SEL_OFF, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_stage_action_t");
ESP_STATIC_ASSERT(WDT_STAGE_ACTION_INT == TIMG_WDT_STG_SEL_INT, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_stage_action_t");
ESP_STATIC_ASSERT(WDT_STAGE_ACTION_RESET_CPU == TIMG_WDT_STG_SEL_RESET_CPU, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_stage_action_t");
ESP_STATIC_ASSERT(WDT_STAGE_ACTION_RESET_SYSTEM == TIMG_WDT_STG_SEL_RESET_SYSTEM, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_stage_action_t");
//Type check wdt_reset_sig_length_t
ESP_STATIC_ASSERT(WDT_RESET_SIG_LENGTH_100ns == TIMG_WDT_RESET_LENGTH_100_NS, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_reset_sig_length_t");
ESP_STATIC_ASSERT(WDT_RESET_SIG_LENGTH_200ns == TIMG_WDT_RESET_LENGTH_200_NS, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_reset_sig_length_t");
ESP_STATIC_ASSERT(WDT_RESET_SIG_LENGTH_300ns == TIMG_WDT_RESET_LENGTH_300_NS, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_reset_sig_length_t");
ESP_STATIC_ASSERT(WDT_RESET_SIG_LENGTH_400ns == TIMG_WDT_RESET_LENGTH_400_NS, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_reset_sig_length_t");
ESP_STATIC_ASSERT(WDT_RESET_SIG_LENGTH_500ns == TIMG_WDT_RESET_LENGTH_500_NS, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_reset_sig_length_t");
ESP_STATIC_ASSERT(WDT_RESET_SIG_LENGTH_800ns == TIMG_WDT_RESET_LENGTH_800_NS, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_reset_sig_length_t");
ESP_STATIC_ASSERT(WDT_RESET_SIG_LENGTH_1_6us == TIMG_WDT_RESET_LENGTH_1600_NS, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_reset_sig_length_t");
ESP_STATIC_ASSERT(WDT_RESET_SIG_LENGTH_3_2us == TIMG_WDT_RESET_LENGTH_3200_NS, "Add mapping to LL watchdog timeout behavior, since it's no longer naturally compatible with wdt_reset_sig_length_t");

/**
 * @brief Enable the MWDT
 *
 * @param hw Start address of the peripheral registers.
 */
FORCE_INLINE_ATTR void mwdt_ll_enable(timg_dev_t *hw)
{
    hw->wdtconfig0.wdt_en = 1;
}

/**
 * @brief Disable the MWDT
 *
 * @param hw Start address of the peripheral registers.
 * @note This function does not disable the flashboot mode. Therefore, given that
 *       the MWDT is disabled using this function, a timeout can still occur
 *       if the flashboot mode is simultaneously enabled.
 */
FORCE_INLINE_ATTR void mwdt_ll_disable(timg_dev_t *hw)
{
    hw->wdtconfig0.wdt_en = 0;
}

/**
 * Check if the MWDT is enabled
 *
 * @param hw Start address of the peripheral registers.
 * @return True if the MWDT is enabled, false otherwise
 */
FORCE_INLINE_ATTR bool mwdt_ll_check_if_enabled(timg_dev_t *hw)
{
    return (hw->wdtconfig0.wdt_en) ? true : false;
}

/**
 * @brief Configure a particular stage of the MWDT
 *
 * @param hw Start address of the peripheral registers.
 * @param stage Which stage to configure
 * @param timeout Number of timer ticks for the stage to timeout
 * @param behavior What action to take when the stage times out
 */
FORCE_INLINE_ATTR void mwdt_ll_config_stage(timg_dev_t *hw, wdt_stage_t stage, uint32_t timeout, wdt_stage_action_t behavior)
{
    switch (stage) {
    case WDT_STAGE0:
        hw->wdtconfig0.wdt_stg0 = behavior;
        hw->wdtconfig2.wdt_stg0_hold = timeout;
        break;
    case WDT_STAGE1:
        hw->wdtconfig0.wdt_stg1 = behavior;
        hw->wdtconfig3.wdt_stg1_hold = timeout;
        break;
    case WDT_STAGE2:
        hw->wdtconfig0.wdt_stg2 = behavior;
        hw->wdtconfig4.wdt_stg2_hold = timeout;
        break;
    case WDT_STAGE3:
        hw->wdtconfig0.wdt_stg3 = behavior;
        hw->wdtconfig5.wdt_stg3_hold = timeout;
        break;
    default:
        HAL_ASSERT(false && "unsupported WDT stage");
        break;
    }
}

/**
 * @brief Disable a particular stage of the MWDT
 *
 * @param hw Start address of the peripheral registers.
 * @param stage Which stage to disable
 */
FORCE_INLINE_ATTR void mwdt_ll_disable_stage(timg_dev_t *hw, uint32_t stage)
{
    switch (stage) {
    case WDT_STAGE0:
        hw->wdtconfig0.wdt_stg0 = WDT_STAGE_ACTION_OFF;
        break;
    case WDT_STAGE1:
        hw->wdtconfig0.wdt_stg1 = WDT_STAGE_ACTION_OFF;
        break;
    case WDT_STAGE2:
        hw->wdtconfig0.wdt_stg2 = WDT_STAGE_ACTION_OFF;
        break;
    case WDT_STAGE3:
        hw->wdtconfig0.wdt_stg3 = WDT_STAGE_ACTION_OFF;
        break;
    default:
        HAL_ASSERT(false && "unsupported WDT stage");
        break;
    }
}

/**
 * @brief Enable or disable MWDT edge interrupt
 *
 * @param hw Start address of the peripheral registers.
 * @param enable Whether to enable edge interrupt
 */
FORCE_INLINE_ATTR void mwdt_ll_set_edge_intr(timg_dev_t *hw, bool enable)
{
    hw->wdtconfig0.wdt_edge_int_en = enable;
}

/**
 * @brief Enable or disable MWDT level interrupt
 *
 * @param hw Start address of the peripheral registers.
 * @param enable Whether to enable level interrupt
 */
FORCE_INLINE_ATTR void mwdt_ll_set_level_intr(timg_dev_t *hw, bool enable)
{
    hw->wdtconfig0.wdt_level_int_en = enable;
}

/**
 * @brief Set the length of the CPU reset action
 *
 * @param hw Start address of the peripheral registers.
 * @param length Length of CPU reset signal
 */
FORCE_INLINE_ATTR void mwdt_ll_set_cpu_reset_length(timg_dev_t *hw, wdt_reset_sig_length_t length)
{
    hw->wdtconfig0.wdt_cpu_reset_length = length;
}

/**
 * @brief Set the length of the system reset action
 *
 * @param hw Start address of the peripheral registers.
 * @param length Length of system reset signal
 */
FORCE_INLINE_ATTR void mwdt_ll_set_sys_reset_length(timg_dev_t *hw, wdt_reset_sig_length_t length)
{
    hw->wdtconfig0.wdt_sys_reset_length = length;
}

/**
 * @brief Enable/Disable the MWDT flashboot mode.
 *
 * @param hw Beginning address of the peripheral registers.
 * @param enable True to enable WDT flashboot mode, false to disable WDT flashboot mode.
 *
 * @note Flashboot mode is independent and can trigger a WDT timeout event if the
 *       WDT's enable bit is set to 0. Flashboot mode for TG0 is automatically enabled
 *       on flashboot, and should be disabled by software when flashbooting completes.
 */
FORCE_INLINE_ATTR void mwdt_ll_set_flashboot_en(timg_dev_t *hw, bool enable)
{
    hw->wdtconfig0.wdt_flashboot_mod_en = (enable) ? 1 : 0;
}

/**
 * @brief Set the clock prescaler of the MWDT
 *
 * @param hw Start address of the peripheral registers.
 * @param prescaler Prescaler value between 1 to 65535
 */
FORCE_INLINE_ATTR void mwdt_ll_set_prescaler(timg_dev_t *hw, uint32_t prescaler)
{
    HAL_FORCE_MODIFY_U32_REG_FIELD(hw->wdtconfig1, wdt_clk_prescaler, prescaler);
}

/**
 * @brief Feed the MWDT
 *
 * Resets the current timer count and current stage.
 *
 * @param hw Start address of the peripheral registers.
 */
FORCE_INLINE_ATTR void mwdt_ll_feed(timg_dev_t *hw)
{
    hw->wdtfeed.wdt_feed = 1;
}

/**
 * @brief Enable write protection of the MWDT registers
 *
 * @param hw Start address of the peripheral registers.
 */
FORCE_INLINE_ATTR void mwdt_ll_write_protect_enable(timg_dev_t *hw)
{
    hw->wdtwprotect.wdt_wkey = 0;
}

/**
 * @brief Disable write protection of the MWDT registers
 *
 * @param hw Start address of the peripheral registers.
 */
FORCE_INLINE_ATTR void mwdt_ll_write_protect_disable(timg_dev_t *hw)
{
    hw->wdtwprotect.wdt_wkey = TIMG_WDT_WKEY_VALUE;
}

/**
 * @brief Clear the MWDT interrupt status.
 *
 * @param hw Start address of the peripheral registers.
 */
FORCE_INLINE_ATTR void mwdt_ll_clear_intr_status(timg_dev_t *hw)
{
    hw->int_clr_timers.wdt_int_clr = 1;
}

/**
 * @brief Set the interrupt enable bit for the MWDT interrupt.
 *
 * @param hw Beginning address of the peripheral registers.
 * @param enable Whether to enable the MWDT interrupt
 */
FORCE_INLINE_ATTR void mwdt_ll_set_intr_enable(timg_dev_t *hw, bool enable)
{
    hw->int_ena_timers.wdt_int_ena = enable;
}

/**
 * @brief Set the clock source for the MWDT.
 *
 * @param hw Beginning address of the peripheral registers.
 * @param clk_src Clock source
 */
FORCE_INLINE_ATTR void mwdt_ll_set_clock_source(timg_dev_t *hw, mwdt_clock_source_t clk_src)
{
    /* No clk source option on ESP32, always use APB as clock source */
    (void)hw;
    (void)clk_src;

    HAL_ASSERT(clk_src == MWDT_CLK_SRC_APB);
}

/**
 * @brief Enable MWDT module clock
 *
 * @param hw Beginning address of the peripheral registers.
 * @param en true to enable, false to disable
 */
__attribute__((always_inline))
static inline void mwdt_ll_enable_clock(timg_dev_t *hw, bool en)
{
    /* No MWDT specific clock bit on ESP32 */
    (void)hw;
    (void)en;
}


#ifdef __cplusplus
}
#endif
