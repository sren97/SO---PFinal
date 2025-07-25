/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// The LL layer for ESP32-H2 LP_Timer register operations

#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include "soc/soc.h"
#include "soc/lp_timer_struct.h"
#include "soc/lp_aon_reg.h"
#include "hal/lp_timer_types.h"
#include "hal/misc.h"
#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

FORCE_INLINE_ATTR void lp_timer_ll_set_alarm_target(lp_timer_dev_t *dev, uint8_t timer_id, uint64_t value)
{
    HAL_FORCE_MODIFY_U32_REG_FIELD(dev->target[timer_id].hi, target_hi, (value >> 32) & 0xFFFF);
    HAL_FORCE_MODIFY_U32_REG_FIELD(dev->target[timer_id].lo, target_lo, value & 0xFFFFFFFF);
}

FORCE_INLINE_ATTR void lp_timer_ll_set_target_enable(lp_timer_dev_t *dev, uint8_t timer_id, bool en)
{
    dev->target[timer_id].hi.enable = en;
}

FORCE_INLINE_ATTR uint32_t lp_timer_ll_get_counter_value_low(lp_timer_dev_t *dev, uint8_t timer_id)
{
    return HAL_FORCE_READ_U32_REG_FIELD(dev->counter[timer_id].lo, counter_lo);
}

FORCE_INLINE_ATTR uint32_t lp_timer_ll_get_counter_value_high(lp_timer_dev_t *dev, uint8_t timer_id)
{
    return HAL_FORCE_READ_U32_REG_FIELD(dev->counter[timer_id].hi, counter_hi);
}

FORCE_INLINE_ATTR void lp_timer_ll_counter_snapshot(lp_timer_dev_t *dev)
{
    dev->update.update = 1;
}

FORCE_INLINE_ATTR void lp_timer_ll_clear_alarm_intr_status(lp_timer_dev_t *dev)
{
    dev->int_clr.alarm = 1;
}

FORCE_INLINE_ATTR void lp_timer_ll_clear_overflow_intr_status(lp_timer_dev_t *dev)
{
    dev->int_clr.overflow = 1;
}

#ifdef __cplusplus
}
#endif
