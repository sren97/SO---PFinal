/**
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 *  SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include "soc/soc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_ANA_MST_I2C0_CTRL_REG          (DR_REG_I2C_ANA_MST_BASE + 0x0)
/* I2C_ANA_MST_I2C0_BUSY : RO ;bitpos:[25] ;default: 1'h0 ; */
/*description: .*/
#define I2C_ANA_MST_I2C0_BUSY    (BIT(25))
#define I2C_ANA_MST_I2C0_BUSY_M  (BIT(25))
#define I2C_ANA_MST_I2C0_BUSY_V  0x1
#define I2C_ANA_MST_I2C0_BUSY_S  25
/* I2C_ANA_MST_I2C0_CTRL : R/W ;bitpos:[24:0] ;default: 25'h0 ; */
/*description: .*/
#define I2C_ANA_MST_I2C0_CTRL    0x01FFFFFF
#define I2C_ANA_MST_I2C0_CTRL_M  ((I2C_ANA_MST_I2C0_CTRL_V)<<(I2C_ANA_MST_I2C0_CTRL_S))
#define I2C_ANA_MST_I2C0_CTRL_V  0x1FFFFFF
#define I2C_ANA_MST_I2C0_CTRL_S  0

#define I2C_ANA_MST_I2C1_CTRL_REG          (DR_REG_I2C_ANA_MST_BASE + 0x4)
/* I2C_ANA_MST_I2C1_BUSY : RO ;bitpos:[25] ;default: 1'h0 ; */
/*description: .*/
#define I2C_ANA_MST_I2C1_BUSY    (BIT(25))
#define I2C_ANA_MST_I2C1_BUSY_M  (BIT(25))
#define I2C_ANA_MST_I2C1_BUSY_V  0x1
#define I2C_ANA_MST_I2C1_BUSY_S  25
/* I2C_ANA_MST_I2C1_CTRL : R/W ;bitpos:[24:0] ;default: 25'h0 ; */
/*description: .*/
#define I2C_ANA_MST_I2C1_CTRL    0x01FFFFFF
#define I2C_ANA_MST_I2C1_CTRL_M  ((I2C_ANA_MST_I2C1_CTRL_V)<<(I2C_ANA_MST_I2C1_CTRL_S))
#define I2C_ANA_MST_I2C1_CTRL_V  0x1FFFFFF
#define I2C_ANA_MST_I2C1_CTRL_S  0

#define I2C_ANA_MST_I2C0_CONF_REG          (DR_REG_I2C_ANA_MST_BASE + 0x8)
/* I2C_ANA_MST_I2C0_STATUS : RO ;bitpos:[31:24] ;default: 8'h0 ; */
/*description: .*/
#define I2C_ANA_MST_I2C0_STATUS    0x000000FF
#define I2C_ANA_MST_I2C0_STATUS_M  ((I2C_ANA_MST_I2C0_STATUS_V)<<(I2C_ANA_MST_I2C0_STATUS_S))
#define I2C_ANA_MST_I2C0_STATUS_V  0xFF
#define I2C_ANA_MST_I2C0_STATUS_S  24
/* I2C_ANA_MST_I2C0_CONF : R/W ;bitpos:[23:0] ;default: 24'h0 ; */
/*description: .*/
#define I2C_ANA_MST_I2C0_CONF    0x00FFFFFF
#define I2C_ANA_MST_I2C0_CONF_M  ((I2C_ANA_MST_I2C0_CONF_V)<<(I2C_ANA_MST_I2C0_CONF_S))
#define I2C_ANA_MST_I2C0_CONF_V  0xFFFFFF
#define I2C_ANA_MST_I2C0_CONF_S  0

#define I2C_ANA_MST_I2C1_CONF_REG          (DR_REG_I2C_ANA_MST_BASE + 0xC)
/* I2C_ANA_MST_I2C1_STATUS : RO ;bitpos:[31:24] ;default: 8'h0 ; */
/*description: .*/
#define I2C_ANA_MST_I2C1_STATUS    0x000000FF
#define I2C_ANA_MST_I2C1_STATUS_M  ((I2C_ANA_MST_I2C1_STATUS_V)<<(I2C_ANA_MST_I2C1_STATUS_S))
#define I2C_ANA_MST_I2C1_STATUS_V  0xFF
#define I2C_ANA_MST_I2C1_STATUS_S  24
/* I2C_ANA_MST_I2C1_CONF : R/W ;bitpos:[23:0] ;default: 24'h0 ; */
/*description: .*/
#define I2C_ANA_MST_I2C1_CONF    0x00FFFFFF
#define I2C_ANA_MST_I2C1_CONF_M  ((I2C_ANA_MST_I2C1_CONF_V)<<(I2C_ANA_MST_I2C1_CONF_S))
#define I2C_ANA_MST_I2C1_CONF_V  0xFFFFFF
#define I2C_ANA_MST_I2C1_CONF_S  0

#define I2C_ANA_MST_I2C_BURST_CONF_REG          (DR_REG_I2C_ANA_MST_BASE + 0x10)
/* I2C_ANA_MST_BURST_CTRL : R/W ;bitpos:[31:0] ;default: 32'h0 ; */
/*description: .*/
#define I2C_ANA_MST_BURST_CTRL    0xFFFFFFFF
#define I2C_ANA_MST_BURST_CTRL_M  ((I2C_ANA_MST_BURST_CTRL_V)<<(I2C_ANA_MST_BURST_CTRL_S))
#define I2C_ANA_MST_BURST_CTRL_V  0xFFFFFFFF
#define I2C_ANA_MST_BURST_CTRL_S  0

#define I2C_ANA_MST_I2C_BURST_STATUS_REG          (DR_REG_I2C_ANA_MST_BASE + 0x14)
/* I2C_ANA_MST_BURST_TIMEOUT_CNT : R/W ;bitpos:[31:20] ;default: 12'h400 ; */
/*description: .*/
#define I2C_ANA_MST_BURST_TIMEOUT_CNT    0x00000FFF
#define I2C_ANA_MST_BURST_TIMEOUT_CNT_M  ((I2C_ANA_MST_BURST_TIMEOUT_CNT_V)<<(I2C_ANA_MST_BURST_TIMEOUT_CNT_S))
#define I2C_ANA_MST_BURST_TIMEOUT_CNT_V  0xFFF
#define I2C_ANA_MST_BURST_TIMEOUT_CNT_S  20
/* I2C_ANA_MST1_BURST_ERR_FLAG : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: .*/
#define I2C_ANA_MST1_BURST_ERR_FLAG    (BIT(2))
#define I2C_ANA_MST1_BURST_ERR_FLAG_M  (BIT(2))
#define I2C_ANA_MST1_BURST_ERR_FLAG_V  0x1
#define I2C_ANA_MST1_BURST_ERR_FLAG_S  2
/* I2C_ANA_MST0_BURST_ERR_FLAG : RO ;bitpos:[1] ;default: 1'b0 ; */
/*description: .*/
#define I2C_ANA_MST0_BURST_ERR_FLAG    (BIT(1))
#define I2C_ANA_MST0_BURST_ERR_FLAG_M  (BIT(1))
#define I2C_ANA_MST0_BURST_ERR_FLAG_V  0x1
#define I2C_ANA_MST0_BURST_ERR_FLAG_S  1
/* I2C_ANA_MST_BURST_DONE : RO ;bitpos:[0] ;default: 1'b0 ; */
/*description: .*/
#define I2C_ANA_MST_BURST_DONE    (BIT(0))
#define I2C_ANA_MST_BURST_DONE_M  (BIT(0))
#define I2C_ANA_MST_BURST_DONE_V  0x1
#define I2C_ANA_MST_BURST_DONE_S  0

#define I2C_ANA_MST_ANA_CONF0_REG          (DR_REG_I2C_ANA_MST_BASE + 0x18)
/* I2C_ANA_MST_STATUS0 : RO ;bitpos:[31:24] ;default: 8'h0 ; */
/*description: .*/
#define I2C_ANA_MST_ANA_STATUS0    0x000000FF
#define I2C_ANA_MST_ANA_STATUS0_M  ((I2C_ANA_MST_STATUS0_V)<<(I2C_ANA_MST_STATUS0_S))
#define I2C_ANA_MST_ANA_STATUS0_V  0xFF
#define I2C_ANA_MST_ANA_STATUS0_S  24
/* I2C_ANA_MST_ANA_CONF0 : R/W ;bitpos:[23:0] ;default: 24'h00_e408 ; */
/*description: .*/
#define I2C_ANA_MST_ANA_CONF0    0x00FFFFFF
#define I2C_ANA_MST_ANA_CONF0_M  ((I2C_ANA_MST_ANA_CONF0_V)<<(I2C_ANA_MST_ANA_CONF0_S))
#define I2C_ANA_MST_ANA_CONF0_V  0xFFFFFF
#define I2C_ANA_MST_ANA_CONF0_S  0

#define I2C_ANA_MST_ANA_CONF1_REG          (DR_REG_I2C_ANA_MST_BASE + 0x1C)
/* I2C_ANA_MST_STATUS1 : RO ;bitpos:[31:24] ;default: 8'h0 ; */
/*description: .*/
#define I2C_ANA_MST_ANA_STATUS1    0x000000FF
#define I2C_ANA_MST_ANA_STATUS1_M  ((I2C_ANA_MST_STATUS1_V)<<(I2C_ANA_MST_STATUS1_S))
#define I2C_ANA_MST_ANA_STATUS1_V  0xFF
#define I2C_ANA_MST_ANA_STATUS1_S  24
/* I2C_ANA_MST_ANA_CONF1 : R/W ;bitpos:[23:0] ;default: 24'h00_002d ; */
/*description: .*/
#define I2C_ANA_MST_ANA_CONF1    0x00FFFFFF
#define I2C_ANA_MST_ANA_CONF1_M  ((I2C_ANA_MST_ANA_CONF1_V)<<(I2C_ANA_MST_ANA_CONF1_S))
#define I2C_ANA_MST_ANA_CONF1_V  0xFFFFFF
#define I2C_ANA_MST_ANA_CONF1_S  0

#define I2C_ANA_MST_ANA_CONF2_REG          (DR_REG_I2C_ANA_MST_BASE + 0x20)
/* I2C_ANA_MST_STATUS2 : RO ;bitpos:[31:24] ;default: 8'h0 ; */
/*description: .*/
#define I2C_ANA_MST_ANA_STATUS2    0x000000FF
#define I2C_ANA_MST_ANA_STATUS2_M  ((I2C_ANA_MST_STATUS2_V)<<(I2C_ANA_MST_STATUS2_S))
#define I2C_ANA_MST_ANA_STATUS2_V  0xFF
#define I2C_ANA_MST_ANA_STATUS2_S  24
/* I2C_ANA_MST_ANA_CONF2 : R/W ;bitpos:[23:0] ;default: 24'h00_0004 ; */
/*description: .*/
#define I2C_ANA_MST_ANA_CONF2    0x00FFFFFF
#define I2C_ANA_MST_ANA_CONF2_M  ((I2C_ANA_MST_ANA_CONF2_V)<<(I2C_ANA_MST_ANA_CONF2_S))
#define I2C_ANA_MST_ANA_CONF2_V  0xFFFFFF
#define I2C_ANA_MST_ANA_CONF2_S  0

#define I2C_ANA_MST_I2C0_CTRL1_REG          (DR_REG_I2C_ANA_MST_BASE + 0x24)
/* I2C_ANA_MST_I2C0_SDA_SIDE_GUARD : R/W ;bitpos:[10:6] ;default: 5'h1 ; */
/*description: .*/
#define I2C_ANA_MST_I2C0_SDA_SIDE_GUARD    0x0000001F
#define I2C_ANA_MST_I2C0_SDA_SIDE_GUARD_M  ((I2C_ANA_MST_I2C0_SDA_SIDE_GUARD_V)<<(I2C_ANA_MST_I2C0_SDA_SIDE_GUARD_S))
#define I2C_ANA_MST_I2C0_SDA_SIDE_GUARD_V  0x1F
#define I2C_ANA_MST_I2C0_SDA_SIDE_GUARD_S  6
/* I2C_ANA_MST_I2C0_SCL_PULSE_DUR : R/W ;bitpos:[5:0] ;default: 6'h2 ; */
/*description: .*/
#define I2C_ANA_MST_I2C0_SCL_PULSE_DUR    0x0000003F
#define I2C_ANA_MST_I2C0_SCL_PULSE_DUR_M  ((I2C_ANA_MST_I2C0_SCL_PULSE_DUR_V)<<(I2C_ANA_MST_I2C0_SCL_PULSE_DUR_S))
#define I2C_ANA_MST_I2C0_SCL_PULSE_DUR_V  0x3F
#define I2C_ANA_MST_I2C0_SCL_PULSE_DUR_S  0

#define I2C_ANA_MST_I2C1_CTRL1_REG          (DR_REG_I2C_ANA_MST_BASE + 0x28)
/* I2C_ANA_MST_I2C1_SDA_SIDE_GUARD : R/W ;bitpos:[10:6] ;default: 5'h1 ; */
/*description: .*/
#define I2C_ANA_MST_I2C1_SDA_SIDE_GUARD    0x0000001F
#define I2C_ANA_MST_I2C1_SDA_SIDE_GUARD_M  ((I2C_ANA_MST_I2C1_SDA_SIDE_GUARD_V)<<(I2C_ANA_MST_I2C1_SDA_SIDE_GUARD_S))
#define I2C_ANA_MST_I2C1_SDA_SIDE_GUARD_V  0x1F
#define I2C_ANA_MST_I2C1_SDA_SIDE_GUARD_S  6
/* I2C_ANA_MST_I2C1_SCL_PULSE_DUR : R/W ;bitpos:[5:0] ;default: 6'h2 ; */
/*description: .*/
#define I2C_ANA_MST_I2C1_SCL_PULSE_DUR    0x0000003F
#define I2C_ANA_MST_I2C1_SCL_PULSE_DUR_M  ((I2C_ANA_MST_I2C1_SCL_PULSE_DUR_V)<<(I2C_ANA_MST_I2C1_SCL_PULSE_DUR_S))
#define I2C_ANA_MST_I2C1_SCL_PULSE_DUR_V  0x3F
#define I2C_ANA_MST_I2C1_SCL_PULSE_DUR_S  0

#define I2C_ANA_MST_HW_I2C_CTRL_REG          (DR_REG_I2C_ANA_MST_BASE + 0x2C)
/* I2C_ANA_MST_ARBITER_DIS : R/W ;bitpos:[11] ;default: 1'h0 ; */
/*description: .*/
#define I2C_ANA_MST_ARBITER_DIS    (BIT(11))
#define I2C_ANA_MST_ARBITER_DIS_M  (BIT(11))
#define I2C_ANA_MST_ARBITER_DIS_V  0x1
#define I2C_ANA_MST_ARBITER_DIS_S  11
/* I2C_ANA_MST_HW_I2C_SDA_SIDE_GUARD : R/W ;bitpos:[10:6] ;default: 5'h1 ; */
/*description: .*/
#define I2C_ANA_MST_HW_I2C_SDA_SIDE_GUARD    0x0000001F
#define I2C_ANA_MST_HW_I2C_SDA_SIDE_GUARD_M  ((I2C_ANA_MST_HW_I2C_SDA_SIDE_GUARD_V)<<(I2C_ANA_MST_HW_I2C_SDA_SIDE_GUARD_S))
#define I2C_ANA_MST_HW_I2C_SDA_SIDE_GUARD_V  0x1F
#define I2C_ANA_MST_HW_I2C_SDA_SIDE_GUARD_S  6
/* I2C_ANA_MST_HW_I2C_SCL_PULSE_DUR : R/W ;bitpos:[5:0] ;default: 6'h2 ; */
/*description: .*/
#define I2C_ANA_MST_HW_I2C_SCL_PULSE_DUR    0x0000003F
#define I2C_ANA_MST_HW_I2C_SCL_PULSE_DUR_M  ((I2C_ANA_MST_HW_I2C_SCL_PULSE_DUR_V)<<(I2C_ANA_MST_HW_I2C_SCL_PULSE_DUR_S))
#define I2C_ANA_MST_HW_I2C_SCL_PULSE_DUR_V  0x3F
#define I2C_ANA_MST_HW_I2C_SCL_PULSE_DUR_S  0

#define I2C_ANA_MST_NOUSE_REG          (DR_REG_I2C_ANA_MST_BASE + 0x30)
/* I2C_ANA_MST_NOUSE : R/W ;bitpos:[31:0] ;default: 32'h0 ; */
/*description: .*/
#define I2C_ANA_MST_NOUSE    0xFFFFFFFF
#define I2C_ANA_MST_NOUSE_M  ((I2C_ANA_MST_NOUSE_V)<<(I2C_ANA_MST_NOUSE_S))
#define I2C_ANA_MST_NOUSE_V  0xFFFFFFFF
#define I2C_ANA_MST_NOUSE_S  0

#define I2C_ANA_MST_DATE_REG          (DR_REG_I2C_ANA_MST_BASE + 0x34)
/* I2C_ANA_MST_CLK_EN : R/W ;bitpos:[28] ;default: 1'h0 ; */
/*description: .*/
#define I2C_ANA_MST_CLK_EN    (BIT(28))
#define I2C_ANA_MST_CLK_EN_M  (BIT(28))
#define I2C_ANA_MST_CLK_EN_V  0x1
#define I2C_ANA_MST_CLK_EN_S  28
/* I2C_ANA_MST_DATE : R/W ;bitpos:[27:0] ;default: 28'h2210310 ; */
/*description: .*/
#define I2C_ANA_MST_DATE    0x0FFFFFFF
#define I2C_ANA_MST_DATE_M  ((I2C_ANA_MST_DATE_V)<<(I2C_ANA_MST_DATE_S))
#define I2C_ANA_MST_DATE_V  0xFFFFFFF
#define I2C_ANA_MST_DATE_S  0

#ifdef __cplusplus
}
#endif
