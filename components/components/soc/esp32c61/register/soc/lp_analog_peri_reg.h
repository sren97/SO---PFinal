/**
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 *  SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "soc/soc.h"
#ifdef __cplusplus
extern "C" {
#endif

/** LP_ANA_BOD_MODE0_CNTL_REG register
 *  need_des
 */
#define LP_ANA_BOD_MODE0_CNTL_REG (DR_REG_LP_ANA_BASE + 0x0)
/** LP_ANA_BOD_MODE0_CLOSE_FLASH_ENA : R/W; bitpos: [6]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_CLOSE_FLASH_ENA    (BIT(6))
#define LP_ANA_BOD_MODE0_CLOSE_FLASH_ENA_M  (LP_ANA_BOD_MODE0_CLOSE_FLASH_ENA_V << LP_ANA_BOD_MODE0_CLOSE_FLASH_ENA_S)
#define LP_ANA_BOD_MODE0_CLOSE_FLASH_ENA_V  0x00000001U
#define LP_ANA_BOD_MODE0_CLOSE_FLASH_ENA_S  6
/** LP_ANA_BOD_MODE0_PD_RF_ENA : R/W; bitpos: [7]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_PD_RF_ENA    (BIT(7))
#define LP_ANA_BOD_MODE0_PD_RF_ENA_M  (LP_ANA_BOD_MODE0_PD_RF_ENA_V << LP_ANA_BOD_MODE0_PD_RF_ENA_S)
#define LP_ANA_BOD_MODE0_PD_RF_ENA_V  0x00000001U
#define LP_ANA_BOD_MODE0_PD_RF_ENA_S  7
/** LP_ANA_BOD_MODE0_INTR_WAIT : R/W; bitpos: [17:8]; default: 1;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_INTR_WAIT    0x000003FFU
#define LP_ANA_BOD_MODE0_INTR_WAIT_M  (LP_ANA_BOD_MODE0_INTR_WAIT_V << LP_ANA_BOD_MODE0_INTR_WAIT_S)
#define LP_ANA_BOD_MODE0_INTR_WAIT_V  0x000003FFU
#define LP_ANA_BOD_MODE0_INTR_WAIT_S  8
/** LP_ANA_BOD_MODE0_RESET_WAIT : R/W; bitpos: [27:18]; default: 1023;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_RESET_WAIT    0x000003FFU
#define LP_ANA_BOD_MODE0_RESET_WAIT_M  (LP_ANA_BOD_MODE0_RESET_WAIT_V << LP_ANA_BOD_MODE0_RESET_WAIT_S)
#define LP_ANA_BOD_MODE0_RESET_WAIT_V  0x000003FFU
#define LP_ANA_BOD_MODE0_RESET_WAIT_S  18
/** LP_ANA_BOD_MODE0_CNT_CLR : R/W; bitpos: [28]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_CNT_CLR    (BIT(28))
#define LP_ANA_BOD_MODE0_CNT_CLR_M  (LP_ANA_BOD_MODE0_CNT_CLR_V << LP_ANA_BOD_MODE0_CNT_CLR_S)
#define LP_ANA_BOD_MODE0_CNT_CLR_V  0x00000001U
#define LP_ANA_BOD_MODE0_CNT_CLR_S  28
/** LP_ANA_BOD_MODE0_INTR_ENA : R/W; bitpos: [29]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_INTR_ENA    (BIT(29))
#define LP_ANA_BOD_MODE0_INTR_ENA_M  (LP_ANA_BOD_MODE0_INTR_ENA_V << LP_ANA_BOD_MODE0_INTR_ENA_S)
#define LP_ANA_BOD_MODE0_INTR_ENA_V  0x00000001U
#define LP_ANA_BOD_MODE0_INTR_ENA_S  29
/** LP_ANA_BOD_MODE0_RESET_SEL : R/W; bitpos: [30]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_RESET_SEL    (BIT(30))
#define LP_ANA_BOD_MODE0_RESET_SEL_M  (LP_ANA_BOD_MODE0_RESET_SEL_V << LP_ANA_BOD_MODE0_RESET_SEL_S)
#define LP_ANA_BOD_MODE0_RESET_SEL_V  0x00000001U
#define LP_ANA_BOD_MODE0_RESET_SEL_S  30
/** LP_ANA_BOD_MODE0_RESET_ENA : R/W; bitpos: [31]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_RESET_ENA    (BIT(31))
#define LP_ANA_BOD_MODE0_RESET_ENA_M  (LP_ANA_BOD_MODE0_RESET_ENA_V << LP_ANA_BOD_MODE0_RESET_ENA_S)
#define LP_ANA_BOD_MODE0_RESET_ENA_V  0x00000001U
#define LP_ANA_BOD_MODE0_RESET_ENA_S  31

/** LP_ANA_BOD_MODE1_CNTL_REG register
 *  need_des
 */
#define LP_ANA_BOD_MODE1_CNTL_REG (DR_REG_LP_ANA_BASE + 0x4)
/** LP_ANA_BOD_MODE1_RESET_ENA : R/W; bitpos: [31]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE1_RESET_ENA    (BIT(31))
#define LP_ANA_BOD_MODE1_RESET_ENA_M  (LP_ANA_BOD_MODE1_RESET_ENA_V << LP_ANA_BOD_MODE1_RESET_ENA_S)
#define LP_ANA_BOD_MODE1_RESET_ENA_V  0x00000001U
#define LP_ANA_BOD_MODE1_RESET_ENA_S  31

/** LP_ANA_POWER_GLITCH_CNTL_REG register
 *  need_des
 */
#define LP_ANA_POWER_GLITCH_CNTL_REG (DR_REG_LP_ANA_BASE + 0x8)
/** LP_ANA_POWER_GLITCH_RESET_ENA : R/W; bitpos: [31:28]; default: 0;
 *  need_des
 */
#define LP_ANA_POWER_GLITCH_RESET_ENA    0x0000000FU
#define LP_ANA_POWER_GLITCH_RESET_ENA_M  (LP_ANA_POWER_GLITCH_RESET_ENA_V << LP_ANA_POWER_GLITCH_RESET_ENA_S)
#define LP_ANA_POWER_GLITCH_RESET_ENA_V  0x0000000FU
#define LP_ANA_POWER_GLITCH_RESET_ENA_S  28

/** LP_ANA_FIB_ENABLE_REG register
 *  need_des
 */
#define LP_ANA_FIB_ENABLE_REG (DR_REG_LP_ANA_BASE + 0xc)
/** LP_ANA_ANA_FIB_ENA : R/W; bitpos: [31:0]; default: 4294967295;
 *  need_des
 */
#define LP_ANA_ANA_FIB_ENA    0xFFFFFFFFU
#define LP_ANA_ANA_FIB_ENA_M  (LP_ANA_ANA_FIB_ENA_V << LP_ANA_ANA_FIB_ENA_S)
#define LP_ANA_ANA_FIB_ENA_V  0xFFFFFFFFU
#define LP_ANA_ANA_FIB_ENA_S  0

#define LP_ANA_ANA_FIB_PWR_GLITCH_ENA    0x0000000FU
#define LP_ANA_ANA_FIB_PWR_GLITCH_ENA_M  (LP_ANA_ANA_FIB_PWR_GLITCH_ENA_V << LP_ANA_ANA_FIB_PWR_GLITCH_ENA_S)
#define LP_ANA_ANA_FIB_PWR_GLITCH_ENA_V  0x0000000FU
#define LP_ANA_ANA_FIB_PWR_GLITCH_ENA_S  2

/** LP_ANA_INT_RAW_REG register
 *  need_des
 */
#define LP_ANA_INT_RAW_REG (DR_REG_LP_ANA_BASE + 0x10)
/** LP_ANA_BOD_MODE0_INT_RAW : R/WTC/SS; bitpos: [31]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_INT_RAW    (BIT(31))
#define LP_ANA_BOD_MODE0_INT_RAW_M  (LP_ANA_BOD_MODE0_INT_RAW_V << LP_ANA_BOD_MODE0_INT_RAW_S)
#define LP_ANA_BOD_MODE0_INT_RAW_V  0x00000001U
#define LP_ANA_BOD_MODE0_INT_RAW_S  31

/** LP_ANA_INT_ST_REG register
 *  need_des
 */
#define LP_ANA_INT_ST_REG (DR_REG_LP_ANA_BASE + 0x14)
/** LP_ANA_BOD_MODE0_INT_ST : RO; bitpos: [31]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_INT_ST    (BIT(31))
#define LP_ANA_BOD_MODE0_INT_ST_M  (LP_ANA_BOD_MODE0_INT_ST_V << LP_ANA_BOD_MODE0_INT_ST_S)
#define LP_ANA_BOD_MODE0_INT_ST_V  0x00000001U
#define LP_ANA_BOD_MODE0_INT_ST_S  31

/** LP_ANA_INT_ENA_REG register
 *  need_des
 */
#define LP_ANA_INT_ENA_REG (DR_REG_LP_ANA_BASE + 0x18)
/** LP_ANA_BOD_MODE0_INT_ENA : R/W; bitpos: [31]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_INT_ENA    (BIT(31))
#define LP_ANA_BOD_MODE0_INT_ENA_M  (LP_ANA_BOD_MODE0_INT_ENA_V << LP_ANA_BOD_MODE0_INT_ENA_S)
#define LP_ANA_BOD_MODE0_INT_ENA_V  0x00000001U
#define LP_ANA_BOD_MODE0_INT_ENA_S  31

/** LP_ANA_INT_CLR_REG register
 *  need_des
 */
#define LP_ANA_INT_CLR_REG (DR_REG_LP_ANA_BASE + 0x1c)
/** LP_ANA_BOD_MODE0_INT_CLR : WT; bitpos: [31]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_INT_CLR    (BIT(31))
#define LP_ANA_BOD_MODE0_INT_CLR_M  (LP_ANA_BOD_MODE0_INT_CLR_V << LP_ANA_BOD_MODE0_INT_CLR_S)
#define LP_ANA_BOD_MODE0_INT_CLR_V  0x00000001U
#define LP_ANA_BOD_MODE0_INT_CLR_S  31

/** LP_ANA_LP_INT_RAW_REG register
 *  need_des
 */
#define LP_ANA_LP_INT_RAW_REG (DR_REG_LP_ANA_BASE + 0x20)
/** LP_ANA_BOD_MODE0_LP_INT_RAW : R/WTC/SS; bitpos: [31]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_LP_INT_RAW    (BIT(31))
#define LP_ANA_BOD_MODE0_LP_INT_RAW_M  (LP_ANA_BOD_MODE0_LP_INT_RAW_V << LP_ANA_BOD_MODE0_LP_INT_RAW_S)
#define LP_ANA_BOD_MODE0_LP_INT_RAW_V  0x00000001U
#define LP_ANA_BOD_MODE0_LP_INT_RAW_S  31

/** LP_ANA_LP_INT_ST_REG register
 *  need_des
 */
#define LP_ANA_LP_INT_ST_REG (DR_REG_LP_ANA_BASE + 0x24)
/** LP_ANA_BOD_MODE0_LP_INT_ST : RO; bitpos: [31]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_LP_INT_ST    (BIT(31))
#define LP_ANA_BOD_MODE0_LP_INT_ST_M  (LP_ANA_BOD_MODE0_LP_INT_ST_V << LP_ANA_BOD_MODE0_LP_INT_ST_S)
#define LP_ANA_BOD_MODE0_LP_INT_ST_V  0x00000001U
#define LP_ANA_BOD_MODE0_LP_INT_ST_S  31

/** LP_ANA_LP_INT_ENA_REG register
 *  need_des
 */
#define LP_ANA_LP_INT_ENA_REG (DR_REG_LP_ANA_BASE + 0x28)
/** LP_ANA_BOD_MODE0_LP_INT_ENA : R/W; bitpos: [31]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_LP_INT_ENA    (BIT(31))
#define LP_ANA_BOD_MODE0_LP_INT_ENA_M  (LP_ANA_BOD_MODE0_LP_INT_ENA_V << LP_ANA_BOD_MODE0_LP_INT_ENA_S)
#define LP_ANA_BOD_MODE0_LP_INT_ENA_V  0x00000001U
#define LP_ANA_BOD_MODE0_LP_INT_ENA_S  31

/** LP_ANA_LP_INT_CLR_REG register
 *  need_des
 */
#define LP_ANA_LP_INT_CLR_REG (DR_REG_LP_ANA_BASE + 0x2c)
/** LP_ANA_BOD_MODE0_LP_INT_CLR : WT; bitpos: [31]; default: 0;
 *  need_des
 */
#define LP_ANA_BOD_MODE0_LP_INT_CLR    (BIT(31))
#define LP_ANA_BOD_MODE0_LP_INT_CLR_M  (LP_ANA_BOD_MODE0_LP_INT_CLR_V << LP_ANA_BOD_MODE0_LP_INT_CLR_S)
#define LP_ANA_BOD_MODE0_LP_INT_CLR_V  0x00000001U
#define LP_ANA_BOD_MODE0_LP_INT_CLR_S  31

/** LP_ANA_DATE_REG register
 *  need_des
 */
#define LP_ANA_DATE_REG (DR_REG_LP_ANA_BASE + 0x3fc)
/** LP_ANA_LP_ANA_DATE : R/W; bitpos: [30:0]; default: 37752896;
 *  need_des
 */
#define LP_ANA_LP_ANA_DATE    0x7FFFFFFFU
#define LP_ANA_LP_ANA_DATE_M  (LP_ANA_LP_ANA_DATE_V << LP_ANA_LP_ANA_DATE_S)
#define LP_ANA_LP_ANA_DATE_V  0x7FFFFFFFU
#define LP_ANA_LP_ANA_DATE_S  0
/** LP_ANA_CLK_EN : R/W; bitpos: [31]; default: 0;
 *  need_des
 */
#define LP_ANA_CLK_EN    (BIT(31))
#define LP_ANA_CLK_EN_M  (LP_ANA_CLK_EN_V << LP_ANA_CLK_EN_S)
#define LP_ANA_CLK_EN_V  0x00000001U
#define LP_ANA_CLK_EN_S  31

#ifdef __cplusplus
}
#endif
