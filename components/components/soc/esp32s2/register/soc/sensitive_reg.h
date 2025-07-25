/*
 * SPDX-FileCopyrightText: 2017-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _SOC_SENSITIVE_REG_H_
#define _SOC_SENSITIVE_REG_H_


#ifdef __cplusplus
extern "C" {
#endif
#include "soc/soc.h"
#define DPORT_PMS_SDIO_0_REG          (DR_REG_SENSITIVE_BASE + 0x000)
/* DPORT_PMS_SDIO_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_SDIO_LOCK  (BIT(0))
#define DPORT_PMS_SDIO_LOCK_M  (BIT(0))
#define DPORT_PMS_SDIO_LOCK_V  0x1
#define DPORT_PMS_SDIO_LOCK_S  0

#define DPORT_PMS_SDIO_1_REG          (DR_REG_SENSITIVE_BASE + 0x004)
/* DPORT_PMS_SDIO_DISABLE : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_SDIO_DISABLE  (BIT(0))
#define DPORT_PMS_SDIO_DISABLE_M  (BIT(0))
#define DPORT_PMS_SDIO_DISABLE_V  0x1
#define DPORT_PMS_SDIO_DISABLE_S  0

#define DPORT_PMS_MAC_DUMP_0_REG          (DR_REG_SENSITIVE_BASE + 0x008)
/* DPORT_PMS_MAC_DUMP_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_MAC_DUMP_LOCK  (BIT(0))
#define DPORT_PMS_MAC_DUMP_LOCK_M  (BIT(0))
#define DPORT_PMS_MAC_DUMP_LOCK_V  0x1
#define DPORT_PMS_MAC_DUMP_LOCK_S  0

#define DPORT_PMS_MAC_DUMP_1_REG          (DR_REG_SENSITIVE_BASE + 0x00C)
/* DPORT_PMS_MAC_DUMP_CONNECT : R/W ;bitpos:[11:0] ;default: 12'b000011100100 ; */
/*description: */
#define DPORT_PMS_MAC_DUMP_CONNECT  0x00000FFF
#define DPORT_PMS_MAC_DUMP_CONNECT_M  ((DPORT_PMS_MAC_DUMP_CONNECT_V)<<(DPORT_PMS_MAC_DUMP_CONNECT_S))
#define DPORT_PMS_MAC_DUMP_CONNECT_V  0xFFF
#define DPORT_PMS_MAC_DUMP_CONNECT_S  0

#define DPORT_PMS_PRO_IRAM0_0_REG          (DR_REG_SENSITIVE_BASE + 0x010)
/* DPORT_PMS_PRO_IRAM0_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_LOCK  (BIT(0))
#define DPORT_PMS_PRO_IRAM0_LOCK_M  (BIT(0))
#define DPORT_PMS_PRO_IRAM0_LOCK_V  0x1
#define DPORT_PMS_PRO_IRAM0_LOCK_S  0

#define DPORT_PMS_PRO_IRAM0_1_REG          (DR_REG_SENSITIVE_BASE + 0x014)
/* DPORT_PMS_PRO_IRAM0_SRAM_3_W : R/W ;bitpos:[11] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_3_W  (BIT(11))
#define DPORT_PMS_PRO_IRAM0_SRAM_3_W_M  (BIT(11))
#define DPORT_PMS_PRO_IRAM0_SRAM_3_W_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_3_W_S  11
/* DPORT_PMS_PRO_IRAM0_SRAM_3_R : R/W ;bitpos:[10] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_3_R  (BIT(10))
#define DPORT_PMS_PRO_IRAM0_SRAM_3_R_M  (BIT(10))
#define DPORT_PMS_PRO_IRAM0_SRAM_3_R_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_3_R_S  10
/* DPORT_PMS_PRO_IRAM0_SRAM_3_F : R/W ;bitpos:[9] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_3_F  (BIT(9))
#define DPORT_PMS_PRO_IRAM0_SRAM_3_F_M  (BIT(9))
#define DPORT_PMS_PRO_IRAM0_SRAM_3_F_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_3_F_S  9
/* DPORT_PMS_PRO_IRAM0_SRAM_2_W : R/W ;bitpos:[8] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_2_W  (BIT(8))
#define DPORT_PMS_PRO_IRAM0_SRAM_2_W_M  (BIT(8))
#define DPORT_PMS_PRO_IRAM0_SRAM_2_W_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_2_W_S  8
/* DPORT_PMS_PRO_IRAM0_SRAM_2_R : R/W ;bitpos:[7] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_2_R  (BIT(7))
#define DPORT_PMS_PRO_IRAM0_SRAM_2_R_M  (BIT(7))
#define DPORT_PMS_PRO_IRAM0_SRAM_2_R_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_2_R_S  7
/* DPORT_PMS_PRO_IRAM0_SRAM_2_F : R/W ;bitpos:[6] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_2_F  (BIT(6))
#define DPORT_PMS_PRO_IRAM0_SRAM_2_F_M  (BIT(6))
#define DPORT_PMS_PRO_IRAM0_SRAM_2_F_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_2_F_S  6
/* DPORT_PMS_PRO_IRAM0_SRAM_1_W : R/W ;bitpos:[5] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_1_W  (BIT(5))
#define DPORT_PMS_PRO_IRAM0_SRAM_1_W_M  (BIT(5))
#define DPORT_PMS_PRO_IRAM0_SRAM_1_W_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_1_W_S  5
/* DPORT_PMS_PRO_IRAM0_SRAM_1_R : R/W ;bitpos:[4] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_1_R  (BIT(4))
#define DPORT_PMS_PRO_IRAM0_SRAM_1_R_M  (BIT(4))
#define DPORT_PMS_PRO_IRAM0_SRAM_1_R_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_1_R_S  4
/* DPORT_PMS_PRO_IRAM0_SRAM_1_F : R/W ;bitpos:[3] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_1_F  (BIT(3))
#define DPORT_PMS_PRO_IRAM0_SRAM_1_F_M  (BIT(3))
#define DPORT_PMS_PRO_IRAM0_SRAM_1_F_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_1_F_S  3
/* DPORT_PMS_PRO_IRAM0_SRAM_0_W : R/W ;bitpos:[2] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_0_W  (BIT(2))
#define DPORT_PMS_PRO_IRAM0_SRAM_0_W_M  (BIT(2))
#define DPORT_PMS_PRO_IRAM0_SRAM_0_W_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_0_W_S  2
/* DPORT_PMS_PRO_IRAM0_SRAM_0_R : R/W ;bitpos:[1] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_0_R  (BIT(1))
#define DPORT_PMS_PRO_IRAM0_SRAM_0_R_M  (BIT(1))
#define DPORT_PMS_PRO_IRAM0_SRAM_0_R_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_0_R_S  1
/* DPORT_PMS_PRO_IRAM0_SRAM_0_F : R/W ;bitpos:[0] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_0_F  (BIT(0))
#define DPORT_PMS_PRO_IRAM0_SRAM_0_F_M  (BIT(0))
#define DPORT_PMS_PRO_IRAM0_SRAM_0_F_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_0_F_S  0

#define DPORT_PMS_PRO_IRAM0_2_REG          (DR_REG_SENSITIVE_BASE + 0x018)
/* DPORT_PMS_PRO_IRAM0_SRAM_4_H_W : R/W ;bitpos:[22] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_W  (BIT(22))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_W_M  (BIT(22))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_W_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_W_S  22
/* DPORT_PMS_PRO_IRAM0_SRAM_4_H_R : R/W ;bitpos:[21] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_R  (BIT(21))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_R_M  (BIT(21))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_R_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_R_S  21
/* DPORT_PMS_PRO_IRAM0_SRAM_4_H_F : R/W ;bitpos:[20] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_F  (BIT(20))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_F_M  (BIT(20))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_F_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_4_H_F_S  20
/* DPORT_PMS_PRO_IRAM0_SRAM_4_L_W : R/W ;bitpos:[19] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_W  (BIT(19))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_W_M  (BIT(19))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_W_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_W_S  19
/* DPORT_PMS_PRO_IRAM0_SRAM_4_L_R : R/W ;bitpos:[18] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_R  (BIT(18))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_R_M  (BIT(18))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_R_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_R_S  18
/* DPORT_PMS_PRO_IRAM0_SRAM_4_L_F : R/W ;bitpos:[17] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_F  (BIT(17))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_F_M  (BIT(17))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_F_V  0x1
#define DPORT_PMS_PRO_IRAM0_SRAM_4_L_F_S  17
/* DPORT_PMS_PRO_IRAM0_SRAM_4_SPLTADDR : R/W ;bitpos:[16:0] ;default: 17'b0 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_SRAM_4_SPLTADDR  0x0001FFFF
#define DPORT_PMS_PRO_IRAM0_SRAM_4_SPLTADDR_M  ((DPORT_PMS_PRO_IRAM0_SRAM_4_SPLTADDR_V)<<(DPORT_PMS_PRO_IRAM0_SRAM_4_SPLTADDR_S))
#define DPORT_PMS_PRO_IRAM0_SRAM_4_SPLTADDR_V  0x1FFFF
#define DPORT_PMS_PRO_IRAM0_SRAM_4_SPLTADDR_S  0

#define DPORT_PMS_PRO_IRAM0_3_REG          (DR_REG_SENSITIVE_BASE + 0x01C)
/* DPORT_PMS_PRO_IRAM0_RTCFAST_H_W : R/W ;bitpos:[16] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_W  (BIT(16))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_W_M  (BIT(16))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_W_V  0x1
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_W_S  16
/* DPORT_PMS_PRO_IRAM0_RTCFAST_H_R : R/W ;bitpos:[15] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_R  (BIT(15))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_R_M  (BIT(15))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_R_V  0x1
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_R_S  15
/* DPORT_PMS_PRO_IRAM0_RTCFAST_H_F : R/W ;bitpos:[14] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_F  (BIT(14))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_F_M  (BIT(14))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_F_V  0x1
#define DPORT_PMS_PRO_IRAM0_RTCFAST_H_F_S  14
/* DPORT_PMS_PRO_IRAM0_RTCFAST_L_W : R/W ;bitpos:[13] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_W  (BIT(13))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_W_M  (BIT(13))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_W_V  0x1
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_W_S  13
/* DPORT_PMS_PRO_IRAM0_RTCFAST_L_R : R/W ;bitpos:[12] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_R  (BIT(12))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_R_M  (BIT(12))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_R_V  0x1
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_R_S  12
/* DPORT_PMS_PRO_IRAM0_RTCFAST_L_F : R/W ;bitpos:[11] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_F  (BIT(11))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_F_M  (BIT(11))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_F_V  0x1
#define DPORT_PMS_PRO_IRAM0_RTCFAST_L_F_S  11
/* DPORT_PMS_PRO_IRAM0_RTCFAST_SPLTADDR : R/W ;bitpos:[10:0] ;default: 11'b0 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_RTCFAST_SPLTADDR  0x000007FF
#define DPORT_PMS_PRO_IRAM0_RTCFAST_SPLTADDR_M  ((DPORT_PMS_PRO_IRAM0_RTCFAST_SPLTADDR_V)<<(DPORT_PMS_PRO_IRAM0_RTCFAST_SPLTADDR_S))
#define DPORT_PMS_PRO_IRAM0_RTCFAST_SPLTADDR_V  0x7FF
#define DPORT_PMS_PRO_IRAM0_RTCFAST_SPLTADDR_S  0

#define DPORT_PMS_PRO_IRAM0_4_REG          (DR_REG_SENSITIVE_BASE + 0x020)
/* DPORT_PMS_PRO_IRAM0_ILG_INTR : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_ILG_INTR  (BIT(2))
#define DPORT_PMS_PRO_IRAM0_ILG_INTR_M  (BIT(2))
#define DPORT_PMS_PRO_IRAM0_ILG_INTR_V  0x1
#define DPORT_PMS_PRO_IRAM0_ILG_INTR_S  2
/* DPORT_PMS_PRO_IRAM0_ILG_EN : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_ILG_EN  (BIT(1))
#define DPORT_PMS_PRO_IRAM0_ILG_EN_M  (BIT(1))
#define DPORT_PMS_PRO_IRAM0_ILG_EN_V  0x1
#define DPORT_PMS_PRO_IRAM0_ILG_EN_S  1
/* DPORT_PMS_PRO_IRAM0_ILG_CLR : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_ILG_CLR  (BIT(0))
#define DPORT_PMS_PRO_IRAM0_ILG_CLR_M  (BIT(0))
#define DPORT_PMS_PRO_IRAM0_ILG_CLR_V  0x1
#define DPORT_PMS_PRO_IRAM0_ILG_CLR_S  0

#define DPORT_PMS_PRO_IRAM0_5_REG          (DR_REG_SENSITIVE_BASE + 0x024)
/* DPORT_PMS_PRO_IRAM0_ILG_ST : RO ;bitpos:[21:0] ;default: 22'b0 ; */
/*description: */
#define DPORT_PMS_PRO_IRAM0_ILG_ST  0x003FFFFF
#define DPORT_PMS_PRO_IRAM0_ILG_ST_M  ((DPORT_PMS_PRO_IRAM0_ILG_ST_V)<<(DPORT_PMS_PRO_IRAM0_ILG_ST_S))
#define DPORT_PMS_PRO_IRAM0_ILG_ST_V  0x3FFFFF
#define DPORT_PMS_PRO_IRAM0_ILG_ST_S  0

#define DPORT_PMS_PRO_DRAM0_0_REG          (DR_REG_SENSITIVE_BASE + 0x028)
/* DPORT_PMS_PRO_DRAM0_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_LOCK  (BIT(0))
#define DPORT_PMS_PRO_DRAM0_LOCK_M  (BIT(0))
#define DPORT_PMS_PRO_DRAM0_LOCK_V  0x1
#define DPORT_PMS_PRO_DRAM0_LOCK_S  0

#define DPORT_PMS_PRO_DRAM0_1_REG          (DR_REG_SENSITIVE_BASE + 0x02C)
/* DPORT_PMS_PRO_DRAM0_SRAM_4_H_W : R/W ;bitpos:[28] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_4_H_W  (BIT(28))
#define DPORT_PMS_PRO_DRAM0_SRAM_4_H_W_M  (BIT(28))
#define DPORT_PMS_PRO_DRAM0_SRAM_4_H_W_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_4_H_W_S  28
/* DPORT_PMS_PRO_DRAM0_SRAM_4_H_R : R/W ;bitpos:[27] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_4_H_R  (BIT(27))
#define DPORT_PMS_PRO_DRAM0_SRAM_4_H_R_M  (BIT(27))
#define DPORT_PMS_PRO_DRAM0_SRAM_4_H_R_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_4_H_R_S  27
/* DPORT_PMS_PRO_DRAM0_SRAM_4_L_W : R/W ;bitpos:[26] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_4_L_W  (BIT(26))
#define DPORT_PMS_PRO_DRAM0_SRAM_4_L_W_M  (BIT(26))
#define DPORT_PMS_PRO_DRAM0_SRAM_4_L_W_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_4_L_W_S  26
/* DPORT_PMS_PRO_DRAM0_SRAM_4_L_R : R/W ;bitpos:[25] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_4_L_R  (BIT(25))
#define DPORT_PMS_PRO_DRAM0_SRAM_4_L_R_M  (BIT(25))
#define DPORT_PMS_PRO_DRAM0_SRAM_4_L_R_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_4_L_R_S  25
/* DPORT_PMS_PRO_DRAM0_SRAM_4_SPLTADDR : R/W ;bitpos:[24:8] ;default: 17'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_4_SPLTADDR  0x0001FFFF
#define DPORT_PMS_PRO_DRAM0_SRAM_4_SPLTADDR_M  ((DPORT_PMS_PRO_DRAM0_SRAM_4_SPLTADDR_V)<<(DPORT_PMS_PRO_DRAM0_SRAM_4_SPLTADDR_S))
#define DPORT_PMS_PRO_DRAM0_SRAM_4_SPLTADDR_V  0x1FFFF
#define DPORT_PMS_PRO_DRAM0_SRAM_4_SPLTADDR_S  8
/* DPORT_PMS_PRO_DRAM0_SRAM_3_W : R/W ;bitpos:[7] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_3_W  (BIT(7))
#define DPORT_PMS_PRO_DRAM0_SRAM_3_W_M  (BIT(7))
#define DPORT_PMS_PRO_DRAM0_SRAM_3_W_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_3_W_S  7
/* DPORT_PMS_PRO_DRAM0_SRAM_3_R : R/W ;bitpos:[6] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_3_R  (BIT(6))
#define DPORT_PMS_PRO_DRAM0_SRAM_3_R_M  (BIT(6))
#define DPORT_PMS_PRO_DRAM0_SRAM_3_R_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_3_R_S  6
/* DPORT_PMS_PRO_DRAM0_SRAM_2_W : R/W ;bitpos:[5] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_2_W  (BIT(5))
#define DPORT_PMS_PRO_DRAM0_SRAM_2_W_M  (BIT(5))
#define DPORT_PMS_PRO_DRAM0_SRAM_2_W_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_2_W_S  5
/* DPORT_PMS_PRO_DRAM0_SRAM_2_R : R/W ;bitpos:[4] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_2_R  (BIT(4))
#define DPORT_PMS_PRO_DRAM0_SRAM_2_R_M  (BIT(4))
#define DPORT_PMS_PRO_DRAM0_SRAM_2_R_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_2_R_S  4
/* DPORT_PMS_PRO_DRAM0_SRAM_1_W : R/W ;bitpos:[3] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_1_W  (BIT(3))
#define DPORT_PMS_PRO_DRAM0_SRAM_1_W_M  (BIT(3))
#define DPORT_PMS_PRO_DRAM0_SRAM_1_W_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_1_W_S  3
/* DPORT_PMS_PRO_DRAM0_SRAM_1_R : R/W ;bitpos:[2] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_1_R  (BIT(2))
#define DPORT_PMS_PRO_DRAM0_SRAM_1_R_M  (BIT(2))
#define DPORT_PMS_PRO_DRAM0_SRAM_1_R_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_1_R_S  2
/* DPORT_PMS_PRO_DRAM0_SRAM_0_W : R/W ;bitpos:[1] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_0_W  (BIT(1))
#define DPORT_PMS_PRO_DRAM0_SRAM_0_W_M  (BIT(1))
#define DPORT_PMS_PRO_DRAM0_SRAM_0_W_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_0_W_S  1
/* DPORT_PMS_PRO_DRAM0_SRAM_0_R : R/W ;bitpos:[0] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_SRAM_0_R  (BIT(0))
#define DPORT_PMS_PRO_DRAM0_SRAM_0_R_M  (BIT(0))
#define DPORT_PMS_PRO_DRAM0_SRAM_0_R_V  0x1
#define DPORT_PMS_PRO_DRAM0_SRAM_0_R_S  0

#define DPORT_PMS_PRO_DRAM0_2_REG          (DR_REG_SENSITIVE_BASE + 0x030)
/* DPORT_PMS_PRO_DRAM0_RTCFAST_H_W : R/W ;bitpos:[14] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_RTCFAST_H_W  (BIT(14))
#define DPORT_PMS_PRO_DRAM0_RTCFAST_H_W_M  (BIT(14))
#define DPORT_PMS_PRO_DRAM0_RTCFAST_H_W_V  0x1
#define DPORT_PMS_PRO_DRAM0_RTCFAST_H_W_S  14
/* DPORT_PMS_PRO_DRAM0_RTCFAST_H_R : R/W ;bitpos:[13] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_RTCFAST_H_R  (BIT(13))
#define DPORT_PMS_PRO_DRAM0_RTCFAST_H_R_M  (BIT(13))
#define DPORT_PMS_PRO_DRAM0_RTCFAST_H_R_V  0x1
#define DPORT_PMS_PRO_DRAM0_RTCFAST_H_R_S  13
/* DPORT_PMS_PRO_DRAM0_RTCFAST_L_W : R/W ;bitpos:[12] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_RTCFAST_L_W  (BIT(12))
#define DPORT_PMS_PRO_DRAM0_RTCFAST_L_W_M  (BIT(12))
#define DPORT_PMS_PRO_DRAM0_RTCFAST_L_W_V  0x1
#define DPORT_PMS_PRO_DRAM0_RTCFAST_L_W_S  12
/* DPORT_PMS_PRO_DRAM0_RTCFAST_L_R : R/W ;bitpos:[11] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_RTCFAST_L_R  (BIT(11))
#define DPORT_PMS_PRO_DRAM0_RTCFAST_L_R_M  (BIT(11))
#define DPORT_PMS_PRO_DRAM0_RTCFAST_L_R_V  0x1
#define DPORT_PMS_PRO_DRAM0_RTCFAST_L_R_S  11
/* DPORT_PMS_PRO_DRAM0_RTCFAST_SPLTADDR : R/W ;bitpos:[10:0] ;default: 11'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_RTCFAST_SPLTADDR  0x000007FF
#define DPORT_PMS_PRO_DRAM0_RTCFAST_SPLTADDR_M  ((DPORT_PMS_PRO_DRAM0_RTCFAST_SPLTADDR_V)<<(DPORT_PMS_PRO_DRAM0_RTCFAST_SPLTADDR_S))
#define DPORT_PMS_PRO_DRAM0_RTCFAST_SPLTADDR_V  0x7FF
#define DPORT_PMS_PRO_DRAM0_RTCFAST_SPLTADDR_S  0

#define DPORT_PMS_PRO_DRAM0_3_REG          (DR_REG_SENSITIVE_BASE + 0x034)
/* DPORT_PMS_PRO_DRAM0_ILG_INTR : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_ILG_INTR  (BIT(2))
#define DPORT_PMS_PRO_DRAM0_ILG_INTR_M  (BIT(2))
#define DPORT_PMS_PRO_DRAM0_ILG_INTR_V  0x1
#define DPORT_PMS_PRO_DRAM0_ILG_INTR_S  2
/* DPORT_PMS_PRO_DRAM0_ILG_EN : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_ILG_EN  (BIT(1))
#define DPORT_PMS_PRO_DRAM0_ILG_EN_M  (BIT(1))
#define DPORT_PMS_PRO_DRAM0_ILG_EN_V  0x1
#define DPORT_PMS_PRO_DRAM0_ILG_EN_S  1
/* DPORT_PMS_PRO_DRAM0_ILG_CLR : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_ILG_CLR  (BIT(0))
#define DPORT_PMS_PRO_DRAM0_ILG_CLR_M  (BIT(0))
#define DPORT_PMS_PRO_DRAM0_ILG_CLR_V  0x1
#define DPORT_PMS_PRO_DRAM0_ILG_CLR_S  0

#define DPORT_PMS_PRO_DRAM0_4_REG          (DR_REG_SENSITIVE_BASE + 0x038)
/* DPORT_PMS_PRO_DRAM0_ILG_ST : RO ;bitpos:[25:0] ;default: 26'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DRAM0_ILG_ST  0x03FFFFFF
#define DPORT_PMS_PRO_DRAM0_ILG_ST_M  ((DPORT_PMS_PRO_DRAM0_ILG_ST_V)<<(DPORT_PMS_PRO_DRAM0_ILG_ST_S))
#define DPORT_PMS_PRO_DRAM0_ILG_ST_V  0x3FFFFFF
#define DPORT_PMS_PRO_DRAM0_ILG_ST_S  0

#define DPORT_PMS_PRO_DPORT_0_REG          (DR_REG_SENSITIVE_BASE + 0x03C)
/* DPORT_PMS_PRO_DPORT_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_LOCK  (BIT(0))
#define DPORT_PMS_PRO_DPORT_LOCK_M  (BIT(0))
#define DPORT_PMS_PRO_DPORT_LOCK_V  0x1
#define DPORT_PMS_PRO_DPORT_LOCK_S  0

#define DPORT_PMS_PRO_DPORT_1_REG          (DR_REG_SENSITIVE_BASE + 0x040)
/* DPORT_PMS_PRO_DPORT_RESERVE_FIFO_VALID : R/W ;bitpos:[19:16] ;default: 4'b0000 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_VALID  0x0000000F
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_VALID_M  ((DPORT_PMS_PRO_DPORT_RESERVE_FIFO_VALID_V)<<(DPORT_PMS_PRO_DPORT_RESERVE_FIFO_VALID_S))
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_VALID_V  0xF
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_VALID_S  16
/* DPORT_PMS_PRO_DPORT_RTCSLOW_H_W : R/W ;bitpos:[15] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_RTCSLOW_H_W  (BIT(15))
#define DPORT_PMS_PRO_DPORT_RTCSLOW_H_W_M  (BIT(15))
#define DPORT_PMS_PRO_DPORT_RTCSLOW_H_W_V  0x1
#define DPORT_PMS_PRO_DPORT_RTCSLOW_H_W_S  15
/* DPORT_PMS_PRO_DPORT_RTCSLOW_H_R : R/W ;bitpos:[14] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_RTCSLOW_H_R  (BIT(14))
#define DPORT_PMS_PRO_DPORT_RTCSLOW_H_R_M  (BIT(14))
#define DPORT_PMS_PRO_DPORT_RTCSLOW_H_R_V  0x1
#define DPORT_PMS_PRO_DPORT_RTCSLOW_H_R_S  14
/* DPORT_PMS_PRO_DPORT_RTCSLOW_L_W : R/W ;bitpos:[13] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_RTCSLOW_L_W  (BIT(13))
#define DPORT_PMS_PRO_DPORT_RTCSLOW_L_W_M  (BIT(13))
#define DPORT_PMS_PRO_DPORT_RTCSLOW_L_W_V  0x1
#define DPORT_PMS_PRO_DPORT_RTCSLOW_L_W_S  13
/* DPORT_PMS_PRO_DPORT_RTCSLOW_L_R : R/W ;bitpos:[12] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_RTCSLOW_L_R  (BIT(12))
#define DPORT_PMS_PRO_DPORT_RTCSLOW_L_R_M  (BIT(12))
#define DPORT_PMS_PRO_DPORT_RTCSLOW_L_R_V  0x1
#define DPORT_PMS_PRO_DPORT_RTCSLOW_L_R_S  12
/* DPORT_PMS_PRO_DPORT_RTCSLOW_SPLTADDR : R/W ;bitpos:[11:1] ;default: 11'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_RTCSLOW_SPLTADDR  0x000007FF
#define DPORT_PMS_PRO_DPORT_RTCSLOW_SPLTADDR_M  ((DPORT_PMS_PRO_DPORT_RTCSLOW_SPLTADDR_V)<<(DPORT_PMS_PRO_DPORT_RTCSLOW_SPLTADDR_S))
#define DPORT_PMS_PRO_DPORT_RTCSLOW_SPLTADDR_V  0x7FF
#define DPORT_PMS_PRO_DPORT_RTCSLOW_SPLTADDR_S  1
/* DPORT_PMS_PRO_DPORT_APB_PERIPHERAL_FORBID : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_APB_PERIPHERAL_FORBID  (BIT(0))
#define DPORT_PMS_PRO_DPORT_APB_PERIPHERAL_FORBID_M  (BIT(0))
#define DPORT_PMS_PRO_DPORT_APB_PERIPHERAL_FORBID_V  0x1
#define DPORT_PMS_PRO_DPORT_APB_PERIPHERAL_FORBID_S  0

#define DPORT_PMS_PRO_DPORT_2_REG          (DR_REG_SENSITIVE_BASE + 0x044)
/* DPORT_PMS_PRO_DPORT_RESERVE_FIFO_0 : R/W ;bitpos:[17:0] ;default: 18'h0 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_0  0x0003FFFF
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_0_M  ((DPORT_PMS_PRO_DPORT_RESERVE_FIFO_0_V)<<(DPORT_PMS_PRO_DPORT_RESERVE_FIFO_0_S))
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_0_V  0x3FFFF
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_0_S  0

#define DPORT_PMS_PRO_DPORT_3_REG          (DR_REG_SENSITIVE_BASE + 0x048)
/* DPORT_PMS_PRO_DPORT_RESERVE_FIFO_1 : R/W ;bitpos:[17:0] ;default: 18'h0 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_1  0x0003FFFF
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_1_M  ((DPORT_PMS_PRO_DPORT_RESERVE_FIFO_1_V)<<(DPORT_PMS_PRO_DPORT_RESERVE_FIFO_1_S))
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_1_V  0x3FFFF
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_1_S  0

#define DPORT_PMS_PRO_DPORT_4_REG          (DR_REG_SENSITIVE_BASE + 0x04C)
/* DPORT_PMS_PRO_DPORT_RESERVE_FIFO_2 : R/W ;bitpos:[17:0] ;default: 18'h0 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_2  0x0003FFFF
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_2_M  ((DPORT_PMS_PRO_DPORT_RESERVE_FIFO_2_V)<<(DPORT_PMS_PRO_DPORT_RESERVE_FIFO_2_S))
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_2_V  0x3FFFF
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_2_S  0

#define DPORT_PMS_PRO_DPORT_5_REG          (DR_REG_SENSITIVE_BASE + 0x050)
/* DPORT_PMS_PRO_DPORT_RESERVE_FIFO_3 : R/W ;bitpos:[17:0] ;default: 18'h0 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_3  0x0003FFFF
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_3_M  ((DPORT_PMS_PRO_DPORT_RESERVE_FIFO_3_V)<<(DPORT_PMS_PRO_DPORT_RESERVE_FIFO_3_S))
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_3_V  0x3FFFF
#define DPORT_PMS_PRO_DPORT_RESERVE_FIFO_3_S  0

#define DPORT_PMS_PRO_DPORT_6_REG          (DR_REG_SENSITIVE_BASE + 0x054)
/* DPORT_PMS_PRO_DPORT_ILG_INTR : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_ILG_INTR  (BIT(2))
#define DPORT_PMS_PRO_DPORT_ILG_INTR_M  (BIT(2))
#define DPORT_PMS_PRO_DPORT_ILG_INTR_V  0x1
#define DPORT_PMS_PRO_DPORT_ILG_INTR_S  2
/* DPORT_PMS_PRO_DPORT_ILG_EN : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_ILG_EN  (BIT(1))
#define DPORT_PMS_PRO_DPORT_ILG_EN_M  (BIT(1))
#define DPORT_PMS_PRO_DPORT_ILG_EN_V  0x1
#define DPORT_PMS_PRO_DPORT_ILG_EN_S  1
/* DPORT_PMS_PRO_DPORT_ILG_CLR : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_ILG_CLR  (BIT(0))
#define DPORT_PMS_PRO_DPORT_ILG_CLR_M  (BIT(0))
#define DPORT_PMS_PRO_DPORT_ILG_CLR_V  0x1
#define DPORT_PMS_PRO_DPORT_ILG_CLR_S  0

#define DPORT_PMS_PRO_DPORT_7_REG          (DR_REG_SENSITIVE_BASE + 0x058)
/* DPORT_PMS_PRO_DPORT_ILG_ST : RO ;bitpos:[25:0] ;default: 26'b0 ; */
/*description: */
#define DPORT_PMS_PRO_DPORT_ILG_ST  0x03FFFFFF
#define DPORT_PMS_PRO_DPORT_ILG_ST_M  ((DPORT_PMS_PRO_DPORT_ILG_ST_V)<<(DPORT_PMS_PRO_DPORT_ILG_ST_S))
#define DPORT_PMS_PRO_DPORT_ILG_ST_V  0x3FFFFFF
#define DPORT_PMS_PRO_DPORT_ILG_ST_S  0

#define DPORT_PMS_PRO_AHB_0_REG          (DR_REG_SENSITIVE_BASE + 0x05C)
/* DPORT_PMS_PRO_AHB_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_LOCK  (BIT(0))
#define DPORT_PMS_PRO_AHB_LOCK_M  (BIT(0))
#define DPORT_PMS_PRO_AHB_LOCK_V  0x1
#define DPORT_PMS_PRO_AHB_LOCK_S  0

#define DPORT_PMS_PRO_AHB_1_REG          (DR_REG_SENSITIVE_BASE + 0x060)
/* DPORT_PMS_PRO_AHB_RTCSLOW_0_H_W : R/W ;bitpos:[16] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_W  (BIT(16))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_W_M  (BIT(16))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_W_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_W_S  16
/* DPORT_PMS_PRO_AHB_RTCSLOW_0_H_R : R/W ;bitpos:[15] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_R  (BIT(15))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_R_M  (BIT(15))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_R_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_R_S  15
/* DPORT_PMS_PRO_AHB_RTCSLOW_0_H_F : R/W ;bitpos:[14] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_F  (BIT(14))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_F_M  (BIT(14))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_F_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_H_F_S  14
/* DPORT_PMS_PRO_AHB_RTCSLOW_0_L_W : R/W ;bitpos:[13] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_W  (BIT(13))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_W_M  (BIT(13))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_W_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_W_S  13
/* DPORT_PMS_PRO_AHB_RTCSLOW_0_L_R : R/W ;bitpos:[12] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_R  (BIT(12))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_R_M  (BIT(12))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_R_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_R_S  12
/* DPORT_PMS_PRO_AHB_RTCSLOW_0_L_F : R/W ;bitpos:[11] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_F  (BIT(11))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_F_M  (BIT(11))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_F_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_L_F_S  11
/* DPORT_PMS_PRO_AHB_RTCSLOW_0_SPLTADDR : R/W ;bitpos:[10:0] ;default: 11'b0 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_SPLTADDR  0x000007FF
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_SPLTADDR_M  ((DPORT_PMS_PRO_AHB_RTCSLOW_0_SPLTADDR_V)<<(DPORT_PMS_PRO_AHB_RTCSLOW_0_SPLTADDR_S))
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_SPLTADDR_V  0x7FF
#define DPORT_PMS_PRO_AHB_RTCSLOW_0_SPLTADDR_S  0

#define DPORT_PMS_PRO_AHB_2_REG          (DR_REG_SENSITIVE_BASE + 0x064)
/* DPORT_PMS_PRO_AHB_RTCSLOW_1_H_W : R/W ;bitpos:[16] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_W  (BIT(16))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_W_M  (BIT(16))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_W_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_W_S  16
/* DPORT_PMS_PRO_AHB_RTCSLOW_1_H_R : R/W ;bitpos:[15] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_R  (BIT(15))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_R_M  (BIT(15))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_R_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_R_S  15
/* DPORT_PMS_PRO_AHB_RTCSLOW_1_H_F : R/W ;bitpos:[14] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_F  (BIT(14))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_F_M  (BIT(14))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_F_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_H_F_S  14
/* DPORT_PMS_PRO_AHB_RTCSLOW_1_L_W : R/W ;bitpos:[13] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_W  (BIT(13))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_W_M  (BIT(13))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_W_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_W_S  13
/* DPORT_PMS_PRO_AHB_RTCSLOW_1_L_R : R/W ;bitpos:[12] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_R  (BIT(12))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_R_M  (BIT(12))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_R_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_R_S  12
/* DPORT_PMS_PRO_AHB_RTCSLOW_1_L_F : R/W ;bitpos:[11] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_F  (BIT(11))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_F_M  (BIT(11))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_F_V  0x1
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_L_F_S  11
/* DPORT_PMS_PRO_AHB_RTCSLOW_1_SPLTADDR : R/W ;bitpos:[10:0] ;default: 11'b0 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_SPLTADDR  0x000007FF
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_SPLTADDR_M  ((DPORT_PMS_PRO_AHB_RTCSLOW_1_SPLTADDR_V)<<(DPORT_PMS_PRO_AHB_RTCSLOW_1_SPLTADDR_S))
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_SPLTADDR_V  0x7FF
#define DPORT_PMS_PRO_AHB_RTCSLOW_1_SPLTADDR_S  0

#define DPORT_PMS_PRO_AHB_3_REG          (DR_REG_SENSITIVE_BASE + 0x068)
/* DPORT_PMS_PRO_AHB_ILG_INTR : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_ILG_INTR  (BIT(2))
#define DPORT_PMS_PRO_AHB_ILG_INTR_M  (BIT(2))
#define DPORT_PMS_PRO_AHB_ILG_INTR_V  0x1
#define DPORT_PMS_PRO_AHB_ILG_INTR_S  2
/* DPORT_PMS_PRO_AHB_ILG_EN : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_ILG_EN  (BIT(1))
#define DPORT_PMS_PRO_AHB_ILG_EN_M  (BIT(1))
#define DPORT_PMS_PRO_AHB_ILG_EN_V  0x1
#define DPORT_PMS_PRO_AHB_ILG_EN_S  1
/* DPORT_PMS_PRO_AHB_ILG_CLR : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_ILG_CLR  (BIT(0))
#define DPORT_PMS_PRO_AHB_ILG_CLR_M  (BIT(0))
#define DPORT_PMS_PRO_AHB_ILG_CLR_V  0x1
#define DPORT_PMS_PRO_AHB_ILG_CLR_S  0

#define DPORT_PMS_PRO_AHB_4_REG          (DR_REG_SENSITIVE_BASE + 0x06C)
/* DPORT_PMS_PRO_AHB_ILG_ST : RO ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define DPORT_PMS_PRO_AHB_ILG_ST  0xFFFFFFFF
#define DPORT_PMS_PRO_AHB_ILG_ST_M  ((DPORT_PMS_PRO_AHB_ILG_ST_V)<<(DPORT_PMS_PRO_AHB_ILG_ST_S))
#define DPORT_PMS_PRO_AHB_ILG_ST_V  0xFFFFFFFF
#define DPORT_PMS_PRO_AHB_ILG_ST_S  0

#define DPORT_PMS_PRO_TRACE_0_REG          (DR_REG_SENSITIVE_BASE + 0x070)
/* DPORT_PMS_PRO_TRACE_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_TRACE_LOCK  (BIT(0))
#define DPORT_PMS_PRO_TRACE_LOCK_M  (BIT(0))
#define DPORT_PMS_PRO_TRACE_LOCK_V  0x1
#define DPORT_PMS_PRO_TRACE_LOCK_S  0

#define DPORT_PMS_PRO_TRACE_1_REG          (DR_REG_SENSITIVE_BASE + 0x074)
/* DPORT_PMS_PRO_TRACE_DISABLE : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_TRACE_DISABLE  (BIT(0))
#define DPORT_PMS_PRO_TRACE_DISABLE_M  (BIT(0))
#define DPORT_PMS_PRO_TRACE_DISABLE_V  0x1
#define DPORT_PMS_PRO_TRACE_DISABLE_S  0

#define DPORT_PMS_PRO_CACHE_0_REG          (DR_REG_SENSITIVE_BASE + 0x078)
/* DPORT_PMS_PRO_CACHE_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_CACHE_LOCK  (BIT(0))
#define DPORT_PMS_PRO_CACHE_LOCK_M  (BIT(0))
#define DPORT_PMS_PRO_CACHE_LOCK_V  0x1
#define DPORT_PMS_PRO_CACHE_LOCK_S  0

#define DPORT_PMS_PRO_CACHE_1_REG          (DR_REG_SENSITIVE_BASE + 0x07C)
/* DPORT_PMS_PRO_CACHE_CONNECT : R/W ;bitpos:[15:0] ;default: 16'b0 ; */
/*description: */
#define DPORT_PMS_PRO_CACHE_CONNECT  0x0000FFFF
#define DPORT_PMS_PRO_CACHE_CONNECT_M  ((DPORT_PMS_PRO_CACHE_CONNECT_V)<<(DPORT_PMS_PRO_CACHE_CONNECT_S))
#define DPORT_PMS_PRO_CACHE_CONNECT_V  0xFFFF
#define DPORT_PMS_PRO_CACHE_CONNECT_S  0

#define DPORT_PMS_PRO_CACHE_2_REG          (DR_REG_SENSITIVE_BASE + 0x080)
/* DPORT_PMS_PRO_CACHE_ILG_INTR : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_CACHE_ILG_INTR  (BIT(2))
#define DPORT_PMS_PRO_CACHE_ILG_INTR_M  (BIT(2))
#define DPORT_PMS_PRO_CACHE_ILG_INTR_V  0x1
#define DPORT_PMS_PRO_CACHE_ILG_INTR_S  2
/* DPORT_PMS_PRO_CACHE_ILG_EN : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_CACHE_ILG_EN  (BIT(1))
#define DPORT_PMS_PRO_CACHE_ILG_EN_M  (BIT(1))
#define DPORT_PMS_PRO_CACHE_ILG_EN_V  0x1
#define DPORT_PMS_PRO_CACHE_ILG_EN_S  1
/* DPORT_PMS_PRO_CACHE_ILG_CLR : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_PRO_CACHE_ILG_CLR  (BIT(0))
#define DPORT_PMS_PRO_CACHE_ILG_CLR_M  (BIT(0))
#define DPORT_PMS_PRO_CACHE_ILG_CLR_V  0x1
#define DPORT_PMS_PRO_CACHE_ILG_CLR_S  0

#define DPORT_PMS_PRO_CACHE_3_REG          (DR_REG_SENSITIVE_BASE + 0x084)
/* DPORT_PMS_PRO_CACHE_ILG_ST_I : RO ;bitpos:[16:0] ;default: 17'b0 ; */
/*description: */
#define DPORT_PMS_PRO_CACHE_ILG_ST_I  0x0001FFFF
#define DPORT_PMS_PRO_CACHE_ILG_ST_I_M  ((DPORT_PMS_PRO_CACHE_ILG_ST_I_V)<<(DPORT_PMS_PRO_CACHE_ILG_ST_I_S))
#define DPORT_PMS_PRO_CACHE_ILG_ST_I_V  0x1FFFF
#define DPORT_PMS_PRO_CACHE_ILG_ST_I_S  0

#define DPORT_PMS_PRO_CACHE_4_REG          (DR_REG_SENSITIVE_BASE + 0x088)
/* DPORT_PMS_PRO_CACHE_ILG_ST_D : RO ;bitpos:[16:0] ;default: 17'b0 ; */
/*description: */
#define DPORT_PMS_PRO_CACHE_ILG_ST_D  0x0001FFFF
#define DPORT_PMS_PRO_CACHE_ILG_ST_D_M  ((DPORT_PMS_PRO_CACHE_ILG_ST_D_V)<<(DPORT_PMS_PRO_CACHE_ILG_ST_D_S))
#define DPORT_PMS_PRO_CACHE_ILG_ST_D_V  0x1FFFF
#define DPORT_PMS_PRO_CACHE_ILG_ST_D_S  0

#define DPORT_PMS_DMA_APB_I_0_REG          (DR_REG_SENSITIVE_BASE + 0x08C)
/* DPORT_PMS_DMA_APB_I_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_LOCK  (BIT(0))
#define DPORT_PMS_DMA_APB_I_LOCK_M  (BIT(0))
#define DPORT_PMS_DMA_APB_I_LOCK_V  0x1
#define DPORT_PMS_DMA_APB_I_LOCK_S  0

#define DPORT_PMS_DMA_APB_I_1_REG          (DR_REG_SENSITIVE_BASE + 0x090)
/* DPORT_PMS_DMA_APB_I_SRAM_4_H_W : R/W ;bitpos:[28] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_4_H_W  (BIT(28))
#define DPORT_PMS_DMA_APB_I_SRAM_4_H_W_M  (BIT(28))
#define DPORT_PMS_DMA_APB_I_SRAM_4_H_W_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_4_H_W_S  28
/* DPORT_PMS_DMA_APB_I_SRAM_4_H_R : R/W ;bitpos:[27] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_4_H_R  (BIT(27))
#define DPORT_PMS_DMA_APB_I_SRAM_4_H_R_M  (BIT(27))
#define DPORT_PMS_DMA_APB_I_SRAM_4_H_R_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_4_H_R_S  27
/* DPORT_PMS_DMA_APB_I_SRAM_4_L_W : R/W ;bitpos:[26] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_4_L_W  (BIT(26))
#define DPORT_PMS_DMA_APB_I_SRAM_4_L_W_M  (BIT(26))
#define DPORT_PMS_DMA_APB_I_SRAM_4_L_W_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_4_L_W_S  26
/* DPORT_PMS_DMA_APB_I_SRAM_4_L_R : R/W ;bitpos:[25] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_4_L_R  (BIT(25))
#define DPORT_PMS_DMA_APB_I_SRAM_4_L_R_M  (BIT(25))
#define DPORT_PMS_DMA_APB_I_SRAM_4_L_R_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_4_L_R_S  25
/* DPORT_PMS_DMA_APB_I_SRAM_4_SPLTADDR : R/W ;bitpos:[24:8] ;default: 17'b0 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_4_SPLTADDR  0x0001FFFF
#define DPORT_PMS_DMA_APB_I_SRAM_4_SPLTADDR_M  ((DPORT_PMS_DMA_APB_I_SRAM_4_SPLTADDR_V)<<(DPORT_PMS_DMA_APB_I_SRAM_4_SPLTADDR_S))
#define DPORT_PMS_DMA_APB_I_SRAM_4_SPLTADDR_V  0x1FFFF
#define DPORT_PMS_DMA_APB_I_SRAM_4_SPLTADDR_S  8
/* DPORT_PMS_DMA_APB_I_SRAM_3_W : R/W ;bitpos:[7] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_3_W  (BIT(7))
#define DPORT_PMS_DMA_APB_I_SRAM_3_W_M  (BIT(7))
#define DPORT_PMS_DMA_APB_I_SRAM_3_W_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_3_W_S  7
/* DPORT_PMS_DMA_APB_I_SRAM_3_R : R/W ;bitpos:[6] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_3_R  (BIT(6))
#define DPORT_PMS_DMA_APB_I_SRAM_3_R_M  (BIT(6))
#define DPORT_PMS_DMA_APB_I_SRAM_3_R_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_3_R_S  6
/* DPORT_PMS_DMA_APB_I_SRAM_2_W : R/W ;bitpos:[5] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_2_W  (BIT(5))
#define DPORT_PMS_DMA_APB_I_SRAM_2_W_M  (BIT(5))
#define DPORT_PMS_DMA_APB_I_SRAM_2_W_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_2_W_S  5
/* DPORT_PMS_DMA_APB_I_SRAM_2_R : R/W ;bitpos:[4] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_2_R  (BIT(4))
#define DPORT_PMS_DMA_APB_I_SRAM_2_R_M  (BIT(4))
#define DPORT_PMS_DMA_APB_I_SRAM_2_R_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_2_R_S  4
/* DPORT_PMS_DMA_APB_I_SRAM_1_W : R/W ;bitpos:[3] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_1_W  (BIT(3))
#define DPORT_PMS_DMA_APB_I_SRAM_1_W_M  (BIT(3))
#define DPORT_PMS_DMA_APB_I_SRAM_1_W_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_1_W_S  3
/* DPORT_PMS_DMA_APB_I_SRAM_1_R : R/W ;bitpos:[2] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_1_R  (BIT(2))
#define DPORT_PMS_DMA_APB_I_SRAM_1_R_M  (BIT(2))
#define DPORT_PMS_DMA_APB_I_SRAM_1_R_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_1_R_S  2
/* DPORT_PMS_DMA_APB_I_SRAM_0_W : R/W ;bitpos:[1] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_0_W  (BIT(1))
#define DPORT_PMS_DMA_APB_I_SRAM_0_W_M  (BIT(1))
#define DPORT_PMS_DMA_APB_I_SRAM_0_W_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_0_W_S  1
/* DPORT_PMS_DMA_APB_I_SRAM_0_R : R/W ;bitpos:[0] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_SRAM_0_R  (BIT(0))
#define DPORT_PMS_DMA_APB_I_SRAM_0_R_M  (BIT(0))
#define DPORT_PMS_DMA_APB_I_SRAM_0_R_V  0x1
#define DPORT_PMS_DMA_APB_I_SRAM_0_R_S  0

#define DPORT_PMS_DMA_APB_I_2_REG          (DR_REG_SENSITIVE_BASE + 0x094)
/* DPORT_PMS_DMA_APB_I_ILG_INTR : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_ILG_INTR  (BIT(2))
#define DPORT_PMS_DMA_APB_I_ILG_INTR_M  (BIT(2))
#define DPORT_PMS_DMA_APB_I_ILG_INTR_V  0x1
#define DPORT_PMS_DMA_APB_I_ILG_INTR_S  2
/* DPORT_PMS_DMA_APB_I_ILG_EN : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_ILG_EN  (BIT(1))
#define DPORT_PMS_DMA_APB_I_ILG_EN_M  (BIT(1))
#define DPORT_PMS_DMA_APB_I_ILG_EN_V  0x1
#define DPORT_PMS_DMA_APB_I_ILG_EN_S  1
/* DPORT_PMS_DMA_APB_I_ILG_CLR : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_ILG_CLR  (BIT(0))
#define DPORT_PMS_DMA_APB_I_ILG_CLR_M  (BIT(0))
#define DPORT_PMS_DMA_APB_I_ILG_CLR_V  0x1
#define DPORT_PMS_DMA_APB_I_ILG_CLR_S  0

#define DPORT_PMS_DMA_APB_I_3_REG          (DR_REG_SENSITIVE_BASE + 0x098)
/* DPORT_PMS_DMA_APB_I_ILG_ST : RO ;bitpos:[22:0] ;default: 23'b0 ; */
/*description: */
#define DPORT_PMS_DMA_APB_I_ILG_ST  0x007FFFFF
#define DPORT_PMS_DMA_APB_I_ILG_ST_M  ((DPORT_PMS_DMA_APB_I_ILG_ST_V)<<(DPORT_PMS_DMA_APB_I_ILG_ST_S))
#define DPORT_PMS_DMA_APB_I_ILG_ST_V  0x7FFFFF
#define DPORT_PMS_DMA_APB_I_ILG_ST_S  0

#define DPORT_PMS_DMA_RX_I_0_REG          (DR_REG_SENSITIVE_BASE + 0x09C)
/* DPORT_PMS_DMA_RX_I_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_LOCK  (BIT(0))
#define DPORT_PMS_DMA_RX_I_LOCK_M  (BIT(0))
#define DPORT_PMS_DMA_RX_I_LOCK_V  0x1
#define DPORT_PMS_DMA_RX_I_LOCK_S  0

#define DPORT_PMS_DMA_RX_I_1_REG          (DR_REG_SENSITIVE_BASE + 0x0A0)
/* DPORT_PMS_DMA_RX_I_SRAM_4_H_W : R/W ;bitpos:[28] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_4_H_W  (BIT(28))
#define DPORT_PMS_DMA_RX_I_SRAM_4_H_W_M  (BIT(28))
#define DPORT_PMS_DMA_RX_I_SRAM_4_H_W_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_4_H_W_S  28
/* DPORT_PMS_DMA_RX_I_SRAM_4_H_R : R/W ;bitpos:[27] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_4_H_R  (BIT(27))
#define DPORT_PMS_DMA_RX_I_SRAM_4_H_R_M  (BIT(27))
#define DPORT_PMS_DMA_RX_I_SRAM_4_H_R_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_4_H_R_S  27
/* DPORT_PMS_DMA_RX_I_SRAM_4_L_W : R/W ;bitpos:[26] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_4_L_W  (BIT(26))
#define DPORT_PMS_DMA_RX_I_SRAM_4_L_W_M  (BIT(26))
#define DPORT_PMS_DMA_RX_I_SRAM_4_L_W_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_4_L_W_S  26
/* DPORT_PMS_DMA_RX_I_SRAM_4_L_R : R/W ;bitpos:[25] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_4_L_R  (BIT(25))
#define DPORT_PMS_DMA_RX_I_SRAM_4_L_R_M  (BIT(25))
#define DPORT_PMS_DMA_RX_I_SRAM_4_L_R_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_4_L_R_S  25
/* DPORT_PMS_DMA_RX_I_SRAM_4_SPLTADDR : R/W ;bitpos:[24:8] ;default: 17'b0 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_4_SPLTADDR  0x0001FFFF
#define DPORT_PMS_DMA_RX_I_SRAM_4_SPLTADDR_M  ((DPORT_PMS_DMA_RX_I_SRAM_4_SPLTADDR_V)<<(DPORT_PMS_DMA_RX_I_SRAM_4_SPLTADDR_S))
#define DPORT_PMS_DMA_RX_I_SRAM_4_SPLTADDR_V  0x1FFFF
#define DPORT_PMS_DMA_RX_I_SRAM_4_SPLTADDR_S  8
/* DPORT_PMS_DMA_RX_I_SRAM_3_W : R/W ;bitpos:[7] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_3_W  (BIT(7))
#define DPORT_PMS_DMA_RX_I_SRAM_3_W_M  (BIT(7))
#define DPORT_PMS_DMA_RX_I_SRAM_3_W_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_3_W_S  7
/* DPORT_PMS_DMA_RX_I_SRAM_3_R : R/W ;bitpos:[6] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_3_R  (BIT(6))
#define DPORT_PMS_DMA_RX_I_SRAM_3_R_M  (BIT(6))
#define DPORT_PMS_DMA_RX_I_SRAM_3_R_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_3_R_S  6
/* DPORT_PMS_DMA_RX_I_SRAM_2_W : R/W ;bitpos:[5] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_2_W  (BIT(5))
#define DPORT_PMS_DMA_RX_I_SRAM_2_W_M  (BIT(5))
#define DPORT_PMS_DMA_RX_I_SRAM_2_W_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_2_W_S  5
/* DPORT_PMS_DMA_RX_I_SRAM_2_R : R/W ;bitpos:[4] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_2_R  (BIT(4))
#define DPORT_PMS_DMA_RX_I_SRAM_2_R_M  (BIT(4))
#define DPORT_PMS_DMA_RX_I_SRAM_2_R_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_2_R_S  4
/* DPORT_PMS_DMA_RX_I_SRAM_1_W : R/W ;bitpos:[3] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_1_W  (BIT(3))
#define DPORT_PMS_DMA_RX_I_SRAM_1_W_M  (BIT(3))
#define DPORT_PMS_DMA_RX_I_SRAM_1_W_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_1_W_S  3
/* DPORT_PMS_DMA_RX_I_SRAM_1_R : R/W ;bitpos:[2] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_1_R  (BIT(2))
#define DPORT_PMS_DMA_RX_I_SRAM_1_R_M  (BIT(2))
#define DPORT_PMS_DMA_RX_I_SRAM_1_R_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_1_R_S  2
/* DPORT_PMS_DMA_RX_I_SRAM_0_W : R/W ;bitpos:[1] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_0_W  (BIT(1))
#define DPORT_PMS_DMA_RX_I_SRAM_0_W_M  (BIT(1))
#define DPORT_PMS_DMA_RX_I_SRAM_0_W_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_0_W_S  1
/* DPORT_PMS_DMA_RX_I_SRAM_0_R : R/W ;bitpos:[0] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_SRAM_0_R  (BIT(0))
#define DPORT_PMS_DMA_RX_I_SRAM_0_R_M  (BIT(0))
#define DPORT_PMS_DMA_RX_I_SRAM_0_R_V  0x1
#define DPORT_PMS_DMA_RX_I_SRAM_0_R_S  0

#define DPORT_PMS_DMA_RX_I_2_REG          (DR_REG_SENSITIVE_BASE + 0x0A4)
/* DPORT_PMS_DMA_RX_I_ILG_INTR : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_ILG_INTR  (BIT(2))
#define DPORT_PMS_DMA_RX_I_ILG_INTR_M  (BIT(2))
#define DPORT_PMS_DMA_RX_I_ILG_INTR_V  0x1
#define DPORT_PMS_DMA_RX_I_ILG_INTR_S  2
/* DPORT_PMS_DMA_RX_I_ILG_EN : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_ILG_EN  (BIT(1))
#define DPORT_PMS_DMA_RX_I_ILG_EN_M  (BIT(1))
#define DPORT_PMS_DMA_RX_I_ILG_EN_V  0x1
#define DPORT_PMS_DMA_RX_I_ILG_EN_S  1
/* DPORT_PMS_DMA_RX_I_ILG_CLR : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_ILG_CLR  (BIT(0))
#define DPORT_PMS_DMA_RX_I_ILG_CLR_M  (BIT(0))
#define DPORT_PMS_DMA_RX_I_ILG_CLR_V  0x1
#define DPORT_PMS_DMA_RX_I_ILG_CLR_S  0

#define DPORT_PMS_DMA_RX_I_3_REG          (DR_REG_SENSITIVE_BASE + 0x0A8)
/* DPORT_PMS_DMA_RX_I_ILG_ST : RO ;bitpos:[22:0] ;default: 23'b0 ; */
/*description: */
#define DPORT_PMS_DMA_RX_I_ILG_ST  0x007FFFFF
#define DPORT_PMS_DMA_RX_I_ILG_ST_M  ((DPORT_PMS_DMA_RX_I_ILG_ST_V)<<(DPORT_PMS_DMA_RX_I_ILG_ST_S))
#define DPORT_PMS_DMA_RX_I_ILG_ST_V  0x7FFFFF
#define DPORT_PMS_DMA_RX_I_ILG_ST_S  0

#define DPORT_PMS_DMA_TX_I_0_REG          (DR_REG_SENSITIVE_BASE + 0x0AC)
/* DPORT_PMS_DMA_TX_I_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_LOCK  (BIT(0))
#define DPORT_PMS_DMA_TX_I_LOCK_M  (BIT(0))
#define DPORT_PMS_DMA_TX_I_LOCK_V  0x1
#define DPORT_PMS_DMA_TX_I_LOCK_S  0

#define DPORT_PMS_DMA_TX_I_1_REG          (DR_REG_SENSITIVE_BASE + 0x0B0)
/* DPORT_PMS_DMA_TX_I_SRAM_4_H_W : R/W ;bitpos:[28] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_4_H_W  (BIT(28))
#define DPORT_PMS_DMA_TX_I_SRAM_4_H_W_M  (BIT(28))
#define DPORT_PMS_DMA_TX_I_SRAM_4_H_W_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_4_H_W_S  28
/* DPORT_PMS_DMA_TX_I_SRAM_4_H_R : R/W ;bitpos:[27] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_4_H_R  (BIT(27))
#define DPORT_PMS_DMA_TX_I_SRAM_4_H_R_M  (BIT(27))
#define DPORT_PMS_DMA_TX_I_SRAM_4_H_R_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_4_H_R_S  27
/* DPORT_PMS_DMA_TX_I_SRAM_4_L_W : R/W ;bitpos:[26] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_4_L_W  (BIT(26))
#define DPORT_PMS_DMA_TX_I_SRAM_4_L_W_M  (BIT(26))
#define DPORT_PMS_DMA_TX_I_SRAM_4_L_W_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_4_L_W_S  26
/* DPORT_PMS_DMA_TX_I_SRAM_4_L_R : R/W ;bitpos:[25] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_4_L_R  (BIT(25))
#define DPORT_PMS_DMA_TX_I_SRAM_4_L_R_M  (BIT(25))
#define DPORT_PMS_DMA_TX_I_SRAM_4_L_R_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_4_L_R_S  25
/* DPORT_PMS_DMA_TX_I_SRAM_4_SPLTADDR : R/W ;bitpos:[24:8] ;default: 17'b0 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_4_SPLTADDR  0x0001FFFF
#define DPORT_PMS_DMA_TX_I_SRAM_4_SPLTADDR_M  ((DPORT_PMS_DMA_TX_I_SRAM_4_SPLTADDR_V)<<(DPORT_PMS_DMA_TX_I_SRAM_4_SPLTADDR_S))
#define DPORT_PMS_DMA_TX_I_SRAM_4_SPLTADDR_V  0x1FFFF
#define DPORT_PMS_DMA_TX_I_SRAM_4_SPLTADDR_S  8
/* DPORT_PMS_DMA_TX_I_SRAM_3_W : R/W ;bitpos:[7] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_3_W  (BIT(7))
#define DPORT_PMS_DMA_TX_I_SRAM_3_W_M  (BIT(7))
#define DPORT_PMS_DMA_TX_I_SRAM_3_W_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_3_W_S  7
/* DPORT_PMS_DMA_TX_I_SRAM_3_R : R/W ;bitpos:[6] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_3_R  (BIT(6))
#define DPORT_PMS_DMA_TX_I_SRAM_3_R_M  (BIT(6))
#define DPORT_PMS_DMA_TX_I_SRAM_3_R_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_3_R_S  6
/* DPORT_PMS_DMA_TX_I_SRAM_2_W : R/W ;bitpos:[5] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_2_W  (BIT(5))
#define DPORT_PMS_DMA_TX_I_SRAM_2_W_M  (BIT(5))
#define DPORT_PMS_DMA_TX_I_SRAM_2_W_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_2_W_S  5
/* DPORT_PMS_DMA_TX_I_SRAM_2_R : R/W ;bitpos:[4] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_2_R  (BIT(4))
#define DPORT_PMS_DMA_TX_I_SRAM_2_R_M  (BIT(4))
#define DPORT_PMS_DMA_TX_I_SRAM_2_R_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_2_R_S  4
/* DPORT_PMS_DMA_TX_I_SRAM_1_W : R/W ;bitpos:[3] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_1_W  (BIT(3))
#define DPORT_PMS_DMA_TX_I_SRAM_1_W_M  (BIT(3))
#define DPORT_PMS_DMA_TX_I_SRAM_1_W_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_1_W_S  3
/* DPORT_PMS_DMA_TX_I_SRAM_1_R : R/W ;bitpos:[2] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_1_R  (BIT(2))
#define DPORT_PMS_DMA_TX_I_SRAM_1_R_M  (BIT(2))
#define DPORT_PMS_DMA_TX_I_SRAM_1_R_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_1_R_S  2
/* DPORT_PMS_DMA_TX_I_SRAM_0_W : R/W ;bitpos:[1] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_0_W  (BIT(1))
#define DPORT_PMS_DMA_TX_I_SRAM_0_W_M  (BIT(1))
#define DPORT_PMS_DMA_TX_I_SRAM_0_W_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_0_W_S  1
/* DPORT_PMS_DMA_TX_I_SRAM_0_R : R/W ;bitpos:[0] ;default: 1'b1 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_SRAM_0_R  (BIT(0))
#define DPORT_PMS_DMA_TX_I_SRAM_0_R_M  (BIT(0))
#define DPORT_PMS_DMA_TX_I_SRAM_0_R_V  0x1
#define DPORT_PMS_DMA_TX_I_SRAM_0_R_S  0

#define DPORT_PMS_DMA_TX_I_2_REG          (DR_REG_SENSITIVE_BASE + 0x0B4)
/* DPORT_PMS_DMA_TX_I_ILG_INTR : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_ILG_INTR  (BIT(2))
#define DPORT_PMS_DMA_TX_I_ILG_INTR_M  (BIT(2))
#define DPORT_PMS_DMA_TX_I_ILG_INTR_V  0x1
#define DPORT_PMS_DMA_TX_I_ILG_INTR_S  2
/* DPORT_PMS_DMA_TX_I_ILG_EN : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_ILG_EN  (BIT(1))
#define DPORT_PMS_DMA_TX_I_ILG_EN_M  (BIT(1))
#define DPORT_PMS_DMA_TX_I_ILG_EN_V  0x1
#define DPORT_PMS_DMA_TX_I_ILG_EN_S  1
/* DPORT_PMS_DMA_TX_I_ILG_CLR : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_ILG_CLR  (BIT(0))
#define DPORT_PMS_DMA_TX_I_ILG_CLR_M  (BIT(0))
#define DPORT_PMS_DMA_TX_I_ILG_CLR_V  0x1
#define DPORT_PMS_DMA_TX_I_ILG_CLR_S  0

#define DPORT_PMS_DMA_TX_I_3_REG          (DR_REG_SENSITIVE_BASE + 0x0B8)
/* DPORT_PMS_DMA_TX_I_ILG_ST : RO ;bitpos:[22:0] ;default: 23'b0 ; */
/*description: */
#define DPORT_PMS_DMA_TX_I_ILG_ST  0x007FFFFF
#define DPORT_PMS_DMA_TX_I_ILG_ST_M  ((DPORT_PMS_DMA_TX_I_ILG_ST_V)<<(DPORT_PMS_DMA_TX_I_ILG_ST_S))
#define DPORT_PMS_DMA_TX_I_ILG_ST_V  0x7FFFFF
#define DPORT_PMS_DMA_TX_I_ILG_ST_S  0

#define DPORT_PRO_BOOT_LOCATION_0_REG          (DR_REG_SENSITIVE_BASE + 0x0BC)
/* DPORT_PRO_BOOT_LOCATION_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_BOOT_LOCATION_LOCK  (BIT(0))
#define DPORT_PRO_BOOT_LOCATION_LOCK_M  (BIT(0))
#define DPORT_PRO_BOOT_LOCATION_LOCK_V  0x1
#define DPORT_PRO_BOOT_LOCATION_LOCK_S  0

#define DPORT_PRO_BOOT_LOCATION_1_REG          (DR_REG_SENSITIVE_BASE + 0x0C0)
/* DPORT_PRO_BOOT_REMAP : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_BOOT_REMAP  (BIT(0))
#define DPORT_PRO_BOOT_REMAP_M  (BIT(0))
#define DPORT_PRO_BOOT_REMAP_V  0x1
#define DPORT_PRO_BOOT_REMAP_S  0

#define DPORT_CACHE_SOURCE_0_REG          (DR_REG_SENSITIVE_BASE + 0x0C4)
/* DPORT_CACHE_SOURCE_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_CACHE_SOURCE_LOCK  (BIT(0))
#define DPORT_CACHE_SOURCE_LOCK_M  (BIT(0))
#define DPORT_CACHE_SOURCE_LOCK_V  0x1
#define DPORT_CACHE_SOURCE_LOCK_S  0

#define DPORT_CACHE_SOURCE_1_REG          (DR_REG_SENSITIVE_BASE + 0x0C8)
/* DPORT_PRO_CACHE_D_SOURCE_PRO_DROM0 : R/W ;bitpos:[5] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DROM0  (BIT(5))
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DROM0_M  (BIT(5))
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DROM0_V  0x1
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DROM0_S  5
/* DPORT_PRO_CACHE_D_SOURCE_PRO_DPORT : R/W ;bitpos:[4] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DPORT  (BIT(4))
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DPORT_M  (BIT(4))
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DPORT_V  0x1
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DPORT_S  4
/* DPORT_PRO_CACHE_D_SOURCE_PRO_DRAM0 : R/W ;bitpos:[3] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DRAM0  (BIT(3))
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DRAM0_M  (BIT(3))
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DRAM0_V  0x1
#define DPORT_PRO_CACHE_D_SOURCE_PRO_DRAM0_S  3
/* DPORT_PRO_CACHE_I_SOURCE_PRO_DROM0 : R/W ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_CACHE_I_SOURCE_PRO_DROM0  (BIT(2))
#define DPORT_PRO_CACHE_I_SOURCE_PRO_DROM0_M  (BIT(2))
#define DPORT_PRO_CACHE_I_SOURCE_PRO_DROM0_V  0x1
#define DPORT_PRO_CACHE_I_SOURCE_PRO_DROM0_S  2
/* DPORT_PRO_CACHE_I_SOURCE_PRO_IROM0 : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_CACHE_I_SOURCE_PRO_IROM0  (BIT(1))
#define DPORT_PRO_CACHE_I_SOURCE_PRO_IROM0_M  (BIT(1))
#define DPORT_PRO_CACHE_I_SOURCE_PRO_IROM0_V  0x1
#define DPORT_PRO_CACHE_I_SOURCE_PRO_IROM0_S  1
/* DPORT_PRO_CACHE_I_SOURCE_PRO_IRAM1 : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_CACHE_I_SOURCE_PRO_IRAM1  (BIT(0))
#define DPORT_PRO_CACHE_I_SOURCE_PRO_IRAM1_M  (BIT(0))
#define DPORT_PRO_CACHE_I_SOURCE_PRO_IRAM1_V  0x1
#define DPORT_PRO_CACHE_I_SOURCE_PRO_IRAM1_S  0

#define DPORT_APB_PERIPHERAL_0_REG          (DR_REG_SENSITIVE_BASE + 0x0CC)
/* DPORT_APB_PERIPHERAL_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_APB_PERIPHERAL_LOCK  (BIT(0))
#define DPORT_APB_PERIPHERAL_LOCK_M  (BIT(0))
#define DPORT_APB_PERIPHERAL_LOCK_V  0x1
#define DPORT_APB_PERIPHERAL_LOCK_S  0

#define DPORT_APB_PERIPHERAL_1_REG          (DR_REG_SENSITIVE_BASE + 0x0D0)
/* DPORT_APB_PERIPHERAL_SPLIT_BURST : R/W ;bitpos:[0] ;default: 1'b1 ; */
/*description: */
#define DPORT_APB_PERIPHERAL_SPLIT_BURST  (BIT(0))
#define DPORT_APB_PERIPHERAL_SPLIT_BURST_M  (BIT(0))
#define DPORT_APB_PERIPHERAL_SPLIT_BURST_V  0x1
#define DPORT_APB_PERIPHERAL_SPLIT_BURST_S  0

#define DPORT_PMS_OCCUPY_0_REG          (DR_REG_SENSITIVE_BASE + 0x0D4)
/* DPORT_PMS_OCCUPY_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PMS_OCCUPY_LOCK  (BIT(0))
#define DPORT_PMS_OCCUPY_LOCK_M  (BIT(0))
#define DPORT_PMS_OCCUPY_LOCK_V  0x1
#define DPORT_PMS_OCCUPY_LOCK_S  0

#define DPORT_PMS_OCCUPY_1_REG          (DR_REG_SENSITIVE_BASE + 0x0D8)
/* DPORT_PMS_OCCUPY_CACHE : R/W ;bitpos:[3:0] ;default: 4'b0000 ; */
/*description: */
#define DPORT_PMS_OCCUPY_CACHE  0x0000000F
#define DPORT_PMS_OCCUPY_CACHE_M  ((DPORT_PMS_OCCUPY_CACHE_V)<<(DPORT_PMS_OCCUPY_CACHE_S))
#define DPORT_PMS_OCCUPY_CACHE_V  0xF
#define DPORT_PMS_OCCUPY_CACHE_S  0

#define DPORT_PMS_OCCUPY_2_REG          (DR_REG_SENSITIVE_BASE + 0x0DC)
/* DPORT_PMS_OCCUPY_MAC_DUMP : R/W ;bitpos:[3:0] ;default: 4'b0000 ; */
/*description: */
#define DPORT_PMS_OCCUPY_MAC_DUMP  0x0000000F
#define DPORT_PMS_OCCUPY_MAC_DUMP_M  ((DPORT_PMS_OCCUPY_MAC_DUMP_V)<<(DPORT_PMS_OCCUPY_MAC_DUMP_S))
#define DPORT_PMS_OCCUPY_MAC_DUMP_V  0xF
#define DPORT_PMS_OCCUPY_MAC_DUMP_S  0

#define DPORT_PMS_OCCUPY_3_REG          (DR_REG_SENSITIVE_BASE + 0x0E0)
/* DPORT_PMS_OCCUPY_PRO_TRACE : R/W ;bitpos:[17:0] ;default: 18'b0 ; */
/*description: */
#define DPORT_PMS_OCCUPY_PRO_TRACE  0x0003FFFF
#define DPORT_PMS_OCCUPY_PRO_TRACE_M  ((DPORT_PMS_OCCUPY_PRO_TRACE_V)<<(DPORT_PMS_OCCUPY_PRO_TRACE_S))
#define DPORT_PMS_OCCUPY_PRO_TRACE_V  0x3FFFF
#define DPORT_PMS_OCCUPY_PRO_TRACE_S  0

#define DPORT_CACHE_TAG_ACCESS_0_REG          (DR_REG_SENSITIVE_BASE + 0x0E4)
/* DPORT_CACHE_TAG_ACCESS_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_CACHE_TAG_ACCESS_LOCK  (BIT(0))
#define DPORT_CACHE_TAG_ACCESS_LOCK_M  (BIT(0))
#define DPORT_CACHE_TAG_ACCESS_LOCK_V  0x1
#define DPORT_CACHE_TAG_ACCESS_LOCK_S  0

#define DPORT_CACHE_TAG_ACCESS_1_REG          (DR_REG_SENSITIVE_BASE + 0x0E8)
/* DPORT_PRO_D_TAG_WR_ACS : R/W ;bitpos:[3] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_D_TAG_WR_ACS  (BIT(3))
#define DPORT_PRO_D_TAG_WR_ACS_M  (BIT(3))
#define DPORT_PRO_D_TAG_WR_ACS_V  0x1
#define DPORT_PRO_D_TAG_WR_ACS_S  3
/* DPORT_PRO_D_TAG_RD_ACS : R/W ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_D_TAG_RD_ACS  (BIT(2))
#define DPORT_PRO_D_TAG_RD_ACS_M  (BIT(2))
#define DPORT_PRO_D_TAG_RD_ACS_V  0x1
#define DPORT_PRO_D_TAG_RD_ACS_S  2
/* DPORT_PRO_I_TAG_WR_ACS : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_I_TAG_WR_ACS  (BIT(1))
#define DPORT_PRO_I_TAG_WR_ACS_M  (BIT(1))
#define DPORT_PRO_I_TAG_WR_ACS_V  0x1
#define DPORT_PRO_I_TAG_WR_ACS_S  1
/* DPORT_PRO_I_TAG_RD_ACS : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_PRO_I_TAG_RD_ACS  (BIT(0))
#define DPORT_PRO_I_TAG_RD_ACS_M  (BIT(0))
#define DPORT_PRO_I_TAG_RD_ACS_V  0x1
#define DPORT_PRO_I_TAG_RD_ACS_S  0

#define DPORT_CACHE_MMU_ACCESS_0_REG          (DR_REG_SENSITIVE_BASE + 0x0EC)
/* DPORT_CACHE_MMU_ACCESS_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_CACHE_MMU_ACCESS_LOCK  (BIT(0))
#define DPORT_CACHE_MMU_ACCESS_LOCK_M  (BIT(0))
#define DPORT_CACHE_MMU_ACCESS_LOCK_V  0x1
#define DPORT_CACHE_MMU_ACCESS_LOCK_S  0

#define DPORT_CACHE_MMU_ACCESS_1_REG          (DR_REG_SENSITIVE_BASE + 0x0F0)
/* DPORT_PRO_MMU_WR_ACS : R/W ;bitpos:[1] ;default: 1'b1 ; */
/*description: */
#define DPORT_PRO_MMU_WR_ACS  (BIT(1))
#define DPORT_PRO_MMU_WR_ACS_M  (BIT(1))
#define DPORT_PRO_MMU_WR_ACS_V  0x1
#define DPORT_PRO_MMU_WR_ACS_S  1
/* DPORT_PRO_MMU_RD_ACS : R/W ;bitpos:[0] ;default: 1'b1 ; */
/*description: */
#define DPORT_PRO_MMU_RD_ACS  (BIT(0))
#define DPORT_PRO_MMU_RD_ACS_M  (BIT(0))
#define DPORT_PRO_MMU_RD_ACS_V  0x1
#define DPORT_PRO_MMU_RD_ACS_S  0

#define DPORT_APB_PERIPHERAL_INTR_REG          (DR_REG_SENSITIVE_BASE + 0x0F4)
/* DPORT_APB_PERI_BYTE_ERROR_INTR : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_APB_PERI_BYTE_ERROR_INTR  (BIT(2))
#define DPORT_APB_PERI_BYTE_ERROR_INTR_M  (BIT(2))
#define DPORT_APB_PERI_BYTE_ERROR_INTR_V  0x1
#define DPORT_APB_PERI_BYTE_ERROR_INTR_S  2
/* DPORT_APB_PERI_BYTE_ERROR_EN : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_APB_PERI_BYTE_ERROR_EN  (BIT(1))
#define DPORT_APB_PERI_BYTE_ERROR_EN_M  (BIT(1))
#define DPORT_APB_PERI_BYTE_ERROR_EN_V  0x1
#define DPORT_APB_PERI_BYTE_ERROR_EN_S  1
/* DPORT_APB_PERI_BYTE_ERROR_CLR : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_APB_PERI_BYTE_ERROR_CLR  (BIT(0))
#define DPORT_APB_PERI_BYTE_ERROR_CLR_M  (BIT(0))
#define DPORT_APB_PERI_BYTE_ERROR_CLR_V  0x1
#define DPORT_APB_PERI_BYTE_ERROR_CLR_S  0

#define DPORT_APB_PERIPHERAL_STATUS_REG          (DR_REG_SENSITIVE_BASE + 0x0F8)
/* DPORT_APB_PERI_BYTE_ERROR_ADDR : RO ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define DPORT_APB_PERI_BYTE_ERROR_ADDR  0xFFFFFFFF
#define DPORT_APB_PERI_BYTE_ERROR_ADDR_M  ((DPORT_APB_PERI_BYTE_ERROR_ADDR_V)<<(DPORT_APB_PERI_BYTE_ERROR_ADDR_S))
#define DPORT_APB_PERI_BYTE_ERROR_ADDR_V  0xFFFFFFFF
#define DPORT_APB_PERI_BYTE_ERROR_ADDR_S  0

#define DPORT_CPU_PERIPHERAL_INTR_REG          (DR_REG_SENSITIVE_BASE + 0x0FC)
/* DPORT_CPU_PERI_BYTE_ERROR_INTR : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define DPORT_CPU_PERI_BYTE_ERROR_INTR  (BIT(2))
#define DPORT_CPU_PERI_BYTE_ERROR_INTR_M  (BIT(2))
#define DPORT_CPU_PERI_BYTE_ERROR_INTR_V  0x1
#define DPORT_CPU_PERI_BYTE_ERROR_INTR_S  2
/* DPORT_CPU_PERI_BYTE_ERROR_EN : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define DPORT_CPU_PERI_BYTE_ERROR_EN  (BIT(1))
#define DPORT_CPU_PERI_BYTE_ERROR_EN_M  (BIT(1))
#define DPORT_CPU_PERI_BYTE_ERROR_EN_V  0x1
#define DPORT_CPU_PERI_BYTE_ERROR_EN_S  1
/* DPORT_CPU_PERI_BYTE_ERROR_CLR : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define DPORT_CPU_PERI_BYTE_ERROR_CLR  (BIT(0))
#define DPORT_CPU_PERI_BYTE_ERROR_CLR_M  (BIT(0))
#define DPORT_CPU_PERI_BYTE_ERROR_CLR_V  0x1
#define DPORT_CPU_PERI_BYTE_ERROR_CLR_S  0

#define DPORT_CPU_PERIPHERAL_STATUS_REG          (DR_REG_SENSITIVE_BASE + 0x100)
/* DPORT_CPU_PERI_BYTE_ERROR_ADDR : RO ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define DPORT_CPU_PERI_BYTE_ERROR_ADDR  0xFFFFFFFF
#define DPORT_CPU_PERI_BYTE_ERROR_ADDR_M  ((DPORT_CPU_PERI_BYTE_ERROR_ADDR_V)<<(DPORT_CPU_PERI_BYTE_ERROR_ADDR_S))
#define DPORT_CPU_PERI_BYTE_ERROR_ADDR_V  0xFFFFFFFF
#define DPORT_CPU_PERI_BYTE_ERROR_ADDR_S  0

#define SENSITIVE_CLOCK_GATE_REG          (DR_REG_SENSITIVE_BASE + 0x104)
/* SENSITIVE_CLK_EN : R/W ;bitpos:[0] ;default: 1'b1 ; */
/*description: */
#define SENSITIVE_CLK_EN  (BIT(0))
#define SENSITIVE_CLK_EN_M  (BIT(0))
#define SENSITIVE_CLK_EN_V  0x1
#define SENSITIVE_CLK_EN_S  0

#define SENSITIVE_DATE_REG          (DR_REG_SENSITIVE_BASE + 0xFFC)
/* SENSITIVE_DATE : R/W ;bitpos:[27:0] ;default: 28'h1905090 ; */
/*description: */
#define SENSITIVE_DATE  0x0FFFFFFF
#define SENSITIVE_DATE_M  ((SENSITIVE_DATE_V)<<(SENSITIVE_DATE_S))
#define SENSITIVE_DATE_V  0xFFFFFFF
#define SENSITIVE_DATE_S  0

#ifdef __cplusplus
}
#endif

#endif /*_SOC_SENSITIVE_REG_H_ */
