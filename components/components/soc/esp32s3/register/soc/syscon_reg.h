/**
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 *  SPDX-License-Identifier: Apache-2.0
 */
#ifndef _SOC_SYSCON_REG_H_
#define _SOC_SYSCON_REG_H_


#include "soc/soc.h"
#ifdef __cplusplus
extern "C" {
#endif

#define SYSCON_SYSCLK_CONF_REG          (DR_REG_SYSCON_BASE + 0x0)
/* SYSCON_RST_TICK_CNT : R/W ;bitpos:[12] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_RST_TICK_CNT    (BIT(12))
#define SYSCON_RST_TICK_CNT_M  (BIT(12))
#define SYSCON_RST_TICK_CNT_V  0x1
#define SYSCON_RST_TICK_CNT_S  12
/* SYSCON_CLK_EN : R/W ;bitpos:[11] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_CLK_EN    (BIT(11))
#define SYSCON_CLK_EN_M  (BIT(11))
#define SYSCON_CLK_EN_V  0x1
#define SYSCON_CLK_EN_S  11
/* SYSCON_CLK_320M_EN : R/W ;bitpos:[10] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_CLK_320M_EN    (BIT(10))
#define SYSCON_CLK_320M_EN_M  (BIT(10))
#define SYSCON_CLK_320M_EN_V  0x1
#define SYSCON_CLK_320M_EN_S  10
/* SYSCON_PRE_DIV_CNT : R/W ;bitpos:[9:0] ;default: 10'h1 ; */
/*description: .*/
#define SYSCON_PRE_DIV_CNT    0x000003FF
#define SYSCON_PRE_DIV_CNT_M  ((SYSCON_PRE_DIV_CNT_V)<<(SYSCON_PRE_DIV_CNT_S))
#define SYSCON_PRE_DIV_CNT_V  0x3FF
#define SYSCON_PRE_DIV_CNT_S  0

#define SYSCON_TICK_CONF_REG          (DR_REG_SYSCON_BASE + 0x4)
/* SYSCON_TICK_ENABLE : R/W ;bitpos:[16] ;default: 1'd1 ; */
/*description: .*/
#define SYSCON_TICK_ENABLE    (BIT(16))
#define SYSCON_TICK_ENABLE_M  (BIT(16))
#define SYSCON_TICK_ENABLE_V  0x1
#define SYSCON_TICK_ENABLE_S  16
/* SYSCON_CK8M_TICK_NUM : R/W ;bitpos:[15:8] ;default: 8'd7 ; */
/*description: .*/
#define SYSCON_CK8M_TICK_NUM    0x000000FF
#define SYSCON_CK8M_TICK_NUM_M  ((SYSCON_CK8M_TICK_NUM_V)<<(SYSCON_CK8M_TICK_NUM_S))
#define SYSCON_CK8M_TICK_NUM_V  0xFF
#define SYSCON_CK8M_TICK_NUM_S  8
/* SYSCON_XTAL_TICK_NUM : R/W ;bitpos:[7:0] ;default: 8'd39 ; */
/*description: .*/
#define SYSCON_XTAL_TICK_NUM    0x000000FF
#define SYSCON_XTAL_TICK_NUM_M  ((SYSCON_XTAL_TICK_NUM_V)<<(SYSCON_XTAL_TICK_NUM_S))
#define SYSCON_XTAL_TICK_NUM_V  0xFF
#define SYSCON_XTAL_TICK_NUM_S  0

#define SYSCON_CLK_OUT_EN_REG          (DR_REG_SYSCON_BASE + 0x8)
/* SYSCON_CLK_XTAL_OEN : R/W ;bitpos:[10] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_CLK_XTAL_OEN    (BIT(10))
#define SYSCON_CLK_XTAL_OEN_M  (BIT(10))
#define SYSCON_CLK_XTAL_OEN_V  0x1
#define SYSCON_CLK_XTAL_OEN_S  10
/* SYSCON_CLK40X_BB_OEN : R/W ;bitpos:[9] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_CLK40X_BB_OEN    (BIT(9))
#define SYSCON_CLK40X_BB_OEN_M  (BIT(9))
#define SYSCON_CLK40X_BB_OEN_V  0x1
#define SYSCON_CLK40X_BB_OEN_S  9
/* SYSCON_CLK_DAC_CPU_OEN : R/W ;bitpos:[8] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_CLK_DAC_CPU_OEN    (BIT(8))
#define SYSCON_CLK_DAC_CPU_OEN_M  (BIT(8))
#define SYSCON_CLK_DAC_CPU_OEN_V  0x1
#define SYSCON_CLK_DAC_CPU_OEN_S  8
/* SYSCON_CLK_ADC_INF_OEN : R/W ;bitpos:[7] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_CLK_ADC_INF_OEN    (BIT(7))
#define SYSCON_CLK_ADC_INF_OEN_M  (BIT(7))
#define SYSCON_CLK_ADC_INF_OEN_V  0x1
#define SYSCON_CLK_ADC_INF_OEN_S  7
/* SYSCON_CLK_320M_OEN : R/W ;bitpos:[6] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_CLK_320M_OEN    (BIT(6))
#define SYSCON_CLK_320M_OEN_M  (BIT(6))
#define SYSCON_CLK_320M_OEN_V  0x1
#define SYSCON_CLK_320M_OEN_S  6
/* SYSCON_CLK160_OEN : R/W ;bitpos:[5] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_CLK160_OEN    (BIT(5))
#define SYSCON_CLK160_OEN_M  (BIT(5))
#define SYSCON_CLK160_OEN_V  0x1
#define SYSCON_CLK160_OEN_S  5
/* SYSCON_CLK80_OEN : R/W ;bitpos:[4] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_CLK80_OEN    (BIT(4))
#define SYSCON_CLK80_OEN_M  (BIT(4))
#define SYSCON_CLK80_OEN_V  0x1
#define SYSCON_CLK80_OEN_S  4
/* SYSCON_CLK_BB_OEN : R/W ;bitpos:[3] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_CLK_BB_OEN    (BIT(3))
#define SYSCON_CLK_BB_OEN_M  (BIT(3))
#define SYSCON_CLK_BB_OEN_V  0x1
#define SYSCON_CLK_BB_OEN_S  3
/* SYSCON_CLK44_OEN : R/W ;bitpos:[2] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_CLK44_OEN    (BIT(2))
#define SYSCON_CLK44_OEN_M  (BIT(2))
#define SYSCON_CLK44_OEN_V  0x1
#define SYSCON_CLK44_OEN_S  2
/* SYSCON_CLK22_OEN : R/W ;bitpos:[1] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_CLK22_OEN    (BIT(1))
#define SYSCON_CLK22_OEN_M  (BIT(1))
#define SYSCON_CLK22_OEN_V  0x1
#define SYSCON_CLK22_OEN_S  1
/* SYSCON_CLK20_OEN : R/W ;bitpos:[0] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_CLK20_OEN    (BIT(0))
#define SYSCON_CLK20_OEN_M  (BIT(0))
#define SYSCON_CLK20_OEN_V  0x1
#define SYSCON_CLK20_OEN_S  0

#define SYSCON_WIFI_BB_CFG_REG          (DR_REG_SYSCON_BASE + 0xC)
/* SYSCON_WIFI_BB_CFG : R/W ;bitpos:[31:0] ;default: 32'h0 ; */
/*description: .*/
#define SYSCON_WIFI_BB_CFG    0xFFFFFFFF
#define SYSCON_WIFI_BB_CFG_M  ((SYSCON_WIFI_BB_CFG_V)<<(SYSCON_WIFI_BB_CFG_S))
#define SYSCON_WIFI_BB_CFG_V  0xFFFFFFFF
#define SYSCON_WIFI_BB_CFG_S  0

#define SYSCON_WIFI_BB_CFG_2_REG          (DR_REG_SYSCON_BASE + 0x10)
/* SYSCON_WIFI_BB_CFG_2 : R/W ;bitpos:[31:0] ;default: 32'h0 ; */
/*description: .*/
#define SYSCON_WIFI_BB_CFG_2    0xFFFFFFFF
#define SYSCON_WIFI_BB_CFG_2_M  ((SYSCON_WIFI_BB_CFG_2_V)<<(SYSCON_WIFI_BB_CFG_2_S))
#define SYSCON_WIFI_BB_CFG_2_V  0xFFFFFFFF
#define SYSCON_WIFI_BB_CFG_2_S  0

#define SYSCON_WIFI_CLK_EN_REG          (DR_REG_SYSCON_BASE + 0x14)
/* SYSCON_WIFI_CLK_EN : R/W ;bitpos:[31:0] ;default: 32'hfffce030 ; */
/*description: .*/
#define SYSCON_WIFI_CLK_EN    0xFFFFFFFF
#define SYSCON_WIFI_CLK_EN_M  ((SYSCON_WIFI_CLK_EN_V)<<(SYSCON_WIFI_CLK_EN_S))
#define SYSCON_WIFI_CLK_EN_V  0xFFFFFFFF
#define SYSCON_WIFI_CLK_EN_S  0

#define SYSCON_WIFI_RST_EN_REG          (DR_REG_SYSCON_BASE + 0x18)
/* SYSCON_WIFI_RST : R/W ;bitpos:[31:0] ;default: 32'h0 ; */
/*description: .*/
#define SYSCON_WIFI_RST    0xFFFFFFFF
#define SYSCON_WIFI_RST_M  ((SYSCON_WIFI_RST_V)<<(SYSCON_WIFI_RST_S))
#define SYSCON_WIFI_RST_V  0xFFFFFFFF
#define SYSCON_WIFI_RST_S  0

#define SYSTEM_WIFI_CLK_EN_REG SYSCON_WIFI_CLK_EN_REG
/* SYSTEM_WIFI_CLK_EN : R/W ;bitpos:[31:0] ;default: 32'hfffce030 ; */
/*description: */
#define SYSTEM_WIFI_CLK_EN 0x00FB9FCF
#define SYSTEM_WIFI_CLK_EN_M ((SYSTEM_WIFI_CLK_EN_V) << (SYSTEM_WIFI_CLK_EN_S))
#define SYSTEM_WIFI_CLK_EN_V 0x00FB9FCF
#define SYSTEM_WIFI_CLK_EN_S 0

/* Mask for all Wifi clock bits, 6 */
#define SYSTEM_WIFI_CLK_WIFI_EN  0x0
#define SYSTEM_WIFI_CLK_WIFI_EN_M  ((SYSTEM_WIFI_CLK_WIFI_EN_V)<<(SYSTEM_WIFI_CLK_WIFI_EN_S))
#define SYSTEM_WIFI_CLK_WIFI_EN_V  0x0
#define SYSTEM_WIFI_CLK_WIFI_EN_S  0
/* Mask for all Bluetooth clock bits, 11, 16, 17 */
#define SYSTEM_WIFI_CLK_BT_EN  0x0
#define SYSTEM_WIFI_CLK_BT_EN_M  ((SYSTEM_WIFI_CLK_BT_EN_V)<<(SYSTEM_WIFI_CLK_BT_EN_S))
#define SYSTEM_WIFI_CLK_BT_EN_V  0x0
#define SYSTEM_WIFI_CLK_BT_EN_S  0
/* Mask for clock bits used by both WIFI and Bluetooth, 0, 1, 2, 3, 7, 8, 9, 10, 19, 20, 21, 22, 23 */
#define SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M 0x78078F

//bluetooth baseband bit11
#define SYSTEM_BT_BASEBAND_EN BIT(11)
//bluetooth LC bit16 and bit17
#define SYSTEM_BT_LC_EN (BIT(16) | BIT(17))

/* Remaining single bit clock masks */
#define SYSTEM_WIFI_CLK_I2C_CLK_EN BIT(5)
#define SYSTEM_WIFI_CLK_UNUSED_BIT12 BIT(12)
#define SYSTEM_WIFI_CLK_SDIO_HOST_EN BIT(13)
#define SYSTEM_WIFI_CLK_EMAC_EN BIT(14)
#define SYSTEM_WIFI_CLK_RNG_EN BIT(15)

#define SYSTEM_CORE_RST_EN_REG SYSTEM_WIFI_RST_EN_REG
#define SYSTEM_WIFI_RST_EN_REG SYSCON_WIFI_RST_EN_REG
/* SYSTEM_WIFI_RST : R/W ;bitpos:[31:0] ;default: 32'h0 ; */
/*description: */
#define SYSTEM_WIFI_RST 0xFFFFFFFF
#define SYSTEM_WIFI_RST_M ((SYSTEM_WIFI_RST_V) << (SYSTEM_WIFI_RST_S))
#define SYSTEM_WIFI_RST_V 0xFFFFFFFF
#define SYSTEM_WIFI_RST_S 0

#define SYSTEM_WIFIBB_RST           BIT(0)
#define SYSTEM_FE_RST               BIT(1)
#define SYSTEM_WIFIMAC_RST          BIT(2)
#define SYSTEM_BTBB_RST             BIT(3)    /* Bluetooth Baseband */
#define SYSTEM_BTMAC_RST            BIT(4)    /* deprecated */
#define SYSTEM_SDIO_RST             BIT(5)
#define SYSTEM_EMAC_RST             BIT(7)
#define SYSTEM_MACPWR_RST           BIT(8)
#define SYSTEM_RW_BTMAC_RST         BIT(9)    /* Bluetooth MAC */
#define SYSTEM_RW_BTLP_RST          BIT(10)   /* Bluetooth Low Power Module */
#define SYSTEM_RW_BTMAC_REG_RST     BIT(11)   /* Bluetooth MAC Registers */
#define SYSTEM_RW_BTLP_REG_RST      BIT(12)   /* Bluetooth Low Power Registers */
#define SYSTEM_BTBB_REG_RST         BIT(13)   /* Bluetooth Baseband Registers */

#define MODEM_RESET_FIELD_WHEN_PU   (SYSTEM_WIFIBB_RST       | \
                                     SYSTEM_FE_RST           | \
                                     SYSTEM_WIFIMAC_RST      | \
                                     SYSTEM_BTBB_RST         | \
                                     SYSTEM_BTMAC_RST        | \
                                     SYSTEM_RW_BTMAC_RST     | \
                                     SYSTEM_RW_BTMAC_REG_RST | \
                                     SYSTEM_BTBB_REG_RST)

#define SYSCON_HOST_INF_SEL_REG          (DR_REG_SYSCON_BASE + 0x1C)
/* SYSCON_PERI_IO_SWAP : R/W ;bitpos:[7:0] ;default: 8'h0 ; */
/*description: .*/
#define SYSCON_PERI_IO_SWAP    0x000000FF
#define SYSCON_PERI_IO_SWAP_M  ((SYSCON_PERI_IO_SWAP_V)<<(SYSCON_PERI_IO_SWAP_S))
#define SYSCON_PERI_IO_SWAP_V  0xFF
#define SYSCON_PERI_IO_SWAP_S  0

#define SYSCON_EXT_MEM_PMS_LOCK_REG          (DR_REG_SYSCON_BASE + 0x20)
/* SYSCON_EXT_MEM_PMS_LOCK : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_EXT_MEM_PMS_LOCK    (BIT(0))
#define SYSCON_EXT_MEM_PMS_LOCK_M  (BIT(0))
#define SYSCON_EXT_MEM_PMS_LOCK_V  0x1
#define SYSCON_EXT_MEM_PMS_LOCK_S  0

#define SYSCON_EXT_MEM_WRITEBACK_BYPASS_REG          (DR_REG_SYSCON_BASE + 0x24)
/* SYSCON_WRITEBACK_BYPASS : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: Set 1 to bypass cache writeback request to external memory so that spi will not
check its attribute..*/
#define SYSCON_WRITEBACK_BYPASS    (BIT(0))
#define SYSCON_WRITEBACK_BYPASS_M  (BIT(0))
#define SYSCON_WRITEBACK_BYPASS_V  0x1
#define SYSCON_WRITEBACK_BYPASS_S  0

#define SYSCON_FLASH_ACE0_ATTR_REG          (DR_REG_SYSCON_BASE + 0x28)
/* SYSCON_FLASH_ACE0_ATTR : R/W ;bitpos:[8:0] ;default: 9'hff ; */
/*description: .*/
#define SYSCON_FLASH_ACE0_ATTR    0x000001FF
#define SYSCON_FLASH_ACE0_ATTR_M  ((SYSCON_FLASH_ACE0_ATTR_V)<<(SYSCON_FLASH_ACE0_ATTR_S))
#define SYSCON_FLASH_ACE0_ATTR_V  0x1FF
#define SYSCON_FLASH_ACE0_ATTR_S  0

#define SYSCON_FLASH_ACE1_ATTR_REG          (DR_REG_SYSCON_BASE + 0x2C)
/* SYSCON_FLASH_ACE1_ATTR : R/W ;bitpos:[8:0] ;default: 9'hff ; */
/*description: .*/
#define SYSCON_FLASH_ACE1_ATTR    0x000001FF
#define SYSCON_FLASH_ACE1_ATTR_M  ((SYSCON_FLASH_ACE1_ATTR_V)<<(SYSCON_FLASH_ACE1_ATTR_S))
#define SYSCON_FLASH_ACE1_ATTR_V  0x1FF
#define SYSCON_FLASH_ACE1_ATTR_S  0

#define SYSCON_FLASH_ACE2_ATTR_REG          (DR_REG_SYSCON_BASE + 0x30)
/* SYSCON_FLASH_ACE2_ATTR : R/W ;bitpos:[8:0] ;default: 9'hff ; */
/*description: .*/
#define SYSCON_FLASH_ACE2_ATTR    0x000001FF
#define SYSCON_FLASH_ACE2_ATTR_M  ((SYSCON_FLASH_ACE2_ATTR_V)<<(SYSCON_FLASH_ACE2_ATTR_S))
#define SYSCON_FLASH_ACE2_ATTR_V  0x1FF
#define SYSCON_FLASH_ACE2_ATTR_S  0

#define SYSCON_FLASH_ACE3_ATTR_REG          (DR_REG_SYSCON_BASE + 0x34)
/* SYSCON_FLASH_ACE3_ATTR : R/W ;bitpos:[8:0] ;default: 9'hff ; */
/*description: .*/
#define SYSCON_FLASH_ACE3_ATTR    0x000001FF
#define SYSCON_FLASH_ACE3_ATTR_M  ((SYSCON_FLASH_ACE3_ATTR_V)<<(SYSCON_FLASH_ACE3_ATTR_S))
#define SYSCON_FLASH_ACE3_ATTR_V  0x1FF
#define SYSCON_FLASH_ACE3_ATTR_S  0

#define SYSCON_FLASH_ACE0_ADDR_REG          (DR_REG_SYSCON_BASE + 0x38)
/* SYSCON_FLASH_ACE0_ADDR_S : R/W ;bitpos:[31:0] ;default: 32'h0 ; */
/*description: .*/
#define SYSCON_FLASH_ACE0_ADDR_S    0xFFFFFFFF
#define SYSCON_FLASH_ACE0_ADDR_S_M  ((SYSCON_FLASH_ACE0_ADDR_S_V)<<(SYSCON_FLASH_ACE0_ADDR_S_S))
#define SYSCON_FLASH_ACE0_ADDR_S_V  0xFFFFFFFF
#define SYSCON_FLASH_ACE0_ADDR_S_S  0

#define SYSCON_FLASH_ACE1_ADDR_REG          (DR_REG_SYSCON_BASE + 0x3C)
/* SYSCON_FLASH_ACE1_ADDR_S : R/W ;bitpos:[31:0] ;default: 32'h10000000 ; */
/*description: .*/
#define SYSCON_FLASH_ACE1_ADDR_S    0xFFFFFFFF
#define SYSCON_FLASH_ACE1_ADDR_S_M  ((SYSCON_FLASH_ACE1_ADDR_S_V)<<(SYSCON_FLASH_ACE1_ADDR_S_S))
#define SYSCON_FLASH_ACE1_ADDR_S_V  0xFFFFFFFF
#define SYSCON_FLASH_ACE1_ADDR_S_S  0

#define SYSCON_FLASH_ACE2_ADDR_REG          (DR_REG_SYSCON_BASE + 0x40)
/* SYSCON_FLASH_ACE2_ADDR_S : R/W ;bitpos:[31:0] ;default: 32'h20000000 ; */
/*description: .*/
#define SYSCON_FLASH_ACE2_ADDR_S    0xFFFFFFFF
#define SYSCON_FLASH_ACE2_ADDR_S_M  ((SYSCON_FLASH_ACE2_ADDR_S_V)<<(SYSCON_FLASH_ACE2_ADDR_S_S))
#define SYSCON_FLASH_ACE2_ADDR_S_V  0xFFFFFFFF
#define SYSCON_FLASH_ACE2_ADDR_S_S  0

#define SYSCON_FLASH_ACE3_ADDR_REG          (DR_REG_SYSCON_BASE + 0x44)
/* SYSCON_FLASH_ACE3_ADDR_S : R/W ;bitpos:[31:0] ;default: 32'h30000000 ; */
/*description: .*/
#define SYSCON_FLASH_ACE3_ADDR_S    0xFFFFFFFF
#define SYSCON_FLASH_ACE3_ADDR_S_M  ((SYSCON_FLASH_ACE3_ADDR_S_V)<<(SYSCON_FLASH_ACE3_ADDR_S_S))
#define SYSCON_FLASH_ACE3_ADDR_S_V  0xFFFFFFFF
#define SYSCON_FLASH_ACE3_ADDR_S_S  0

#define SYSCON_FLASH_ACE0_SIZE_REG          (DR_REG_SYSCON_BASE + 0x48)
/* SYSCON_FLASH_ACE0_SIZE : R/W ;bitpos:[15:0] ;default: 16'h1000 ; */
/*description: .*/
#define SYSCON_FLASH_ACE0_SIZE    0x0000FFFF
#define SYSCON_FLASH_ACE0_SIZE_M  ((SYSCON_FLASH_ACE0_SIZE_V)<<(SYSCON_FLASH_ACE0_SIZE_S))
#define SYSCON_FLASH_ACE0_SIZE_V  0xFFFF
#define SYSCON_FLASH_ACE0_SIZE_S  0

#define SYSCON_FLASH_ACE1_SIZE_REG          (DR_REG_SYSCON_BASE + 0x4C)
/* SYSCON_FLASH_ACE1_SIZE : R/W ;bitpos:[15:0] ;default: 16'h1000 ; */
/*description: .*/
#define SYSCON_FLASH_ACE1_SIZE    0x0000FFFF
#define SYSCON_FLASH_ACE1_SIZE_M  ((SYSCON_FLASH_ACE1_SIZE_V)<<(SYSCON_FLASH_ACE1_SIZE_S))
#define SYSCON_FLASH_ACE1_SIZE_V  0xFFFF
#define SYSCON_FLASH_ACE1_SIZE_S  0

#define SYSCON_FLASH_ACE2_SIZE_REG          (DR_REG_SYSCON_BASE + 0x50)
/* SYSCON_FLASH_ACE2_SIZE : R/W ;bitpos:[15:0] ;default: 16'h1000 ; */
/*description: .*/
#define SYSCON_FLASH_ACE2_SIZE    0x0000FFFF
#define SYSCON_FLASH_ACE2_SIZE_M  ((SYSCON_FLASH_ACE2_SIZE_V)<<(SYSCON_FLASH_ACE2_SIZE_S))
#define SYSCON_FLASH_ACE2_SIZE_V  0xFFFF
#define SYSCON_FLASH_ACE2_SIZE_S  0

#define SYSCON_FLASH_ACE3_SIZE_REG          (DR_REG_SYSCON_BASE + 0x54)
/* SYSCON_FLASH_ACE3_SIZE : R/W ;bitpos:[15:0] ;default: 16'h1000 ; */
/*description: .*/
#define SYSCON_FLASH_ACE3_SIZE    0x0000FFFF
#define SYSCON_FLASH_ACE3_SIZE_M  ((SYSCON_FLASH_ACE3_SIZE_V)<<(SYSCON_FLASH_ACE3_SIZE_S))
#define SYSCON_FLASH_ACE3_SIZE_V  0xFFFF
#define SYSCON_FLASH_ACE3_SIZE_S  0

#define SYSCON_SRAM_ACE0_ATTR_REG          (DR_REG_SYSCON_BASE + 0x58)
/* SYSCON_SRAM_ACE0_ATTR : R/W ;bitpos:[8:0] ;default: 9'hff ; */
/*description: .*/
#define SYSCON_SRAM_ACE0_ATTR    0x000001FF
#define SYSCON_SRAM_ACE0_ATTR_M  ((SYSCON_SRAM_ACE0_ATTR_V)<<(SYSCON_SRAM_ACE0_ATTR_S))
#define SYSCON_SRAM_ACE0_ATTR_V  0x1FF
#define SYSCON_SRAM_ACE0_ATTR_S  0

#define SYSCON_SRAM_ACE1_ATTR_REG          (DR_REG_SYSCON_BASE + 0x5C)
/* SYSCON_SRAM_ACE1_ATTR : R/W ;bitpos:[8:0] ;default: 9'hff ; */
/*description: .*/
#define SYSCON_SRAM_ACE1_ATTR    0x000001FF
#define SYSCON_SRAM_ACE1_ATTR_M  ((SYSCON_SRAM_ACE1_ATTR_V)<<(SYSCON_SRAM_ACE1_ATTR_S))
#define SYSCON_SRAM_ACE1_ATTR_V  0x1FF
#define SYSCON_SRAM_ACE1_ATTR_S  0

#define SYSCON_SRAM_ACE2_ATTR_REG          (DR_REG_SYSCON_BASE + 0x60)
/* SYSCON_SRAM_ACE2_ATTR : R/W ;bitpos:[8:0] ;default: 9'hff ; */
/*description: .*/
#define SYSCON_SRAM_ACE2_ATTR    0x000001FF
#define SYSCON_SRAM_ACE2_ATTR_M  ((SYSCON_SRAM_ACE2_ATTR_V)<<(SYSCON_SRAM_ACE2_ATTR_S))
#define SYSCON_SRAM_ACE2_ATTR_V  0x1FF
#define SYSCON_SRAM_ACE2_ATTR_S  0

#define SYSCON_SRAM_ACE3_ATTR_REG          (DR_REG_SYSCON_BASE + 0x64)
/* SYSCON_SRAM_ACE3_ATTR : R/W ;bitpos:[8:0] ;default: 9'hff ; */
/*description: .*/
#define SYSCON_SRAM_ACE3_ATTR    0x000001FF
#define SYSCON_SRAM_ACE3_ATTR_M  ((SYSCON_SRAM_ACE3_ATTR_V)<<(SYSCON_SRAM_ACE3_ATTR_S))
#define SYSCON_SRAM_ACE3_ATTR_V  0x1FF
#define SYSCON_SRAM_ACE3_ATTR_S  0

#define SYSCON_SRAM_ACE0_ADDR_REG          (DR_REG_SYSCON_BASE + 0x68)
/* SYSCON_SRAM_ACE0_ADDR_S : R/W ;bitpos:[31:0] ;default: 32'h0 ; */
/*description: .*/
#define SYSCON_SRAM_ACE0_ADDR_S    0xFFFFFFFF
#define SYSCON_SRAM_ACE0_ADDR_S_M  ((SYSCON_SRAM_ACE0_ADDR_S_V)<<(SYSCON_SRAM_ACE0_ADDR_S_S))
#define SYSCON_SRAM_ACE0_ADDR_S_V  0xFFFFFFFF
#define SYSCON_SRAM_ACE0_ADDR_S_S  0

#define SYSCON_SRAM_ACE1_ADDR_REG          (DR_REG_SYSCON_BASE + 0x6C)
/* SYSCON_SRAM_ACE1_ADDR_S : R/W ;bitpos:[31:0] ;default: 32'h10000000 ; */
/*description: .*/
#define SYSCON_SRAM_ACE1_ADDR_S    0xFFFFFFFF
#define SYSCON_SRAM_ACE1_ADDR_S_M  ((SYSCON_SRAM_ACE1_ADDR_S_V)<<(SYSCON_SRAM_ACE1_ADDR_S_S))
#define SYSCON_SRAM_ACE1_ADDR_S_V  0xFFFFFFFF
#define SYSCON_SRAM_ACE1_ADDR_S_S  0

#define SYSCON_SRAM_ACE2_ADDR_REG          (DR_REG_SYSCON_BASE + 0x70)
/* SYSCON_SRAM_ACE2_ADDR_S : R/W ;bitpos:[31:0] ;default: 32'h20000000 ; */
/*description: .*/
#define SYSCON_SRAM_ACE2_ADDR_S    0xFFFFFFFF
#define SYSCON_SRAM_ACE2_ADDR_S_M  ((SYSCON_SRAM_ACE2_ADDR_S_V)<<(SYSCON_SRAM_ACE2_ADDR_S_S))
#define SYSCON_SRAM_ACE2_ADDR_S_V  0xFFFFFFFF
#define SYSCON_SRAM_ACE2_ADDR_S_S  0

#define SYSCON_SRAM_ACE3_ADDR_REG          (DR_REG_SYSCON_BASE + 0x74)
/* SYSCON_SRAM_ACE3_ADDR_S : R/W ;bitpos:[31:0] ;default: 32'h30000000 ; */
/*description: .*/
#define SYSCON_SRAM_ACE3_ADDR_S    0xFFFFFFFF
#define SYSCON_SRAM_ACE3_ADDR_S_M  ((SYSCON_SRAM_ACE3_ADDR_S_V)<<(SYSCON_SRAM_ACE3_ADDR_S_S))
#define SYSCON_SRAM_ACE3_ADDR_S_V  0xFFFFFFFF
#define SYSCON_SRAM_ACE3_ADDR_S_S  0

#define SYSCON_SRAM_ACE0_SIZE_REG          (DR_REG_SYSCON_BASE + 0x78)
/* SYSCON_SRAM_ACE0_SIZE : R/W ;bitpos:[15:0] ;default: 16'h1000 ; */
/*description: .*/
#define SYSCON_SRAM_ACE0_SIZE    0x0000FFFF
#define SYSCON_SRAM_ACE0_SIZE_M  ((SYSCON_SRAM_ACE0_SIZE_V)<<(SYSCON_SRAM_ACE0_SIZE_S))
#define SYSCON_SRAM_ACE0_SIZE_V  0xFFFF
#define SYSCON_SRAM_ACE0_SIZE_S  0

#define SYSCON_SRAM_ACE1_SIZE_REG          (DR_REG_SYSCON_BASE + 0x7C)
/* SYSCON_SRAM_ACE1_SIZE : R/W ;bitpos:[15:0] ;default: 16'h1000 ; */
/*description: .*/
#define SYSCON_SRAM_ACE1_SIZE    0x0000FFFF
#define SYSCON_SRAM_ACE1_SIZE_M  ((SYSCON_SRAM_ACE1_SIZE_V)<<(SYSCON_SRAM_ACE1_SIZE_S))
#define SYSCON_SRAM_ACE1_SIZE_V  0xFFFF
#define SYSCON_SRAM_ACE1_SIZE_S  0

#define SYSCON_SRAM_ACE2_SIZE_REG          (DR_REG_SYSCON_BASE + 0x80)
/* SYSCON_SRAM_ACE2_SIZE : R/W ;bitpos:[15:0] ;default: 16'h1000 ; */
/*description: .*/
#define SYSCON_SRAM_ACE2_SIZE    0x0000FFFF
#define SYSCON_SRAM_ACE2_SIZE_M  ((SYSCON_SRAM_ACE2_SIZE_V)<<(SYSCON_SRAM_ACE2_SIZE_S))
#define SYSCON_SRAM_ACE2_SIZE_V  0xFFFF
#define SYSCON_SRAM_ACE2_SIZE_S  0

#define SYSCON_SRAM_ACE3_SIZE_REG          (DR_REG_SYSCON_BASE + 0x84)
/* SYSCON_SRAM_ACE3_SIZE : R/W ;bitpos:[15:0] ;default: 16'h1000 ; */
/*description: .*/
#define SYSCON_SRAM_ACE3_SIZE    0x0000FFFF
#define SYSCON_SRAM_ACE3_SIZE_M  ((SYSCON_SRAM_ACE3_SIZE_V)<<(SYSCON_SRAM_ACE3_SIZE_S))
#define SYSCON_SRAM_ACE3_SIZE_V  0xFFFF
#define SYSCON_SRAM_ACE3_SIZE_S  0

#define SYSCON_SPI_MEM_PMS_CTRL_REG          (DR_REG_SYSCON_BASE + 0x88)
/* SYSCON_SPI_MEM_REJECT_CDE : RO ;bitpos:[6:2] ;default: 5'h0 ; */
/*description: .*/
#define SYSCON_SPI_MEM_REJECT_CDE    0x0000001F
#define SYSCON_SPI_MEM_REJECT_CDE_M  ((SYSCON_SPI_MEM_REJECT_CDE_V)<<(SYSCON_SPI_MEM_REJECT_CDE_S))
#define SYSCON_SPI_MEM_REJECT_CDE_V  0x1F
#define SYSCON_SPI_MEM_REJECT_CDE_S  2
/* SYSCON_SPI_MEM_REJECT_CLR : WOD ;bitpos:[1] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_SPI_MEM_REJECT_CLR    (BIT(1))
#define SYSCON_SPI_MEM_REJECT_CLR_M  (BIT(1))
#define SYSCON_SPI_MEM_REJECT_CLR_V  0x1
#define SYSCON_SPI_MEM_REJECT_CLR_S  1
/* SYSCON_SPI_MEM_REJECT_INT : RO ;bitpos:[0] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_SPI_MEM_REJECT_INT    (BIT(0))
#define SYSCON_SPI_MEM_REJECT_INT_M  (BIT(0))
#define SYSCON_SPI_MEM_REJECT_INT_V  0x1
#define SYSCON_SPI_MEM_REJECT_INT_S  0

#define SYSCON_SPI_MEM_REJECT_ADDR_REG          (DR_REG_SYSCON_BASE + 0x8C)
/* SYSCON_SPI_MEM_REJECT_ADDR : RO ;bitpos:[31:0] ;default: 32'h0 ; */
/*description: .*/
#define SYSCON_SPI_MEM_REJECT_ADDR    0xFFFFFFFF
#define SYSCON_SPI_MEM_REJECT_ADDR_M  ((SYSCON_SPI_MEM_REJECT_ADDR_V)<<(SYSCON_SPI_MEM_REJECT_ADDR_S))
#define SYSCON_SPI_MEM_REJECT_ADDR_V  0xFFFFFFFF
#define SYSCON_SPI_MEM_REJECT_ADDR_S  0

#define SYSCON_SDIO_CTRL_REG          (DR_REG_SYSCON_BASE + 0x90)
/* SYSCON_SDIO_WIN_ACCESS_EN : R/W ;bitpos:[0] ;default: 1'h0 ; */
/*description: .*/
#define SYSCON_SDIO_WIN_ACCESS_EN    (BIT(0))
#define SYSCON_SDIO_WIN_ACCESS_EN_M  (BIT(0))
#define SYSCON_SDIO_WIN_ACCESS_EN_V  0x1
#define SYSCON_SDIO_WIN_ACCESS_EN_S  0

#define SYSCON_REDCY_SIG0_REG          (DR_REG_SYSCON_BASE + 0x94)
/* SYSCON_REDCY_ANDOR : RO ;bitpos:[31] ;default: 1'h0 ; */
/*description: .*/
#define SYSCON_REDCY_ANDOR    (BIT(31))
#define SYSCON_REDCY_ANDOR_M  (BIT(31))
#define SYSCON_REDCY_ANDOR_V  0x1
#define SYSCON_REDCY_ANDOR_S  31
/* SYSCON_REDCY_SIG0 : R/W ;bitpos:[30:0] ;default: 31'h0 ; */
/*description: .*/
#define SYSCON_REDCY_SIG0    0x7FFFFFFF
#define SYSCON_REDCY_SIG0_M  ((SYSCON_REDCY_SIG0_V)<<(SYSCON_REDCY_SIG0_S))
#define SYSCON_REDCY_SIG0_V  0x7FFFFFFF
#define SYSCON_REDCY_SIG0_S  0

#define SYSCON_REDCY_SIG1_REG          (DR_REG_SYSCON_BASE + 0x98)
/* SYSCON_REDCY_NANDOR : RO ;bitpos:[31] ;default: 1'h0 ; */
/*description: .*/
#define SYSCON_REDCY_NANDOR    (BIT(31))
#define SYSCON_REDCY_NANDOR_M  (BIT(31))
#define SYSCON_REDCY_NANDOR_V  0x1
#define SYSCON_REDCY_NANDOR_S  31
/* SYSCON_REDCY_SIG1 : R/W ;bitpos:[30:0] ;default: 31'h0 ; */
/*description: .*/
#define SYSCON_REDCY_SIG1    0x7FFFFFFF
#define SYSCON_REDCY_SIG1_M  ((SYSCON_REDCY_SIG1_V)<<(SYSCON_REDCY_SIG1_S))
#define SYSCON_REDCY_SIG1_V  0x7FFFFFFF
#define SYSCON_REDCY_SIG1_S  0

#define SYSCON_FRONT_END_MEM_PD_REG          (DR_REG_SYSCON_BASE + 0x9C)
/* SYSCON_FREQ_MEM_FORCE_PD : R/W ;bitpos:[7] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_FREQ_MEM_FORCE_PD    (BIT(7))
#define SYSCON_FREQ_MEM_FORCE_PD_M  (BIT(7))
#define SYSCON_FREQ_MEM_FORCE_PD_V  0x1
#define SYSCON_FREQ_MEM_FORCE_PD_S  7
/* SYSCON_FREQ_MEM_FORCE_PU : R/W ;bitpos:[6] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_FREQ_MEM_FORCE_PU    (BIT(6))
#define SYSCON_FREQ_MEM_FORCE_PU_M  (BIT(6))
#define SYSCON_FREQ_MEM_FORCE_PU_V  0x1
#define SYSCON_FREQ_MEM_FORCE_PU_S  6
/* SYSCON_DC_MEM_FORCE_PD : R/W ;bitpos:[5] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_DC_MEM_FORCE_PD    (BIT(5))
#define SYSCON_DC_MEM_FORCE_PD_M  (BIT(5))
#define SYSCON_DC_MEM_FORCE_PD_V  0x1
#define SYSCON_DC_MEM_FORCE_PD_S  5
/* SYSCON_DC_MEM_FORCE_PU : R/W ;bitpos:[4] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_DC_MEM_FORCE_PU    (BIT(4))
#define SYSCON_DC_MEM_FORCE_PU_M  (BIT(4))
#define SYSCON_DC_MEM_FORCE_PU_V  0x1
#define SYSCON_DC_MEM_FORCE_PU_S  4
/* SYSCON_PBUS_MEM_FORCE_PD : R/W ;bitpos:[3] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_PBUS_MEM_FORCE_PD    (BIT(3))
#define SYSCON_PBUS_MEM_FORCE_PD_M  (BIT(3))
#define SYSCON_PBUS_MEM_FORCE_PD_V  0x1
#define SYSCON_PBUS_MEM_FORCE_PD_S  3
/* SYSCON_PBUS_MEM_FORCE_PU : R/W ;bitpos:[2] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_PBUS_MEM_FORCE_PU    (BIT(2))
#define SYSCON_PBUS_MEM_FORCE_PU_M  (BIT(2))
#define SYSCON_PBUS_MEM_FORCE_PU_V  0x1
#define SYSCON_PBUS_MEM_FORCE_PU_S  2
/* SYSCON_AGC_MEM_FORCE_PD : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_AGC_MEM_FORCE_PD    (BIT(1))
#define SYSCON_AGC_MEM_FORCE_PD_M  (BIT(1))
#define SYSCON_AGC_MEM_FORCE_PD_V  0x1
#define SYSCON_AGC_MEM_FORCE_PD_S  1
/* SYSCON_AGC_MEM_FORCE_PU : R/W ;bitpos:[0] ;default: 1'b1 ; */
/*description: .*/
#define SYSCON_AGC_MEM_FORCE_PU    (BIT(0))
#define SYSCON_AGC_MEM_FORCE_PU_M  (BIT(0))
#define SYSCON_AGC_MEM_FORCE_PU_V  0x1
#define SYSCON_AGC_MEM_FORCE_PU_S  0

#define SYSCON_SPI_MEM_ECC_CTRL_REG          (DR_REG_SYSCON_BASE + 0xA0)
/* SYSCON_SRAM_PAGE_SIZE : R/W ;bitpos:[21:20] ;default: 2'd2 ; */
/*description: Set the page size of the used MSPI external RAM. 0: 256 bytes. 1: 512 bytes. 2:
1024 bytes. 3: 2048 bytes..*/
#define SYSCON_SRAM_PAGE_SIZE    0x00000003
#define SYSCON_SRAM_PAGE_SIZE_M  ((SYSCON_SRAM_PAGE_SIZE_V)<<(SYSCON_SRAM_PAGE_SIZE_S))
#define SYSCON_SRAM_PAGE_SIZE_V  0x3
#define SYSCON_SRAM_PAGE_SIZE_S  20
/* SYSCON_FLASH_PAGE_SIZE : R/W ;bitpos:[19:18] ;default: 2'd0 ; */
/*description: Set the page size of the used MSPI flash. 0: 256 bytes. 1: 512 bytes. 2: 1024 by
tes. 3: 2048 bytes..*/
#define SYSCON_FLASH_PAGE_SIZE    0x00000003
#define SYSCON_FLASH_PAGE_SIZE_M  ((SYSCON_FLASH_PAGE_SIZE_V)<<(SYSCON_FLASH_PAGE_SIZE_S))
#define SYSCON_FLASH_PAGE_SIZE_V  0x3
#define SYSCON_FLASH_PAGE_SIZE_S  18

#define SYSCON_CLKGATE_FORCE_ON_REG          (DR_REG_SYSCON_BASE + 0xA8)
/* SYSCON_SRAM_CLKGATE_FORCE_ON : R/W ;bitpos:[13:3] ;default: ~11'b0 ; */
/*description: .*/
#define SYSCON_SRAM_CLKGATE_FORCE_ON    0x000007FF
#define SYSCON_SRAM_CLKGATE_FORCE_ON_M  ((SYSCON_SRAM_CLKGATE_FORCE_ON_V)<<(SYSCON_SRAM_CLKGATE_FORCE_ON_S))
#define SYSCON_SRAM_CLKGATE_FORCE_ON_V  0x7FF
#define SYSCON_SRAM_CLKGATE_FORCE_ON_S  3
/* SYSCON_ROM_CLKGATE_FORCE_ON : R/W ;bitpos:[2:0] ;default: ~3'b0 ; */
/*description: .*/
#define SYSCON_ROM_CLKGATE_FORCE_ON    0x00000007
#define SYSCON_ROM_CLKGATE_FORCE_ON_M  ((SYSCON_ROM_CLKGATE_FORCE_ON_V)<<(SYSCON_ROM_CLKGATE_FORCE_ON_S))
#define SYSCON_ROM_CLKGATE_FORCE_ON_V  0x7
#define SYSCON_ROM_CLKGATE_FORCE_ON_S  0

#define SYSCON_MEM_POWER_DOWN_REG          (DR_REG_SYSCON_BASE + 0xAC)
/* SYSCON_SRAM_POWER_DOWN : R/W ;bitpos:[13:3] ;default: 11'b0 ; */
/*description: .*/
#define SYSCON_SRAM_POWER_DOWN    0x000007FF
#define SYSCON_SRAM_POWER_DOWN_M  ((SYSCON_SRAM_POWER_DOWN_V)<<(SYSCON_SRAM_POWER_DOWN_S))
#define SYSCON_SRAM_POWER_DOWN_V  0x7FF
#define SYSCON_SRAM_POWER_DOWN_S  3
/* SYSCON_ROM_POWER_DOWN : R/W ;bitpos:[2:0] ;default: 3'b0 ; */
/*description: .*/
#define SYSCON_ROM_POWER_DOWN    0x00000007
#define SYSCON_ROM_POWER_DOWN_M  ((SYSCON_ROM_POWER_DOWN_V)<<(SYSCON_ROM_POWER_DOWN_S))
#define SYSCON_ROM_POWER_DOWN_V  0x7
#define SYSCON_ROM_POWER_DOWN_S  0

#define SYSCON_MEM_POWER_UP_REG          (DR_REG_SYSCON_BASE + 0xB0)
/* SYSCON_SRAM_POWER_UP : R/W ;bitpos:[13:3] ;default: ~11'b0 ; */
/*description: .*/
#define SYSCON_SRAM_POWER_UP    0x000007FF
#define SYSCON_SRAM_POWER_UP_M  ((SYSCON_SRAM_POWER_UP_V)<<(SYSCON_SRAM_POWER_UP_S))
#define SYSCON_SRAM_POWER_UP_V  0x7FF
#define SYSCON_SRAM_POWER_UP_S  3
/* SYSCON_ROM_POWER_UP : R/W ;bitpos:[2:0] ;default: ~3'b0 ; */
/*description: .*/
#define SYSCON_ROM_POWER_UP    0x00000007
#define SYSCON_ROM_POWER_UP_M  ((SYSCON_ROM_POWER_UP_V)<<(SYSCON_ROM_POWER_UP_S))
#define SYSCON_ROM_POWER_UP_V  0x7
#define SYSCON_ROM_POWER_UP_S  0

#define SYSCON_RETENTION_CTRL_REG          (DR_REG_SYSCON_BASE + 0xB4)
/* SYSCON_NOBYPASS_CPU_ISO_RST : R/W ;bitpos:[27] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_NOBYPASS_CPU_ISO_RST    (BIT(27))
#define SYSCON_NOBYPASS_CPU_ISO_RST_M  (BIT(27))
#define SYSCON_NOBYPASS_CPU_ISO_RST_V  0x1
#define SYSCON_NOBYPASS_CPU_ISO_RST_S  27
/* SYSCON_RETENTION_CPU_LINK_ADDR : R/W ;bitpos:[26:0] ;default: 27'd0 ; */
/*description: .*/
#define SYSCON_RETENTION_CPU_LINK_ADDR    0x07FFFFFF
#define SYSCON_RETENTION_CPU_LINK_ADDR_M  ((SYSCON_RETENTION_CPU_LINK_ADDR_V)<<(SYSCON_RETENTION_CPU_LINK_ADDR_S))
#define SYSCON_RETENTION_CPU_LINK_ADDR_V  0x7FFFFFF
#define SYSCON_RETENTION_CPU_LINK_ADDR_S  0

#define SYSCON_RETENTION_CTRL1_REG          (DR_REG_SYSCON_BASE + 0xB8)
/* SYSCON_RETENTION_TAG_LINK_ADDR : R/W ;bitpos:[26:0] ;default: 27'd0 ; */
/*description: .*/
#define SYSCON_RETENTION_TAG_LINK_ADDR    0x07FFFFFF
#define SYSCON_RETENTION_TAG_LINK_ADDR_M  ((SYSCON_RETENTION_TAG_LINK_ADDR_V)<<(SYSCON_RETENTION_TAG_LINK_ADDR_S))
#define SYSCON_RETENTION_TAG_LINK_ADDR_V  0x7FFFFFF
#define SYSCON_RETENTION_TAG_LINK_ADDR_S  0

#define SYSCON_RETENTION_CTRL2_REG          (DR_REG_SYSCON_BASE + 0xBC)
/* SYSCON_RET_ICACHE_ENABLE : R/W ;bitpos:[31] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_RET_ICACHE_ENABLE    (BIT(31))
#define SYSCON_RET_ICACHE_ENABLE_M  (BIT(31))
#define SYSCON_RET_ICACHE_ENABLE_V  0x1
#define SYSCON_RET_ICACHE_ENABLE_S  31
/* SYSCON_RET_ICACHE_START_POINT : R/W ;bitpos:[29:22] ;default: 8'd0 ; */
/*description: .*/
#define SYSCON_RET_ICACHE_START_POINT    0x000000FF
#define SYSCON_RET_ICACHE_START_POINT_M  ((SYSCON_RET_ICACHE_START_POINT_V)<<(SYSCON_RET_ICACHE_START_POINT_S))
#define SYSCON_RET_ICACHE_START_POINT_V  0xFF
#define SYSCON_RET_ICACHE_START_POINT_S  22
/* SYSCON_RET_ICACHE_VLD_SIZE : R/W ;bitpos:[20:13] ;default: 8'hff ; */
/*description: .*/
#define SYSCON_RET_ICACHE_VLD_SIZE    0x000000FF
#define SYSCON_RET_ICACHE_VLD_SIZE_M  ((SYSCON_RET_ICACHE_VLD_SIZE_V)<<(SYSCON_RET_ICACHE_VLD_SIZE_S))
#define SYSCON_RET_ICACHE_VLD_SIZE_V  0xFF
#define SYSCON_RET_ICACHE_VLD_SIZE_S  13
/* SYSCON_RET_ICACHE_SIZE : R/W ;bitpos:[11:4] ;default: 8'hff ; */
/*description: .*/
#define SYSCON_RET_ICACHE_SIZE    0x000000FF
#define SYSCON_RET_ICACHE_SIZE_M  ((SYSCON_RET_ICACHE_SIZE_V)<<(SYSCON_RET_ICACHE_SIZE_S))
#define SYSCON_RET_ICACHE_SIZE_V  0xFF
#define SYSCON_RET_ICACHE_SIZE_S  4

#define SYSCON_RETENTION_CTRL3_REG          (DR_REG_SYSCON_BASE + 0xC0)
/* SYSCON_RET_DCACHE_ENABLE : R/W ;bitpos:[31] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_RET_DCACHE_ENABLE    (BIT(31))
#define SYSCON_RET_DCACHE_ENABLE_M  (BIT(31))
#define SYSCON_RET_DCACHE_ENABLE_V  0x1
#define SYSCON_RET_DCACHE_ENABLE_S  31
/* SYSCON_RET_DCACHE_START_POINT : R/W ;bitpos:[30:22] ;default: 9'd0 ; */
/*description: .*/
#define SYSCON_RET_DCACHE_START_POINT    0x000001FF
#define SYSCON_RET_DCACHE_START_POINT_M  ((SYSCON_RET_DCACHE_START_POINT_V)<<(SYSCON_RET_DCACHE_START_POINT_S))
#define SYSCON_RET_DCACHE_START_POINT_V  0x1FF
#define SYSCON_RET_DCACHE_START_POINT_S  22
/* SYSCON_RET_DCACHE_VLD_SIZE : R/W ;bitpos:[21:13] ;default: 9'h1ff ; */
/*description: .*/
#define SYSCON_RET_DCACHE_VLD_SIZE    0x000001FF
#define SYSCON_RET_DCACHE_VLD_SIZE_M  ((SYSCON_RET_DCACHE_VLD_SIZE_V)<<(SYSCON_RET_DCACHE_VLD_SIZE_S))
#define SYSCON_RET_DCACHE_VLD_SIZE_V  0x1FF
#define SYSCON_RET_DCACHE_VLD_SIZE_S  13
/* SYSCON_RET_DCACHE_SIZE : R/W ;bitpos:[12:4] ;default: 9'h1ff ; */
/*description: .*/
#define SYSCON_RET_DCACHE_SIZE    0x000001FF
#define SYSCON_RET_DCACHE_SIZE_M  ((SYSCON_RET_DCACHE_SIZE_V)<<(SYSCON_RET_DCACHE_SIZE_S))
#define SYSCON_RET_DCACHE_SIZE_V  0x1FF
#define SYSCON_RET_DCACHE_SIZE_S  4

#define SYSCON_RETENTION_CTRL4_REG          (DR_REG_SYSCON_BASE + 0xC4)
/* SYSCON_RETENTION_INV_CFG : R/W ;bitpos:[31:0] ;default: ~32'h0 ; */
/*description: .*/
#define SYSCON_RETENTION_INV_CFG    0xFFFFFFFF
#define SYSCON_RETENTION_INV_CFG_M  ((SYSCON_RETENTION_INV_CFG_V)<<(SYSCON_RETENTION_INV_CFG_S))
#define SYSCON_RETENTION_INV_CFG_V  0xFFFFFFFF
#define SYSCON_RETENTION_INV_CFG_S  0

#define SYSCON_RETENTION_CTRL5_REG          (DR_REG_SYSCON_BASE + 0xC8)
/* SYSCON_RETENTION_DISABLE : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: .*/
#define SYSCON_RETENTION_DISABLE    (BIT(0))
#define SYSCON_RETENTION_DISABLE_M  (BIT(0))
#define SYSCON_RETENTION_DISABLE_V  0x1
#define SYSCON_RETENTION_DISABLE_S  0

#define SYSCON_DATE_REG          (DR_REG_SYSCON_BASE + 0x3FC)
/* SYSCON_DATE : R/W ;bitpos:[31:0] ;default: 32'h2101150 ; */
/*description: Version control.*/
#define SYSCON_DATE    0xFFFFFFFF
#define SYSCON_DATE_M  ((SYSCON_DATE_V)<<(SYSCON_DATE_S))
#define SYSCON_DATE_V  0xFFFFFFFF
#define SYSCON_DATE_S  0


#ifdef __cplusplus
}
#endif



#endif /*_SOC_SYSCON_REG_H_ */
