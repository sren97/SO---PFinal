/**
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 *  SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "soc/soc.h"
#include "soc/efuse_defs.h"
#ifdef __cplusplus
extern "C" {
#endif

/** EFUSE_BLK0_RDATA0_REG register */
#define EFUSE_BLK0_RDATA0_REG (DR_REG_EFUSE_BASE + 0x0)
/** EFUSE_RD_EFUSE_WR_DIS : R; bitpos: [15:0]; default: 0;
 *  read for efuse_wr_disable
 */
#define EFUSE_RD_EFUSE_WR_DIS    0x0000FFFFU
#define EFUSE_RD_EFUSE_WR_DIS_M  (EFUSE_RD_EFUSE_WR_DIS_V << EFUSE_RD_EFUSE_WR_DIS_S)
#define EFUSE_RD_EFUSE_WR_DIS_V  0x0000FFFFU
#define EFUSE_RD_EFUSE_WR_DIS_S  0
/** EFUSE_RD_EFUSE_RD_DIS : R; bitpos: [19:16]; default: 0;
 *  read for efuse_rd_disable
 */
#define EFUSE_RD_EFUSE_RD_DIS    0x0000000FU
#define EFUSE_RD_EFUSE_RD_DIS_M  (EFUSE_RD_EFUSE_RD_DIS_V << EFUSE_RD_EFUSE_RD_DIS_S)
#define EFUSE_RD_EFUSE_RD_DIS_V  0x0000000FU
#define EFUSE_RD_EFUSE_RD_DIS_S  16
/** EFUSE_RD_FLASH_CRYPT_CNT : R; bitpos: [26:20]; default: 0;
 *  read for flash_crypt_cnt
 */
#define EFUSE_RD_FLASH_CRYPT_CNT    0x0000007FU
#define EFUSE_RD_FLASH_CRYPT_CNT_M  (EFUSE_RD_FLASH_CRYPT_CNT_V << EFUSE_RD_FLASH_CRYPT_CNT_S)
#define EFUSE_RD_FLASH_CRYPT_CNT_V  0x0000007FU
#define EFUSE_RD_FLASH_CRYPT_CNT_S  20
/** EFUSE_RD_UART_DOWNLOAD_DIS : R; bitpos: [27]; default: 0;
 *  Disable UART download mode. Valid for ESP32 V3 and newer, only
 */
#define EFUSE_RD_UART_DOWNLOAD_DIS    (BIT(27))
#define EFUSE_RD_UART_DOWNLOAD_DIS_M  (EFUSE_RD_UART_DOWNLOAD_DIS_V << EFUSE_RD_UART_DOWNLOAD_DIS_S)
#define EFUSE_RD_UART_DOWNLOAD_DIS_V  0x00000001U
#define EFUSE_RD_UART_DOWNLOAD_DIS_S  27
/** EFUSE_RESERVED_0_28 : R; bitpos: [31:28]; default: 0;
 *  reserved
 */
#define EFUSE_RESERVED_0_28    0x0000000FU
#define EFUSE_RESERVED_0_28_M  (EFUSE_RESERVED_0_28_V << EFUSE_RESERVED_0_28_S)
#define EFUSE_RESERVED_0_28_V  0x0000000FU
#define EFUSE_RESERVED_0_28_S  28

/** EFUSE_BLK0_RDATA1_REG register */
#define EFUSE_BLK0_RDATA1_REG (DR_REG_EFUSE_BASE + 0x4)
/** EFUSE_RD_MAC : R; bitpos: [31:0]; default: 0;
 *  MAC address
 */
#define EFUSE_RD_MAC    0xFFFFFFFFU
#define EFUSE_RD_MAC_M  (EFUSE_RD_MAC_V << EFUSE_RD_MAC_S)
#define EFUSE_RD_MAC_V  0xFFFFFFFFU
#define EFUSE_RD_MAC_S  0

/** EFUSE_BLK0_RDATA2_REG register */
#define EFUSE_BLK0_RDATA2_REG (DR_REG_EFUSE_BASE + 0x8)
/** EFUSE_RD_MAC_1 : R; bitpos: [15:0]; default: 0;
 *  MAC address
 */
#define EFUSE_RD_MAC_1    0x0000FFFFU
#define EFUSE_RD_MAC_1_M  (EFUSE_RD_MAC_1_V << EFUSE_RD_MAC_1_S)
#define EFUSE_RD_MAC_1_V  0x0000FFFFU
#define EFUSE_RD_MAC_1_S  0
/** EFUSE_RD_MAC_CRC : R; bitpos: [23:16]; default: 0;
 *  CRC8 for MAC address
 */
#define EFUSE_RD_MAC_CRC    0x000000FFU
#define EFUSE_RD_MAC_CRC_M  (EFUSE_RD_MAC_CRC_V << EFUSE_RD_MAC_CRC_S)
#define EFUSE_RD_MAC_CRC_V  0x000000FFU
#define EFUSE_RD_MAC_CRC_S  16
/** EFUSE_RD_RESERVE_0_88 : RW; bitpos: [31:24]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RD_RESERVE_0_88    0x000000FFU
#define EFUSE_RD_RESERVE_0_88_M  (EFUSE_RD_RESERVE_0_88_V << EFUSE_RD_RESERVE_0_88_S)
#define EFUSE_RD_RESERVE_0_88_V  0x000000FFU
#define EFUSE_RD_RESERVE_0_88_S  24

/** EFUSE_BLK0_RDATA3_REG register */
#define EFUSE_BLK0_RDATA3_REG (DR_REG_EFUSE_BASE + 0xc)
/** EFUSE_RD_DISABLE_APP_CPU : R; bitpos: [0]; default: 0;
 *  Disables APP CPU
 */
#define EFUSE_RD_DISABLE_APP_CPU    (BIT(0))
#define EFUSE_RD_DISABLE_APP_CPU_M  (EFUSE_RD_DISABLE_APP_CPU_V << EFUSE_RD_DISABLE_APP_CPU_S)
#define EFUSE_RD_DISABLE_APP_CPU_V  0x00000001U
#define EFUSE_RD_DISABLE_APP_CPU_S  0
/** EFUSE_RD_DISABLE_BT : R; bitpos: [1]; default: 0;
 *  Disables Bluetooth
 */
#define EFUSE_RD_DISABLE_BT    (BIT(1))
#define EFUSE_RD_DISABLE_BT_M  (EFUSE_RD_DISABLE_BT_V << EFUSE_RD_DISABLE_BT_S)
#define EFUSE_RD_DISABLE_BT_V  0x00000001U
#define EFUSE_RD_DISABLE_BT_S  1
/** EFUSE_RD_CHIP_PACKAGE_4BIT : R; bitpos: [2]; default: 0;
 *  Chip package identifier #4bit
 */
#define EFUSE_RD_CHIP_PACKAGE_4BIT    (BIT(2))
#define EFUSE_RD_CHIP_PACKAGE_4BIT_M  (EFUSE_RD_CHIP_PACKAGE_4BIT_V << EFUSE_RD_CHIP_PACKAGE_4BIT_S)
#define EFUSE_RD_CHIP_PACKAGE_4BIT_V  0x00000001U
#define EFUSE_RD_CHIP_PACKAGE_4BIT_S  2
/** EFUSE_RD_DIS_CACHE : R; bitpos: [3]; default: 0;
 *  Disables cache
 */
#define EFUSE_RD_DIS_CACHE    (BIT(3))
#define EFUSE_RD_DIS_CACHE_M  (EFUSE_RD_DIS_CACHE_V << EFUSE_RD_DIS_CACHE_S)
#define EFUSE_RD_DIS_CACHE_V  0x00000001U
#define EFUSE_RD_DIS_CACHE_S  3
/** EFUSE_RD_SPI_PAD_CONFIG_HD : R; bitpos: [8:4]; default: 0;
 *  read for SPI_pad_config_hd
 */
#define EFUSE_RD_SPI_PAD_CONFIG_HD    0x0000001FU
#define EFUSE_RD_SPI_PAD_CONFIG_HD_M  (EFUSE_RD_SPI_PAD_CONFIG_HD_V << EFUSE_RD_SPI_PAD_CONFIG_HD_S)
#define EFUSE_RD_SPI_PAD_CONFIG_HD_V  0x0000001FU
#define EFUSE_RD_SPI_PAD_CONFIG_HD_S  4
/** EFUSE_RD_CHIP_PACKAGE : RW; bitpos: [11:9]; default: 0;
 *  Chip package identifier
 */
#define EFUSE_RD_CHIP_PACKAGE    0x00000007U
#define EFUSE_RD_CHIP_PACKAGE_M  (EFUSE_RD_CHIP_PACKAGE_V << EFUSE_RD_CHIP_PACKAGE_S)
#define EFUSE_RD_CHIP_PACKAGE_V  0x00000007U
#define EFUSE_RD_CHIP_PACKAGE_S  9
/** EFUSE_RD_CHIP_CPU_FREQ_LOW : RW; bitpos: [12]; default: 0;
 *  If set alongside EFUSE_RD_CHIP_CPU_FREQ_RATED, the ESP32's max CPU frequency is
 *  rated for 160MHz. 240MHz otherwise
 */
#define EFUSE_RD_CHIP_CPU_FREQ_LOW    (BIT(12))
#define EFUSE_RD_CHIP_CPU_FREQ_LOW_M  (EFUSE_RD_CHIP_CPU_FREQ_LOW_V << EFUSE_RD_CHIP_CPU_FREQ_LOW_S)
#define EFUSE_RD_CHIP_CPU_FREQ_LOW_V  0x00000001U
#define EFUSE_RD_CHIP_CPU_FREQ_LOW_S  12
/** EFUSE_RD_CHIP_CPU_FREQ_RATED : RW; bitpos: [13]; default: 0;
 *  If set, the ESP32's maximum CPU frequency has been rated
 */
#define EFUSE_RD_CHIP_CPU_FREQ_RATED    (BIT(13))
#define EFUSE_RD_CHIP_CPU_FREQ_RATED_M  (EFUSE_RD_CHIP_CPU_FREQ_RATED_V << EFUSE_RD_CHIP_CPU_FREQ_RATED_S)
#define EFUSE_RD_CHIP_CPU_FREQ_RATED_V  0x00000001U
#define EFUSE_RD_CHIP_CPU_FREQ_RATED_S  13
/** EFUSE_RD_BLK3_PART_RESERVE : RW; bitpos: [14]; default: 0;
 *  If set, this bit indicates that BLOCK3[143:96] is reserved for internal use
 */
#define EFUSE_RD_BLK3_PART_RESERVE    (BIT(14))
#define EFUSE_RD_BLK3_PART_RESERVE_M  (EFUSE_RD_BLK3_PART_RESERVE_V << EFUSE_RD_BLK3_PART_RESERVE_S)
#define EFUSE_RD_BLK3_PART_RESERVE_V  0x00000001U
#define EFUSE_RD_BLK3_PART_RESERVE_S  14
/** EFUSE_RD_CHIP_VER_REV1 : RW; bitpos: [15]; default: 0;
 *  bit is set to 1 for rev1 silicon
 */
#define EFUSE_RD_CHIP_VER_REV1    (BIT(15))
#define EFUSE_RD_CHIP_VER_REV1_M  (EFUSE_RD_CHIP_VER_REV1_V << EFUSE_RD_CHIP_VER_REV1_S)
#define EFUSE_RD_CHIP_VER_REV1_V  0x00000001U
#define EFUSE_RD_CHIP_VER_REV1_S  15
/** EFUSE_RD_RESERVE_0_112 : RW; bitpos: [31:16]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RD_RESERVE_0_112    0x0000FFFFU
#define EFUSE_RD_RESERVE_0_112_M  (EFUSE_RD_RESERVE_0_112_V << EFUSE_RD_RESERVE_0_112_S)
#define EFUSE_RD_RESERVE_0_112_V  0x0000FFFFU
#define EFUSE_RD_RESERVE_0_112_S  16

/** EFUSE_BLK0_RDATA4_REG register */
#define EFUSE_BLK0_RDATA4_REG (DR_REG_EFUSE_BASE + 0x10)
/** EFUSE_RD_CLK8M_FREQ : R; bitpos: [7:0]; default: 0;
 *  8MHz clock freq override
 */
#define EFUSE_RD_CLK8M_FREQ    0x000000FFU
#define EFUSE_RD_CLK8M_FREQ_M  (EFUSE_RD_CLK8M_FREQ_V << EFUSE_RD_CLK8M_FREQ_S)
#define EFUSE_RD_CLK8M_FREQ_V  0x000000FFU
#define EFUSE_RD_CLK8M_FREQ_S  0
/** EFUSE_RD_ADC_VREF : RW; bitpos: [12:8]; default: 0;
 *  True ADC reference voltage
 */
#define EFUSE_RD_ADC_VREF    0x0000001FU
#define EFUSE_RD_ADC_VREF_M  (EFUSE_RD_ADC_VREF_V << EFUSE_RD_ADC_VREF_S)
#define EFUSE_RD_ADC_VREF_V  0x0000001FU
#define EFUSE_RD_ADC_VREF_S  8
/** EFUSE_RD_RESERVE_0_141 : RW; bitpos: [13]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RD_RESERVE_0_141    (BIT(13))
#define EFUSE_RD_RESERVE_0_141_M  (EFUSE_RD_RESERVE_0_141_V << EFUSE_RD_RESERVE_0_141_S)
#define EFUSE_RD_RESERVE_0_141_V  0x00000001U
#define EFUSE_RD_RESERVE_0_141_S  13
/** EFUSE_RD_XPD_SDIO_REG : R; bitpos: [14]; default: 0;
 *  read for XPD_SDIO_REG
 */
#define EFUSE_RD_XPD_SDIO_REG    (BIT(14))
#define EFUSE_RD_XPD_SDIO_REG_M  (EFUSE_RD_XPD_SDIO_REG_V << EFUSE_RD_XPD_SDIO_REG_S)
#define EFUSE_RD_XPD_SDIO_REG_V  0x00000001U
#define EFUSE_RD_XPD_SDIO_REG_S  14
/** EFUSE_RD_XPD_SDIO_TIEH : R; bitpos: [15]; default: 0;
 *  If XPD_SDIO_FORCE & XPD_SDIO_REG
 */
#define EFUSE_RD_XPD_SDIO_TIEH    (BIT(15))
#define EFUSE_RD_XPD_SDIO_TIEH_M  (EFUSE_RD_XPD_SDIO_TIEH_V << EFUSE_RD_XPD_SDIO_TIEH_S)
#define EFUSE_RD_XPD_SDIO_TIEH_V  0x00000001U
#define EFUSE_RD_XPD_SDIO_TIEH_S  15
/** EFUSE_RD_XPD_SDIO_FORCE : R; bitpos: [16]; default: 0;
 *  Ignore MTDI pin (GPIO12) for VDD_SDIO on reset
 */
#define EFUSE_RD_XPD_SDIO_FORCE    (BIT(16))
#define EFUSE_RD_XPD_SDIO_FORCE_M  (EFUSE_RD_XPD_SDIO_FORCE_V << EFUSE_RD_XPD_SDIO_FORCE_S)
#define EFUSE_RD_XPD_SDIO_FORCE_V  0x00000001U
#define EFUSE_RD_XPD_SDIO_FORCE_S  16
/** EFUSE_RD_RESERVE_0_145 : RW; bitpos: [31:17]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RD_RESERVE_0_145    0x00007FFFU
#define EFUSE_RD_RESERVE_0_145_M  (EFUSE_RD_RESERVE_0_145_V << EFUSE_RD_RESERVE_0_145_S)
#define EFUSE_RD_RESERVE_0_145_V  0x00007FFFU
#define EFUSE_RD_RESERVE_0_145_S  17

/** EFUSE_BLK0_RDATA5_REG register */
#define EFUSE_BLK0_RDATA5_REG (DR_REG_EFUSE_BASE + 0x14)
/** EFUSE_RD_SPI_PAD_CONFIG_CLK : R; bitpos: [4:0]; default: 0;
 *  read for SPI_pad_config_clk
 */
#define EFUSE_RD_SPI_PAD_CONFIG_CLK    0x0000001FU
#define EFUSE_RD_SPI_PAD_CONFIG_CLK_M  (EFUSE_RD_SPI_PAD_CONFIG_CLK_V << EFUSE_RD_SPI_PAD_CONFIG_CLK_S)
#define EFUSE_RD_SPI_PAD_CONFIG_CLK_V  0x0000001FU
#define EFUSE_RD_SPI_PAD_CONFIG_CLK_S  0
/** EFUSE_RD_SPI_PAD_CONFIG_Q : R; bitpos: [9:5]; default: 0;
 *  read for SPI_pad_config_q
 */
#define EFUSE_RD_SPI_PAD_CONFIG_Q    0x0000001FU
#define EFUSE_RD_SPI_PAD_CONFIG_Q_M  (EFUSE_RD_SPI_PAD_CONFIG_Q_V << EFUSE_RD_SPI_PAD_CONFIG_Q_S)
#define EFUSE_RD_SPI_PAD_CONFIG_Q_V  0x0000001FU
#define EFUSE_RD_SPI_PAD_CONFIG_Q_S  5
/** EFUSE_RD_SPI_PAD_CONFIG_D : R; bitpos: [14:10]; default: 0;
 *  read for SPI_pad_config_d
 */
#define EFUSE_RD_SPI_PAD_CONFIG_D    0x0000001FU
#define EFUSE_RD_SPI_PAD_CONFIG_D_M  (EFUSE_RD_SPI_PAD_CONFIG_D_V << EFUSE_RD_SPI_PAD_CONFIG_D_S)
#define EFUSE_RD_SPI_PAD_CONFIG_D_V  0x0000001FU
#define EFUSE_RD_SPI_PAD_CONFIG_D_S  10
/** EFUSE_RD_SPI_PAD_CONFIG_CS0 : R; bitpos: [19:15]; default: 0;
 *  read for SPI_pad_config_cs0
 */
#define EFUSE_RD_SPI_PAD_CONFIG_CS0    0x0000001FU
#define EFUSE_RD_SPI_PAD_CONFIG_CS0_M  (EFUSE_RD_SPI_PAD_CONFIG_CS0_V << EFUSE_RD_SPI_PAD_CONFIG_CS0_S)
#define EFUSE_RD_SPI_PAD_CONFIG_CS0_V  0x0000001FU
#define EFUSE_RD_SPI_PAD_CONFIG_CS0_S  15
/** EFUSE_RD_CHIP_VER_REV2 : R; bitpos: [20]; default: 0; */
#define EFUSE_RD_CHIP_VER_REV2    (BIT(20))
#define EFUSE_RD_CHIP_VER_REV2_M  (EFUSE_RD_CHIP_VER_REV2_V << EFUSE_RD_CHIP_VER_REV2_S)
#define EFUSE_RD_CHIP_VER_REV2_V  0x00000001U
#define EFUSE_RD_CHIP_VER_REV2_S  20
/** EFUSE_RD_RESERVE_0_181 : RW; bitpos: [21]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RD_RESERVE_0_181    (BIT(21))
#define EFUSE_RD_RESERVE_0_181_M  (EFUSE_RD_RESERVE_0_181_V << EFUSE_RD_RESERVE_0_181_S)
#define EFUSE_RD_RESERVE_0_181_V  0x00000001U
#define EFUSE_RD_RESERVE_0_181_S  21
/** EFUSE_RD_VOL_LEVEL_HP_INV : R; bitpos: [23:22]; default: 0;
 *  This field stores the voltage level for CPU to run at 240 MHz, or for flash/PSRAM
 *  to run at 80 MHz.0x0: level 7; 0x1: level 6; 0x2: level 5; 0x3: level 4. (RO)
 */
#define EFUSE_RD_VOL_LEVEL_HP_INV    0x00000003U
#define EFUSE_RD_VOL_LEVEL_HP_INV_M  (EFUSE_RD_VOL_LEVEL_HP_INV_V << EFUSE_RD_VOL_LEVEL_HP_INV_S)
#define EFUSE_RD_VOL_LEVEL_HP_INV_V  0x00000003U
#define EFUSE_RD_VOL_LEVEL_HP_INV_S  22
/** EFUSE_RD_WAFER_VERSION_MINOR : R; bitpos: [25:24]; default: 0; */
#define EFUSE_RD_WAFER_VERSION_MINOR    0x00000003U
#define EFUSE_RD_WAFER_VERSION_MINOR_M  (EFUSE_RD_WAFER_VERSION_MINOR_V << EFUSE_RD_WAFER_VERSION_MINOR_S)
#define EFUSE_RD_WAFER_VERSION_MINOR_V  0x00000003U
#define EFUSE_RD_WAFER_VERSION_MINOR_S  24
/** EFUSE_RD_RESERVE_0_186 : RW; bitpos: [27:26]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RD_RESERVE_0_186    0x00000003U
#define EFUSE_RD_RESERVE_0_186_M  (EFUSE_RD_RESERVE_0_186_V << EFUSE_RD_RESERVE_0_186_S)
#define EFUSE_RD_RESERVE_0_186_V  0x00000003U
#define EFUSE_RD_RESERVE_0_186_S  26
/** EFUSE_RD_FLASH_CRYPT_CONFIG : R; bitpos: [31:28]; default: 0;
 *  read for flash_crypt_config
 */
#define EFUSE_RD_FLASH_CRYPT_CONFIG    0x0000000FU
#define EFUSE_RD_FLASH_CRYPT_CONFIG_M  (EFUSE_RD_FLASH_CRYPT_CONFIG_V << EFUSE_RD_FLASH_CRYPT_CONFIG_S)
#define EFUSE_RD_FLASH_CRYPT_CONFIG_V  0x0000000FU
#define EFUSE_RD_FLASH_CRYPT_CONFIG_S  28

/** EFUSE_BLK0_RDATA6_REG register */
#define EFUSE_BLK0_RDATA6_REG (DR_REG_EFUSE_BASE + 0x18)
/** EFUSE_RD_CODING_SCHEME : R; bitpos: [1:0]; default: 0;
 *  read for coding_scheme
 */
#define EFUSE_RD_CODING_SCHEME    0x00000003U
#define EFUSE_RD_CODING_SCHEME_M  (EFUSE_RD_CODING_SCHEME_V << EFUSE_RD_CODING_SCHEME_S)
#define EFUSE_RD_CODING_SCHEME_V  0x00000003U
#define EFUSE_RD_CODING_SCHEME_S  0
/** EFUSE_RD_CONSOLE_DEBUG_DISABLE : R; bitpos: [2]; default: 0;
 *  read for console_debug_disable
 */
#define EFUSE_RD_CONSOLE_DEBUG_DISABLE    (BIT(2))
#define EFUSE_RD_CONSOLE_DEBUG_DISABLE_M  (EFUSE_RD_CONSOLE_DEBUG_DISABLE_V << EFUSE_RD_CONSOLE_DEBUG_DISABLE_S)
#define EFUSE_RD_CONSOLE_DEBUG_DISABLE_V  0x00000001U
#define EFUSE_RD_CONSOLE_DEBUG_DISABLE_S  2
/** EFUSE_RD_DISABLE_SDIO_HOST : R; bitpos: [3]; default: 0; */
#define EFUSE_RD_DISABLE_SDIO_HOST    (BIT(3))
#define EFUSE_RD_DISABLE_SDIO_HOST_M  (EFUSE_RD_DISABLE_SDIO_HOST_V << EFUSE_RD_DISABLE_SDIO_HOST_S)
#define EFUSE_RD_DISABLE_SDIO_HOST_V  0x00000001U
#define EFUSE_RD_DISABLE_SDIO_HOST_S  3
/** EFUSE_RD_ABS_DONE_0 : R; bitpos: [4]; default: 0;
 *  read for abstract_done_0
 */
#define EFUSE_RD_ABS_DONE_0    (BIT(4))
#define EFUSE_RD_ABS_DONE_0_M  (EFUSE_RD_ABS_DONE_0_V << EFUSE_RD_ABS_DONE_0_S)
#define EFUSE_RD_ABS_DONE_0_V  0x00000001U
#define EFUSE_RD_ABS_DONE_0_S  4
/** EFUSE_RD_ABS_DONE_1 : R; bitpos: [5]; default: 0;
 *  read for abstract_done_1
 */
#define EFUSE_RD_ABS_DONE_1    (BIT(5))
#define EFUSE_RD_ABS_DONE_1_M  (EFUSE_RD_ABS_DONE_1_V << EFUSE_RD_ABS_DONE_1_S)
#define EFUSE_RD_ABS_DONE_1_V  0x00000001U
#define EFUSE_RD_ABS_DONE_1_S  5
/** EFUSE_RD_JTAG_DISABLE : R; bitpos: [6]; default: 0;
 *  Disable JTAG
 */
#define EFUSE_RD_JTAG_DISABLE    (BIT(6))
#define EFUSE_RD_JTAG_DISABLE_M  (EFUSE_RD_JTAG_DISABLE_V << EFUSE_RD_JTAG_DISABLE_S)
#define EFUSE_RD_JTAG_DISABLE_V  0x00000001U
#define EFUSE_RD_JTAG_DISABLE_S  6
/** EFUSE_RD_DISABLE_DL_ENCRYPT : R; bitpos: [7]; default: 0;
 *  read for download_dis_encrypt
 */
#define EFUSE_RD_DISABLE_DL_ENCRYPT    (BIT(7))
#define EFUSE_RD_DISABLE_DL_ENCRYPT_M  (EFUSE_RD_DISABLE_DL_ENCRYPT_V << EFUSE_RD_DISABLE_DL_ENCRYPT_S)
#define EFUSE_RD_DISABLE_DL_ENCRYPT_V  0x00000001U
#define EFUSE_RD_DISABLE_DL_ENCRYPT_S  7
/** EFUSE_RD_DISABLE_DL_DECRYPT : R; bitpos: [8]; default: 0;
 *  read for download_dis_decrypt
 */
#define EFUSE_RD_DISABLE_DL_DECRYPT    (BIT(8))
#define EFUSE_RD_DISABLE_DL_DECRYPT_M  (EFUSE_RD_DISABLE_DL_DECRYPT_V << EFUSE_RD_DISABLE_DL_DECRYPT_S)
#define EFUSE_RD_DISABLE_DL_DECRYPT_V  0x00000001U
#define EFUSE_RD_DISABLE_DL_DECRYPT_S  8
/** EFUSE_RD_DISABLE_DL_CACHE : R; bitpos: [9]; default: 0;
 *  read for download_dis_cache
 */
#define EFUSE_RD_DISABLE_DL_CACHE    (BIT(9))
#define EFUSE_RD_DISABLE_DL_CACHE_M  (EFUSE_RD_DISABLE_DL_CACHE_V << EFUSE_RD_DISABLE_DL_CACHE_S)
#define EFUSE_RD_DISABLE_DL_CACHE_V  0x00000001U
#define EFUSE_RD_DISABLE_DL_CACHE_S  9
/** EFUSE_RD_KEY_STATUS : R; bitpos: [10]; default: 0;
 *  read for key_status
 */
#define EFUSE_RD_KEY_STATUS    (BIT(10))
#define EFUSE_RD_KEY_STATUS_M  (EFUSE_RD_KEY_STATUS_V << EFUSE_RD_KEY_STATUS_S)
#define EFUSE_RD_KEY_STATUS_V  0x00000001U
#define EFUSE_RD_KEY_STATUS_S  10
/** EFUSE_RD_RESERVE_0_203 : RW; bitpos: [31:11]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RD_RESERVE_0_203    0x001FFFFFU
#define EFUSE_RD_RESERVE_0_203_M  (EFUSE_RD_RESERVE_0_203_V << EFUSE_RD_RESERVE_0_203_S)
#define EFUSE_RD_RESERVE_0_203_V  0x001FFFFFU
#define EFUSE_RD_RESERVE_0_203_S  11

/** EFUSE_BLK0_WDATA0_REG register */
#define EFUSE_BLK0_WDATA0_REG (DR_REG_EFUSE_BASE + 0x1c)
/** EFUSE_WR_DIS : RW; bitpos: [15:0]; default: 0;
 *  program for efuse_wr_disable
 */
#define EFUSE_WR_DIS    0x0000FFFFU
#define EFUSE_WR_DIS_M  (EFUSE_WR_DIS_V << EFUSE_WR_DIS_S)
#define EFUSE_WR_DIS_V  0x0000FFFFU
#define EFUSE_WR_DIS_S  0
/** EFUSE_RD_DIS : RW; bitpos: [19:16]; default: 0;
 *  program for efuse_rd_disable
 */
#define EFUSE_RD_DIS    0x0000000FU
#define EFUSE_RD_DIS_M  (EFUSE_RD_DIS_V << EFUSE_RD_DIS_S)
#define EFUSE_RD_DIS_V  0x0000000FU
#define EFUSE_RD_DIS_S  16
/** EFUSE_FLASH_CRYPT_CNT : RW; bitpos: [26:20]; default: 0;
 *  program for flash_crypt_cnt
 */
#define EFUSE_FLASH_CRYPT_CNT    0x0000007FU
#define EFUSE_FLASH_CRYPT_CNT_M  (EFUSE_FLASH_CRYPT_CNT_V << EFUSE_FLASH_CRYPT_CNT_S)
#define EFUSE_FLASH_CRYPT_CNT_V  0x0000007FU
#define EFUSE_FLASH_CRYPT_CNT_S  20

/** EFUSE_BLK0_WDATA1_REG register */
#define EFUSE_BLK0_WDATA1_REG (DR_REG_EFUSE_BASE + 0x20)
/** EFUSE_WIFI_MAC_CRC_LOW : RW; bitpos: [31:0]; default: 0;
 *  program for low 32bit WIFI_MAC_Address
 */
#define EFUSE_WIFI_MAC_CRC_LOW    0xFFFFFFFFU
#define EFUSE_WIFI_MAC_CRC_LOW_M  (EFUSE_WIFI_MAC_CRC_LOW_V << EFUSE_WIFI_MAC_CRC_LOW_S)
#define EFUSE_WIFI_MAC_CRC_LOW_V  0xFFFFFFFFU
#define EFUSE_WIFI_MAC_CRC_LOW_S  0

/** EFUSE_BLK0_WDATA2_REG register */
#define EFUSE_BLK0_WDATA2_REG (DR_REG_EFUSE_BASE + 0x24)
/** EFUSE_WIFI_MAC_CRC_HIGH : RW; bitpos: [23:0]; default: 0;
 *  program for high 24bit WIFI_MAC_Address
 */
#define EFUSE_WIFI_MAC_CRC_HIGH    0x00FFFFFFU
#define EFUSE_WIFI_MAC_CRC_HIGH_M  (EFUSE_WIFI_MAC_CRC_HIGH_V << EFUSE_WIFI_MAC_CRC_HIGH_S)
#define EFUSE_WIFI_MAC_CRC_HIGH_V  0x00FFFFFFU
#define EFUSE_WIFI_MAC_CRC_HIGH_S  0

/** EFUSE_BLK0_WDATA3_REG register */
#define EFUSE_BLK0_WDATA3_REG (DR_REG_EFUSE_BASE + 0x28)
/** EFUSE_DISABLE_APP_CPU : R; bitpos: [0]; default: 0;
 *  Disables APP CPU
 */
#define EFUSE_DISABLE_APP_CPU    (BIT(0))
#define EFUSE_DISABLE_APP_CPU_M  (EFUSE_DISABLE_APP_CPU_V << EFUSE_DISABLE_APP_CPU_S)
#define EFUSE_DISABLE_APP_CPU_V  0x00000001U
#define EFUSE_DISABLE_APP_CPU_S  0
/** EFUSE_DISABLE_BT : R; bitpos: [1]; default: 0;
 *  Disables Bluetooth
 */
#define EFUSE_DISABLE_BT    (BIT(1))
#define EFUSE_DISABLE_BT_M  (EFUSE_DISABLE_BT_V << EFUSE_DISABLE_BT_S)
#define EFUSE_DISABLE_BT_V  0x00000001U
#define EFUSE_DISABLE_BT_S  1
/** EFUSE_CHIP_PACKAGE_4BIT : R; bitpos: [2]; default: 0;
 *  Chip package identifier #4bit
 */
#define EFUSE_CHIP_PACKAGE_4BIT    (BIT(2))
#define EFUSE_CHIP_PACKAGE_4BIT_M  (EFUSE_CHIP_PACKAGE_4BIT_V << EFUSE_CHIP_PACKAGE_4BIT_S)
#define EFUSE_CHIP_PACKAGE_4BIT_V  0x00000001U
#define EFUSE_CHIP_PACKAGE_4BIT_S  2
/** EFUSE_DIS_CACHE : R; bitpos: [3]; default: 0;
 *  Disables cache
 */
#define EFUSE_DIS_CACHE    (BIT(3))
#define EFUSE_DIS_CACHE_M  (EFUSE_DIS_CACHE_V << EFUSE_DIS_CACHE_S)
#define EFUSE_DIS_CACHE_V  0x00000001U
#define EFUSE_DIS_CACHE_S  3
/** EFUSE_SPI_PAD_CONFIG_HD : R; bitpos: [8:4]; default: 0;
 *  program for SPI_pad_config_hd
 */
#define EFUSE_SPI_PAD_CONFIG_HD    0x0000001FU
#define EFUSE_SPI_PAD_CONFIG_HD_M  (EFUSE_SPI_PAD_CONFIG_HD_V << EFUSE_SPI_PAD_CONFIG_HD_S)
#define EFUSE_SPI_PAD_CONFIG_HD_V  0x0000001FU
#define EFUSE_SPI_PAD_CONFIG_HD_S  4
/** EFUSE_CHIP_PACKAGE : RW; bitpos: [11:9]; default: 0;
 *  Chip package identifier
 */
#define EFUSE_CHIP_PACKAGE    0x00000007U
#define EFUSE_CHIP_PACKAGE_M  (EFUSE_CHIP_PACKAGE_V << EFUSE_CHIP_PACKAGE_S)
#define EFUSE_CHIP_PACKAGE_V  0x00000007U
#define EFUSE_CHIP_PACKAGE_S  9
/** EFUSE_CHIP_CPU_FREQ_LOW : RW; bitpos: [12]; default: 0;
 *  If set alongside EFUSE_RD_CHIP_CPU_FREQ_RATED, the ESP32's max CPU frequency is
 *  rated for 160MHz. 240MHz otherwise
 */
#define EFUSE_CHIP_CPU_FREQ_LOW    (BIT(12))
#define EFUSE_CHIP_CPU_FREQ_LOW_M  (EFUSE_CHIP_CPU_FREQ_LOW_V << EFUSE_CHIP_CPU_FREQ_LOW_S)
#define EFUSE_CHIP_CPU_FREQ_LOW_V  0x00000001U
#define EFUSE_CHIP_CPU_FREQ_LOW_S  12
/** EFUSE_CHIP_CPU_FREQ_RATED : RW; bitpos: [13]; default: 0;
 *  If set, the ESP32's maximum CPU frequency has been rated
 */
#define EFUSE_CHIP_CPU_FREQ_RATED    (BIT(13))
#define EFUSE_CHIP_CPU_FREQ_RATED_M  (EFUSE_CHIP_CPU_FREQ_RATED_V << EFUSE_CHIP_CPU_FREQ_RATED_S)
#define EFUSE_CHIP_CPU_FREQ_RATED_V  0x00000001U
#define EFUSE_CHIP_CPU_FREQ_RATED_S  13
/** EFUSE_BLK3_PART_RESERVE : RW; bitpos: [14]; default: 0;
 *  If set, this bit indicates that BLOCK3[143:96] is reserved for internal use
 */
#define EFUSE_BLK3_PART_RESERVE    (BIT(14))
#define EFUSE_BLK3_PART_RESERVE_M  (EFUSE_BLK3_PART_RESERVE_V << EFUSE_BLK3_PART_RESERVE_S)
#define EFUSE_BLK3_PART_RESERVE_V  0x00000001U
#define EFUSE_BLK3_PART_RESERVE_S  14
/** EFUSE_CHIP_VER_REV1 : RW; bitpos: [15]; default: 0;
 *  bit is set to 1 for rev1 silicon
 */
#define EFUSE_CHIP_VER_REV1    (BIT(15))
#define EFUSE_CHIP_VER_REV1_M  (EFUSE_CHIP_VER_REV1_V << EFUSE_CHIP_VER_REV1_S)
#define EFUSE_CHIP_VER_REV1_V  0x00000001U
#define EFUSE_CHIP_VER_REV1_S  15
/** EFUSE_RESERVE_0_112 : RW; bitpos: [31:16]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RESERVE_0_112    0x0000FFFFU
#define EFUSE_RESERVE_0_112_M  (EFUSE_RESERVE_0_112_V << EFUSE_RESERVE_0_112_S)
#define EFUSE_RESERVE_0_112_V  0x0000FFFFU
#define EFUSE_RESERVE_0_112_S  16

/** EFUSE_BLK0_WDATA4_REG register */
#define EFUSE_BLK0_WDATA4_REG (DR_REG_EFUSE_BASE + 0x2c)
/** EFUSE_CLK8M_FREQ : R; bitpos: [7:0]; default: 0;
 *  8MHz clock freq override
 */
#define EFUSE_CLK8M_FREQ    0x000000FFU
#define EFUSE_CLK8M_FREQ_M  (EFUSE_CLK8M_FREQ_V << EFUSE_CLK8M_FREQ_S)
#define EFUSE_CLK8M_FREQ_V  0x000000FFU
#define EFUSE_CLK8M_FREQ_S  0
/** EFUSE_ADC_VREF : RW; bitpos: [12:8]; default: 0;
 *  True ADC reference voltage
 */
#define EFUSE_ADC_VREF    0x0000001FU
#define EFUSE_ADC_VREF_M  (EFUSE_ADC_VREF_V << EFUSE_ADC_VREF_S)
#define EFUSE_ADC_VREF_V  0x0000001FU
#define EFUSE_ADC_VREF_S  8
/** EFUSE_RESERVE_0_141 : RW; bitpos: [13]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RESERVE_0_141    (BIT(13))
#define EFUSE_RESERVE_0_141_M  (EFUSE_RESERVE_0_141_V << EFUSE_RESERVE_0_141_S)
#define EFUSE_RESERVE_0_141_V  0x00000001U
#define EFUSE_RESERVE_0_141_S  13
/** EFUSE_XPD_SDIO_REG : R; bitpos: [14]; default: 0;
 *  program for XPD_SDIO_REG
 */
#define EFUSE_XPD_SDIO_REG    (BIT(14))
#define EFUSE_XPD_SDIO_REG_M  (EFUSE_XPD_SDIO_REG_V << EFUSE_XPD_SDIO_REG_S)
#define EFUSE_XPD_SDIO_REG_V  0x00000001U
#define EFUSE_XPD_SDIO_REG_S  14
/** EFUSE_XPD_SDIO_TIEH : R; bitpos: [15]; default: 0;
 *  If XPD_SDIO_FORCE & XPD_SDIO_REG
 */
#define EFUSE_XPD_SDIO_TIEH    (BIT(15))
#define EFUSE_XPD_SDIO_TIEH_M  (EFUSE_XPD_SDIO_TIEH_V << EFUSE_XPD_SDIO_TIEH_S)
#define EFUSE_XPD_SDIO_TIEH_V  0x00000001U
#define EFUSE_XPD_SDIO_TIEH_S  15
/** EFUSE_XPD_SDIO_FORCE : R; bitpos: [16]; default: 0;
 *  Ignore MTDI pin (GPIO12) for VDD_SDIO on reset
 */
#define EFUSE_XPD_SDIO_FORCE    (BIT(16))
#define EFUSE_XPD_SDIO_FORCE_M  (EFUSE_XPD_SDIO_FORCE_V << EFUSE_XPD_SDIO_FORCE_S)
#define EFUSE_XPD_SDIO_FORCE_V  0x00000001U
#define EFUSE_XPD_SDIO_FORCE_S  16
/** EFUSE_RESERVE_0_145 : RW; bitpos: [31:17]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RESERVE_0_145    0x00007FFFU
#define EFUSE_RESERVE_0_145_M  (EFUSE_RESERVE_0_145_V << EFUSE_RESERVE_0_145_S)
#define EFUSE_RESERVE_0_145_V  0x00007FFFU
#define EFUSE_RESERVE_0_145_S  17

/** EFUSE_BLK0_WDATA5_REG register */
#define EFUSE_BLK0_WDATA5_REG (DR_REG_EFUSE_BASE + 0x30)
/** EFUSE_SPI_PAD_CONFIG_CLK : R; bitpos: [4:0]; default: 0;
 *  program for SPI_pad_config_clk
 */
#define EFUSE_SPI_PAD_CONFIG_CLK    0x0000001FU
#define EFUSE_SPI_PAD_CONFIG_CLK_M  (EFUSE_SPI_PAD_CONFIG_CLK_V << EFUSE_SPI_PAD_CONFIG_CLK_S)
#define EFUSE_SPI_PAD_CONFIG_CLK_V  0x0000001FU
#define EFUSE_SPI_PAD_CONFIG_CLK_S  0
/** EFUSE_SPI_PAD_CONFIG_Q : R; bitpos: [9:5]; default: 0;
 *  program for SPI_pad_config_q
 */
#define EFUSE_SPI_PAD_CONFIG_Q    0x0000001FU
#define EFUSE_SPI_PAD_CONFIG_Q_M  (EFUSE_SPI_PAD_CONFIG_Q_V << EFUSE_SPI_PAD_CONFIG_Q_S)
#define EFUSE_SPI_PAD_CONFIG_Q_V  0x0000001FU
#define EFUSE_SPI_PAD_CONFIG_Q_S  5
/** EFUSE_SPI_PAD_CONFIG_D : R; bitpos: [14:10]; default: 0;
 *  program for SPI_pad_config_d
 */
#define EFUSE_SPI_PAD_CONFIG_D    0x0000001FU
#define EFUSE_SPI_PAD_CONFIG_D_M  (EFUSE_SPI_PAD_CONFIG_D_V << EFUSE_SPI_PAD_CONFIG_D_S)
#define EFUSE_SPI_PAD_CONFIG_D_V  0x0000001FU
#define EFUSE_SPI_PAD_CONFIG_D_S  10
/** EFUSE_SPI_PAD_CONFIG_CS0 : R; bitpos: [19:15]; default: 0;
 *  program for SPI_pad_config_cs0
 */
#define EFUSE_SPI_PAD_CONFIG_CS0    0x0000001FU
#define EFUSE_SPI_PAD_CONFIG_CS0_M  (EFUSE_SPI_PAD_CONFIG_CS0_V << EFUSE_SPI_PAD_CONFIG_CS0_S)
#define EFUSE_SPI_PAD_CONFIG_CS0_V  0x0000001FU
#define EFUSE_SPI_PAD_CONFIG_CS0_S  15
/** EFUSE_CHIP_VER_REV2 : R; bitpos: [20]; default: 0; */
#define EFUSE_CHIP_VER_REV2    (BIT(20))
#define EFUSE_CHIP_VER_REV2_M  (EFUSE_CHIP_VER_REV2_V << EFUSE_CHIP_VER_REV2_S)
#define EFUSE_CHIP_VER_REV2_V  0x00000001U
#define EFUSE_CHIP_VER_REV2_S  20
/** EFUSE_RESERVE_0_181 : RW; bitpos: [21]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RESERVE_0_181    (BIT(21))
#define EFUSE_RESERVE_0_181_M  (EFUSE_RESERVE_0_181_V << EFUSE_RESERVE_0_181_S)
#define EFUSE_RESERVE_0_181_V  0x00000001U
#define EFUSE_RESERVE_0_181_S  21
/** EFUSE_VOL_LEVEL_HP_INV : R; bitpos: [23:22]; default: 0;
 *  This field stores the voltage level for CPU to run at 240 MHz, or for flash/PSRAM
 *  to run at 80 MHz.0x0: level 7; 0x1: level 6; 0x2: level 5; 0x3: level 4. (RO)
 */
#define EFUSE_VOL_LEVEL_HP_INV    0x00000003U
#define EFUSE_VOL_LEVEL_HP_INV_M  (EFUSE_VOL_LEVEL_HP_INV_V << EFUSE_VOL_LEVEL_HP_INV_S)
#define EFUSE_VOL_LEVEL_HP_INV_V  0x00000003U
#define EFUSE_VOL_LEVEL_HP_INV_S  22
/** EFUSE_WAFER_VERSION_MINOR : R; bitpos: [25:24]; default: 0; */
#define EFUSE_WAFER_VERSION_MINOR    0x00000003U
#define EFUSE_WAFER_VERSION_MINOR_M  (EFUSE_WAFER_VERSION_MINOR_V << EFUSE_WAFER_VERSION_MINOR_S)
#define EFUSE_WAFER_VERSION_MINOR_V  0x00000003U
#define EFUSE_WAFER_VERSION_MINOR_S  24
/** EFUSE_RESERVE_0_186 : RW; bitpos: [27:26]; default: 0;
 *  Reserved, it was created by set_missed_fields_in_regs func
 */
#define EFUSE_RESERVE_0_186    0x00000003U
#define EFUSE_RESERVE_0_186_M  (EFUSE_RESERVE_0_186_V << EFUSE_RESERVE_0_186_S)
#define EFUSE_RESERVE_0_186_V  0x00000003U
#define EFUSE_RESERVE_0_186_S  26
/** EFUSE_FLASH_CRYPT_CONFIG : R; bitpos: [31:28]; default: 0;
 *  program for flash_crypt_config
 */
#define EFUSE_FLASH_CRYPT_CONFIG    0x0000000FU
#define EFUSE_FLASH_CRYPT_CONFIG_M  (EFUSE_FLASH_CRYPT_CONFIG_V << EFUSE_FLASH_CRYPT_CONFIG_S)
#define EFUSE_FLASH_CRYPT_CONFIG_V  0x0000000FU
#define EFUSE_FLASH_CRYPT_CONFIG_S  28

/** EFUSE_BLK0_WDATA6_REG register */
#define EFUSE_BLK0_WDATA6_REG (DR_REG_EFUSE_BASE + 0x34)
/** EFUSE_CODING_SCHEME : RW; bitpos: [1:0]; default: 0;
 *  program for coding_scheme
 */
#define EFUSE_CODING_SCHEME    0x00000003U
#define EFUSE_CODING_SCHEME_M  (EFUSE_CODING_SCHEME_V << EFUSE_CODING_SCHEME_S)
#define EFUSE_CODING_SCHEME_V  0x00000003U
#define EFUSE_CODING_SCHEME_S  0
/** EFUSE_CONSOLE_DEBUG_DISABLE : RW; bitpos: [2]; default: 0;
 *  program for console_debug_disable
 */
#define EFUSE_CONSOLE_DEBUG_DISABLE    (BIT(2))
#define EFUSE_CONSOLE_DEBUG_DISABLE_M  (EFUSE_CONSOLE_DEBUG_DISABLE_V << EFUSE_CONSOLE_DEBUG_DISABLE_S)
#define EFUSE_CONSOLE_DEBUG_DISABLE_V  0x00000001U
#define EFUSE_CONSOLE_DEBUG_DISABLE_S  2
/** EFUSE_DISABLE_SDIO_HOST : RW; bitpos: [3]; default: 0; */
#define EFUSE_DISABLE_SDIO_HOST    (BIT(3))
#define EFUSE_DISABLE_SDIO_HOST_M  (EFUSE_DISABLE_SDIO_HOST_V << EFUSE_DISABLE_SDIO_HOST_S)
#define EFUSE_DISABLE_SDIO_HOST_V  0x00000001U
#define EFUSE_DISABLE_SDIO_HOST_S  3
/** EFUSE_ABS_DONE_0 : RW; bitpos: [4]; default: 0;
 *  program for abstract_done_0
 */
#define EFUSE_ABS_DONE_0    (BIT(4))
#define EFUSE_ABS_DONE_0_M  (EFUSE_ABS_DONE_0_V << EFUSE_ABS_DONE_0_S)
#define EFUSE_ABS_DONE_0_V  0x00000001U
#define EFUSE_ABS_DONE_0_S  4
/** EFUSE_ABS_DONE_1 : RW; bitpos: [5]; default: 0;
 *  program for abstract_done_1
 */
#define EFUSE_ABS_DONE_1    (BIT(5))
#define EFUSE_ABS_DONE_1_M  (EFUSE_ABS_DONE_1_V << EFUSE_ABS_DONE_1_S)
#define EFUSE_ABS_DONE_1_V  0x00000001U
#define EFUSE_ABS_DONE_1_S  5
/** EFUSE_DISABLE_JTAG : RW; bitpos: [6]; default: 0;
 *  program for JTAG_disable
 */
#define EFUSE_DISABLE_JTAG    (BIT(6))
#define EFUSE_DISABLE_JTAG_M  (EFUSE_DISABLE_JTAG_V << EFUSE_DISABLE_JTAG_S)
#define EFUSE_DISABLE_JTAG_V  0x00000001U
#define EFUSE_DISABLE_JTAG_S  6
/** EFUSE_DISABLE_DL_ENCRYPT : RW; bitpos: [7]; default: 0;
 *  program for download_dis_encrypt
 */
#define EFUSE_DISABLE_DL_ENCRYPT    (BIT(7))
#define EFUSE_DISABLE_DL_ENCRYPT_M  (EFUSE_DISABLE_DL_ENCRYPT_V << EFUSE_DISABLE_DL_ENCRYPT_S)
#define EFUSE_DISABLE_DL_ENCRYPT_V  0x00000001U
#define EFUSE_DISABLE_DL_ENCRYPT_S  7
/** EFUSE_DISABLE_DL_DECRYPT : RW; bitpos: [8]; default: 0;
 *  program for download_dis_decrypt
 */
#define EFUSE_DISABLE_DL_DECRYPT    (BIT(8))
#define EFUSE_DISABLE_DL_DECRYPT_M  (EFUSE_DISABLE_DL_DECRYPT_V << EFUSE_DISABLE_DL_DECRYPT_S)
#define EFUSE_DISABLE_DL_DECRYPT_V  0x00000001U
#define EFUSE_DISABLE_DL_DECRYPT_S  8
/** EFUSE_DISABLE_DL_CACHE : RW; bitpos: [9]; default: 0;
 *  program for download_dis_cache
 */
#define EFUSE_DISABLE_DL_CACHE    (BIT(9))
#define EFUSE_DISABLE_DL_CACHE_M  (EFUSE_DISABLE_DL_CACHE_V << EFUSE_DISABLE_DL_CACHE_S)
#define EFUSE_DISABLE_DL_CACHE_V  0x00000001U
#define EFUSE_DISABLE_DL_CACHE_S  9
/** EFUSE_KEY_STATUS : RW; bitpos: [10]; default: 0;
 *  program for key_status
 */
#define EFUSE_KEY_STATUS    (BIT(10))
#define EFUSE_KEY_STATUS_M  (EFUSE_KEY_STATUS_V << EFUSE_KEY_STATUS_S)
#define EFUSE_KEY_STATUS_V  0x00000001U
#define EFUSE_KEY_STATUS_S  10

/** EFUSE_BLK1_RDATA0_REG register */
#define EFUSE_BLK1_RDATA0_REG (DR_REG_EFUSE_BASE + 0x38)
/** EFUSE_RD_BLOCK1 : R; bitpos: [31:0]; default: 0;
 *  Flash encryption key
 */
#define EFUSE_RD_BLOCK1    0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_M  (EFUSE_RD_BLOCK1_V << EFUSE_RD_BLOCK1_S)
#define EFUSE_RD_BLOCK1_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_S  0

/** EFUSE_BLK1_RDATA1_REG register */
#define EFUSE_BLK1_RDATA1_REG (DR_REG_EFUSE_BASE + 0x3c)
/** EFUSE_RD_BLOCK1_1 : R; bitpos: [31:0]; default: 0;
 *  Flash encryption key
 */
#define EFUSE_RD_BLOCK1_1    0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_1_M  (EFUSE_RD_BLOCK1_1_V << EFUSE_RD_BLOCK1_1_S)
#define EFUSE_RD_BLOCK1_1_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_1_S  0

/** EFUSE_BLK1_RDATA2_REG register */
#define EFUSE_BLK1_RDATA2_REG (DR_REG_EFUSE_BASE + 0x40)
/** EFUSE_RD_BLOCK1_2 : R; bitpos: [31:0]; default: 0;
 *  Flash encryption key
 */
#define EFUSE_RD_BLOCK1_2    0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_2_M  (EFUSE_RD_BLOCK1_2_V << EFUSE_RD_BLOCK1_2_S)
#define EFUSE_RD_BLOCK1_2_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_2_S  0

/** EFUSE_BLK1_RDATA3_REG register */
#define EFUSE_BLK1_RDATA3_REG (DR_REG_EFUSE_BASE + 0x44)
/** EFUSE_RD_BLOCK1_3 : R; bitpos: [31:0]; default: 0;
 *  Flash encryption key
 */
#define EFUSE_RD_BLOCK1_3    0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_3_M  (EFUSE_RD_BLOCK1_3_V << EFUSE_RD_BLOCK1_3_S)
#define EFUSE_RD_BLOCK1_3_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_3_S  0

/** EFUSE_BLK1_RDATA4_REG register */
#define EFUSE_BLK1_RDATA4_REG (DR_REG_EFUSE_BASE + 0x48)
/** EFUSE_RD_BLOCK1_4 : R; bitpos: [31:0]; default: 0;
 *  Flash encryption key
 */
#define EFUSE_RD_BLOCK1_4    0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_4_M  (EFUSE_RD_BLOCK1_4_V << EFUSE_RD_BLOCK1_4_S)
#define EFUSE_RD_BLOCK1_4_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_4_S  0

/** EFUSE_BLK1_RDATA5_REG register */
#define EFUSE_BLK1_RDATA5_REG (DR_REG_EFUSE_BASE + 0x4c)
/** EFUSE_RD_BLOCK1_5 : R; bitpos: [31:0]; default: 0;
 *  Flash encryption key
 */
#define EFUSE_RD_BLOCK1_5    0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_5_M  (EFUSE_RD_BLOCK1_5_V << EFUSE_RD_BLOCK1_5_S)
#define EFUSE_RD_BLOCK1_5_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_5_S  0

/** EFUSE_BLK1_RDATA6_REG register */
#define EFUSE_BLK1_RDATA6_REG (DR_REG_EFUSE_BASE + 0x50)
/** EFUSE_RD_BLOCK1_6 : R; bitpos: [31:0]; default: 0;
 *  Flash encryption key
 */
#define EFUSE_RD_BLOCK1_6    0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_6_M  (EFUSE_RD_BLOCK1_6_V << EFUSE_RD_BLOCK1_6_S)
#define EFUSE_RD_BLOCK1_6_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_6_S  0

/** EFUSE_BLK1_RDATA7_REG register */
#define EFUSE_BLK1_RDATA7_REG (DR_REG_EFUSE_BASE + 0x54)
/** EFUSE_RD_BLOCK1_7 : R; bitpos: [31:0]; default: 0;
 *  Flash encryption key
 */
#define EFUSE_RD_BLOCK1_7    0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_7_M  (EFUSE_RD_BLOCK1_7_V << EFUSE_RD_BLOCK1_7_S)
#define EFUSE_RD_BLOCK1_7_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK1_7_S  0

/** EFUSE_BLK2_RDATA0_REG register */
#define EFUSE_BLK2_RDATA0_REG (DR_REG_EFUSE_BASE + 0x58)
/** EFUSE_RD_BLOCK2 : R; bitpos: [31:0]; default: 0;
 *  Security boot key
 */
#define EFUSE_RD_BLOCK2    0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_M  (EFUSE_RD_BLOCK2_V << EFUSE_RD_BLOCK2_S)
#define EFUSE_RD_BLOCK2_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_S  0

/** EFUSE_BLK2_RDATA1_REG register */
#define EFUSE_BLK2_RDATA1_REG (DR_REG_EFUSE_BASE + 0x5c)
/** EFUSE_RD_BLOCK2_1 : R; bitpos: [31:0]; default: 0;
 *  Security boot key
 */
#define EFUSE_RD_BLOCK2_1    0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_1_M  (EFUSE_RD_BLOCK2_1_V << EFUSE_RD_BLOCK2_1_S)
#define EFUSE_RD_BLOCK2_1_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_1_S  0

/** EFUSE_BLK2_RDATA2_REG register */
#define EFUSE_BLK2_RDATA2_REG (DR_REG_EFUSE_BASE + 0x60)
/** EFUSE_RD_BLOCK2_2 : R; bitpos: [31:0]; default: 0;
 *  Security boot key
 */
#define EFUSE_RD_BLOCK2_2    0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_2_M  (EFUSE_RD_BLOCK2_2_V << EFUSE_RD_BLOCK2_2_S)
#define EFUSE_RD_BLOCK2_2_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_2_S  0

/** EFUSE_BLK2_RDATA3_REG register */
#define EFUSE_BLK2_RDATA3_REG (DR_REG_EFUSE_BASE + 0x64)
/** EFUSE_RD_BLOCK2_3 : R; bitpos: [31:0]; default: 0;
 *  Security boot key
 */
#define EFUSE_RD_BLOCK2_3    0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_3_M  (EFUSE_RD_BLOCK2_3_V << EFUSE_RD_BLOCK2_3_S)
#define EFUSE_RD_BLOCK2_3_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_3_S  0

/** EFUSE_BLK2_RDATA4_REG register */
#define EFUSE_BLK2_RDATA4_REG (DR_REG_EFUSE_BASE + 0x68)
/** EFUSE_RD_BLOCK2_4 : R; bitpos: [31:0]; default: 0;
 *  Security boot key
 */
#define EFUSE_RD_BLOCK2_4    0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_4_M  (EFUSE_RD_BLOCK2_4_V << EFUSE_RD_BLOCK2_4_S)
#define EFUSE_RD_BLOCK2_4_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_4_S  0

/** EFUSE_BLK2_RDATA5_REG register */
#define EFUSE_BLK2_RDATA5_REG (DR_REG_EFUSE_BASE + 0x6c)
/** EFUSE_RD_BLOCK2_5 : R; bitpos: [31:0]; default: 0;
 *  Security boot key
 */
#define EFUSE_RD_BLOCK2_5    0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_5_M  (EFUSE_RD_BLOCK2_5_V << EFUSE_RD_BLOCK2_5_S)
#define EFUSE_RD_BLOCK2_5_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_5_S  0

/** EFUSE_BLK2_RDATA6_REG register */
#define EFUSE_BLK2_RDATA6_REG (DR_REG_EFUSE_BASE + 0x70)
/** EFUSE_RD_BLOCK2_6 : R; bitpos: [31:0]; default: 0;
 *  Security boot key
 */
#define EFUSE_RD_BLOCK2_6    0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_6_M  (EFUSE_RD_BLOCK2_6_V << EFUSE_RD_BLOCK2_6_S)
#define EFUSE_RD_BLOCK2_6_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_6_S  0

/** EFUSE_BLK2_RDATA7_REG register */
#define EFUSE_BLK2_RDATA7_REG (DR_REG_EFUSE_BASE + 0x74)
/** EFUSE_RD_BLOCK2_7 : R; bitpos: [31:0]; default: 0;
 *  Security boot key
 */
#define EFUSE_RD_BLOCK2_7    0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_7_M  (EFUSE_RD_BLOCK2_7_V << EFUSE_RD_BLOCK2_7_S)
#define EFUSE_RD_BLOCK2_7_V  0xFFFFFFFFU
#define EFUSE_RD_BLOCK2_7_S  0

/** EFUSE_BLK3_RDATA0_REG register */
#define EFUSE_BLK3_RDATA0_REG (DR_REG_EFUSE_BASE + 0x78)
/** EFUSE_RD_CUSTOM_MAC_CRC : R; bitpos: [7:0]; default: 0;
 *  CRC8 for custom MAC address
 */
#define EFUSE_RD_CUSTOM_MAC_CRC    0x000000FFU
#define EFUSE_RD_CUSTOM_MAC_CRC_M  (EFUSE_RD_CUSTOM_MAC_CRC_V << EFUSE_RD_CUSTOM_MAC_CRC_S)
#define EFUSE_RD_CUSTOM_MAC_CRC_V  0x000000FFU
#define EFUSE_RD_CUSTOM_MAC_CRC_S  0
/** EFUSE_RD_CUSTOM_MAC : R; bitpos: [31:8]; default: 0;
 *  Custom MAC address
 */
#define EFUSE_RD_CUSTOM_MAC    0x00FFFFFFU
#define EFUSE_RD_CUSTOM_MAC_M  (EFUSE_RD_CUSTOM_MAC_V << EFUSE_RD_CUSTOM_MAC_S)
#define EFUSE_RD_CUSTOM_MAC_V  0x00FFFFFFU
#define EFUSE_RD_CUSTOM_MAC_S  8

/** EFUSE_BLK3_RDATA1_REG register */
#define EFUSE_BLK3_RDATA1_REG (DR_REG_EFUSE_BASE + 0x7c)
/** EFUSE_RD_CUSTOM_MAC_1 : R; bitpos: [23:0]; default: 0;
 *  Custom MAC address
 */
#define EFUSE_RD_CUSTOM_MAC_1    0x00FFFFFFU
#define EFUSE_RD_CUSTOM_MAC_1_M  (EFUSE_RD_CUSTOM_MAC_1_V << EFUSE_RD_CUSTOM_MAC_1_S)
#define EFUSE_RD_CUSTOM_MAC_1_V  0x00FFFFFFU
#define EFUSE_RD_CUSTOM_MAC_1_S  0
/** EFUSE_RESERVED_3_56 : R; bitpos: [31:24]; default: 0;
 *  reserved
 */
#define EFUSE_RESERVED_3_56    0x000000FFU
#define EFUSE_RESERVED_3_56_M  (EFUSE_RESERVED_3_56_V << EFUSE_RESERVED_3_56_S)
#define EFUSE_RESERVED_3_56_V  0x000000FFU
#define EFUSE_RESERVED_3_56_S  24

/** EFUSE_BLK3_RDATA2_REG register */
#define EFUSE_BLK3_RDATA2_REG (DR_REG_EFUSE_BASE + 0x80)
/** EFUSE_RD_BLK3_RESERVED_2 : R; bitpos: [31:0]; default: 0;
 *  read for BLOCK3
 */
#define EFUSE_RD_BLK3_RESERVED_2    0xFFFFFFFFU
#define EFUSE_RD_BLK3_RESERVED_2_M  (EFUSE_RD_BLK3_RESERVED_2_V << EFUSE_RD_BLK3_RESERVED_2_S)
#define EFUSE_RD_BLK3_RESERVED_2_V  0xFFFFFFFFU
#define EFUSE_RD_BLK3_RESERVED_2_S  0

/** EFUSE_BLK3_RDATA3_REG register */
#define EFUSE_BLK3_RDATA3_REG (DR_REG_EFUSE_BASE + 0x84)
/** EFUSE_RD_ADC1_TP_LOW : RW; bitpos: [6:0]; default: 0;
 *  ADC1 Two Point calibration low point. Only valid if EFUSE_RD_BLK3_PART_RESERVE
 */
#define EFUSE_RD_ADC1_TP_LOW    0x0000007FU
#define EFUSE_RD_ADC1_TP_LOW_M  (EFUSE_RD_ADC1_TP_LOW_V << EFUSE_RD_ADC1_TP_LOW_S)
#define EFUSE_RD_ADC1_TP_LOW_V  0x0000007FU
#define EFUSE_RD_ADC1_TP_LOW_S  0
/** EFUSE_RD_ADC1_TP_HIGH : RW; bitpos: [15:7]; default: 0;
 *  ADC1 Two Point calibration high point. Only valid if EFUSE_RD_BLK3_PART_RESERVE
 */
#define EFUSE_RD_ADC1_TP_HIGH    0x000001FFU
#define EFUSE_RD_ADC1_TP_HIGH_M  (EFUSE_RD_ADC1_TP_HIGH_V << EFUSE_RD_ADC1_TP_HIGH_S)
#define EFUSE_RD_ADC1_TP_HIGH_V  0x000001FFU
#define EFUSE_RD_ADC1_TP_HIGH_S  7
/** EFUSE_RD_ADC2_TP_LOW : RW; bitpos: [22:16]; default: 0;
 *  ADC2 Two Point calibration low point. Only valid if EFUSE_RD_BLK3_PART_RESERVE
 */
#define EFUSE_RD_ADC2_TP_LOW    0x0000007FU
#define EFUSE_RD_ADC2_TP_LOW_M  (EFUSE_RD_ADC2_TP_LOW_V << EFUSE_RD_ADC2_TP_LOW_S)
#define EFUSE_RD_ADC2_TP_LOW_V  0x0000007FU
#define EFUSE_RD_ADC2_TP_LOW_S  16
/** EFUSE_RD_ADC2_TP_HIGH : RW; bitpos: [31:23]; default: 0;
 *  ADC2 Two Point calibration high point. Only valid if EFUSE_RD_BLK3_PART_RESERVE
 */
#define EFUSE_RD_ADC2_TP_HIGH    0x000001FFU
#define EFUSE_RD_ADC2_TP_HIGH_M  (EFUSE_RD_ADC2_TP_HIGH_V << EFUSE_RD_ADC2_TP_HIGH_S)
#define EFUSE_RD_ADC2_TP_HIGH_V  0x000001FFU
#define EFUSE_RD_ADC2_TP_HIGH_S  23

/** EFUSE_BLK3_RDATA4_REG register */
#define EFUSE_BLK3_RDATA4_REG (DR_REG_EFUSE_BASE + 0x88)
/** EFUSE_RD_SECURE_VERSION : R; bitpos: [31:0]; default: 0;
 *  Secure version for anti-rollback
 */
#define EFUSE_RD_SECURE_VERSION    0xFFFFFFFFU
#define EFUSE_RD_SECURE_VERSION_M  (EFUSE_RD_SECURE_VERSION_V << EFUSE_RD_SECURE_VERSION_S)
#define EFUSE_RD_SECURE_VERSION_V  0xFFFFFFFFU
#define EFUSE_RD_SECURE_VERSION_S  0

/** EFUSE_BLK3_RDATA5_REG register */
#define EFUSE_BLK3_RDATA5_REG (DR_REG_EFUSE_BASE + 0x8c)
/** EFUSE_RESERVED_3_160 : R; bitpos: [23:0]; default: 0;
 *  reserved
 */
#define EFUSE_RESERVED_3_160    0x00FFFFFFU
#define EFUSE_RESERVED_3_160_M  (EFUSE_RESERVED_3_160_V << EFUSE_RESERVED_3_160_S)
#define EFUSE_RESERVED_3_160_V  0x00FFFFFFU
#define EFUSE_RESERVED_3_160_S  0
/** EFUSE_RD_MAC_VERSION : R; bitpos: [31:24]; default: 0;
 *  Custom MAC version
 */
#define EFUSE_RD_MAC_VERSION    0x000000FFU
#define EFUSE_RD_MAC_VERSION_M  (EFUSE_RD_MAC_VERSION_V << EFUSE_RD_MAC_VERSION_S)
#define EFUSE_RD_MAC_VERSION_V  0x000000FFU
#define EFUSE_RD_MAC_VERSION_S  24

/** EFUSE_BLK3_RDATA6_REG register */
#define EFUSE_BLK3_RDATA6_REG (DR_REG_EFUSE_BASE + 0x90)
/** EFUSE_RD_BLK3_RESERVED_6 : R; bitpos: [31:0]; default: 0;
 *  read for BLOCK3
 */
#define EFUSE_RD_BLK3_RESERVED_6    0xFFFFFFFFU
#define EFUSE_RD_BLK3_RESERVED_6_M  (EFUSE_RD_BLK3_RESERVED_6_V << EFUSE_RD_BLK3_RESERVED_6_S)
#define EFUSE_RD_BLK3_RESERVED_6_V  0xFFFFFFFFU
#define EFUSE_RD_BLK3_RESERVED_6_S  0

/** EFUSE_BLK3_RDATA7_REG register */
#define EFUSE_BLK3_RDATA7_REG (DR_REG_EFUSE_BASE + 0x94)
/** EFUSE_RD_BLK3_RESERVED_7 : R; bitpos: [31:0]; default: 0;
 *  read for BLOCK3
 */
#define EFUSE_RD_BLK3_RESERVED_7    0xFFFFFFFFU
#define EFUSE_RD_BLK3_RESERVED_7_M  (EFUSE_RD_BLK3_RESERVED_7_V << EFUSE_RD_BLK3_RESERVED_7_S)
#define EFUSE_RD_BLK3_RESERVED_7_V  0xFFFFFFFFU
#define EFUSE_RD_BLK3_RESERVED_7_S  0

/** EFUSE_BLK1_WDATA0_REG register */
#define EFUSE_BLK1_WDATA0_REG (DR_REG_EFUSE_BASE + 0x98)
/** EFUSE_BLK1_DIN0 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK1
 */
#define EFUSE_BLK1_DIN0    0xFFFFFFFFU
#define EFUSE_BLK1_DIN0_M  (EFUSE_BLK1_DIN0_V << EFUSE_BLK1_DIN0_S)
#define EFUSE_BLK1_DIN0_V  0xFFFFFFFFU
#define EFUSE_BLK1_DIN0_S  0

/** EFUSE_BLK1_WDATA1_REG register */
#define EFUSE_BLK1_WDATA1_REG (DR_REG_EFUSE_BASE + 0x9c)
/** EFUSE_BLK1_DIN1 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK1
 */
#define EFUSE_BLK1_DIN1    0xFFFFFFFFU
#define EFUSE_BLK1_DIN1_M  (EFUSE_BLK1_DIN1_V << EFUSE_BLK1_DIN1_S)
#define EFUSE_BLK1_DIN1_V  0xFFFFFFFFU
#define EFUSE_BLK1_DIN1_S  0

/** EFUSE_BLK1_WDATA2_REG register */
#define EFUSE_BLK1_WDATA2_REG (DR_REG_EFUSE_BASE + 0xa0)
/** EFUSE_BLK1_DIN2 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK1
 */
#define EFUSE_BLK1_DIN2    0xFFFFFFFFU
#define EFUSE_BLK1_DIN2_M  (EFUSE_BLK1_DIN2_V << EFUSE_BLK1_DIN2_S)
#define EFUSE_BLK1_DIN2_V  0xFFFFFFFFU
#define EFUSE_BLK1_DIN2_S  0

/** EFUSE_BLK1_WDATA3_REG register */
#define EFUSE_BLK1_WDATA3_REG (DR_REG_EFUSE_BASE + 0xa4)
/** EFUSE_BLK1_DIN3 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK1
 */
#define EFUSE_BLK1_DIN3    0xFFFFFFFFU
#define EFUSE_BLK1_DIN3_M  (EFUSE_BLK1_DIN3_V << EFUSE_BLK1_DIN3_S)
#define EFUSE_BLK1_DIN3_V  0xFFFFFFFFU
#define EFUSE_BLK1_DIN3_S  0

/** EFUSE_BLK1_WDATA4_REG register */
#define EFUSE_BLK1_WDATA4_REG (DR_REG_EFUSE_BASE + 0xa8)
/** EFUSE_BLK1_DIN4 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK1
 */
#define EFUSE_BLK1_DIN4    0xFFFFFFFFU
#define EFUSE_BLK1_DIN4_M  (EFUSE_BLK1_DIN4_V << EFUSE_BLK1_DIN4_S)
#define EFUSE_BLK1_DIN4_V  0xFFFFFFFFU
#define EFUSE_BLK1_DIN4_S  0

/** EFUSE_BLK1_WDATA5_REG register */
#define EFUSE_BLK1_WDATA5_REG (DR_REG_EFUSE_BASE + 0xac)
/** EFUSE_BLK1_DIN5 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK1
 */
#define EFUSE_BLK1_DIN5    0xFFFFFFFFU
#define EFUSE_BLK1_DIN5_M  (EFUSE_BLK1_DIN5_V << EFUSE_BLK1_DIN5_S)
#define EFUSE_BLK1_DIN5_V  0xFFFFFFFFU
#define EFUSE_BLK1_DIN5_S  0

/** EFUSE_BLK1_WDATA6_REG register */
#define EFUSE_BLK1_WDATA6_REG (DR_REG_EFUSE_BASE + 0xb0)
/** EFUSE_BLK1_DIN6 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK1
 */
#define EFUSE_BLK1_DIN6    0xFFFFFFFFU
#define EFUSE_BLK1_DIN6_M  (EFUSE_BLK1_DIN6_V << EFUSE_BLK1_DIN6_S)
#define EFUSE_BLK1_DIN6_V  0xFFFFFFFFU
#define EFUSE_BLK1_DIN6_S  0

/** EFUSE_BLK1_WDATA7_REG register */
#define EFUSE_BLK1_WDATA7_REG (DR_REG_EFUSE_BASE + 0xb4)
/** EFUSE_BLK1_DIN7 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK1
 */
#define EFUSE_BLK1_DIN7    0xFFFFFFFFU
#define EFUSE_BLK1_DIN7_M  (EFUSE_BLK1_DIN7_V << EFUSE_BLK1_DIN7_S)
#define EFUSE_BLK1_DIN7_V  0xFFFFFFFFU
#define EFUSE_BLK1_DIN7_S  0

/** EFUSE_BLK2_WDATA0_REG register */
#define EFUSE_BLK2_WDATA0_REG (DR_REG_EFUSE_BASE + 0xb8)
/** EFUSE_BLK2_DIN0 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK2
 */
#define EFUSE_BLK2_DIN0    0xFFFFFFFFU
#define EFUSE_BLK2_DIN0_M  (EFUSE_BLK2_DIN0_V << EFUSE_BLK2_DIN0_S)
#define EFUSE_BLK2_DIN0_V  0xFFFFFFFFU
#define EFUSE_BLK2_DIN0_S  0

/** EFUSE_BLK2_WDATA1_REG register */
#define EFUSE_BLK2_WDATA1_REG (DR_REG_EFUSE_BASE + 0xbc)
/** EFUSE_BLK2_DIN1 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK2
 */
#define EFUSE_BLK2_DIN1    0xFFFFFFFFU
#define EFUSE_BLK2_DIN1_M  (EFUSE_BLK2_DIN1_V << EFUSE_BLK2_DIN1_S)
#define EFUSE_BLK2_DIN1_V  0xFFFFFFFFU
#define EFUSE_BLK2_DIN1_S  0

/** EFUSE_BLK2_WDATA2_REG register */
#define EFUSE_BLK2_WDATA2_REG (DR_REG_EFUSE_BASE + 0xc0)
/** EFUSE_BLK2_DIN2 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK2
 */
#define EFUSE_BLK2_DIN2    0xFFFFFFFFU
#define EFUSE_BLK2_DIN2_M  (EFUSE_BLK2_DIN2_V << EFUSE_BLK2_DIN2_S)
#define EFUSE_BLK2_DIN2_V  0xFFFFFFFFU
#define EFUSE_BLK2_DIN2_S  0

/** EFUSE_BLK2_WDATA3_REG register */
#define EFUSE_BLK2_WDATA3_REG (DR_REG_EFUSE_BASE + 0xc4)
/** EFUSE_BLK2_DIN3 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK2
 */
#define EFUSE_BLK2_DIN3    0xFFFFFFFFU
#define EFUSE_BLK2_DIN3_M  (EFUSE_BLK2_DIN3_V << EFUSE_BLK2_DIN3_S)
#define EFUSE_BLK2_DIN3_V  0xFFFFFFFFU
#define EFUSE_BLK2_DIN3_S  0

/** EFUSE_BLK2_WDATA4_REG register */
#define EFUSE_BLK2_WDATA4_REG (DR_REG_EFUSE_BASE + 0xc8)
/** EFUSE_BLK2_DIN4 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK2
 */
#define EFUSE_BLK2_DIN4    0xFFFFFFFFU
#define EFUSE_BLK2_DIN4_M  (EFUSE_BLK2_DIN4_V << EFUSE_BLK2_DIN4_S)
#define EFUSE_BLK2_DIN4_V  0xFFFFFFFFU
#define EFUSE_BLK2_DIN4_S  0

/** EFUSE_BLK2_WDATA5_REG register */
#define EFUSE_BLK2_WDATA5_REG (DR_REG_EFUSE_BASE + 0xcc)
/** EFUSE_BLK2_DIN5 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK2
 */
#define EFUSE_BLK2_DIN5    0xFFFFFFFFU
#define EFUSE_BLK2_DIN5_M  (EFUSE_BLK2_DIN5_V << EFUSE_BLK2_DIN5_S)
#define EFUSE_BLK2_DIN5_V  0xFFFFFFFFU
#define EFUSE_BLK2_DIN5_S  0

/** EFUSE_BLK2_WDATA6_REG register */
#define EFUSE_BLK2_WDATA6_REG (DR_REG_EFUSE_BASE + 0xd0)
/** EFUSE_BLK2_DIN6 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK2
 */
#define EFUSE_BLK2_DIN6    0xFFFFFFFFU
#define EFUSE_BLK2_DIN6_M  (EFUSE_BLK2_DIN6_V << EFUSE_BLK2_DIN6_S)
#define EFUSE_BLK2_DIN6_V  0xFFFFFFFFU
#define EFUSE_BLK2_DIN6_S  0

/** EFUSE_BLK2_WDATA7_REG register */
#define EFUSE_BLK2_WDATA7_REG (DR_REG_EFUSE_BASE + 0xd4)
/** EFUSE_BLK2_DIN7 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK2
 */
#define EFUSE_BLK2_DIN7    0xFFFFFFFFU
#define EFUSE_BLK2_DIN7_M  (EFUSE_BLK2_DIN7_V << EFUSE_BLK2_DIN7_S)
#define EFUSE_BLK2_DIN7_V  0xFFFFFFFFU
#define EFUSE_BLK2_DIN7_S  0

/** EFUSE_BLK3_WDATA0_REG register */
#define EFUSE_BLK3_WDATA0_REG (DR_REG_EFUSE_BASE + 0xd8)
/** EFUSE_BLK3_DIN0 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK3
 */
#define EFUSE_BLK3_DIN0    0xFFFFFFFFU
#define EFUSE_BLK3_DIN0_M  (EFUSE_BLK3_DIN0_V << EFUSE_BLK3_DIN0_S)
#define EFUSE_BLK3_DIN0_V  0xFFFFFFFFU
#define EFUSE_BLK3_DIN0_S  0

/** EFUSE_BLK3_WDATA1_REG register */
#define EFUSE_BLK3_WDATA1_REG (DR_REG_EFUSE_BASE + 0xdc)
/** EFUSE_BLK3_DIN1 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK3
 */
#define EFUSE_BLK3_DIN1    0xFFFFFFFFU
#define EFUSE_BLK3_DIN1_M  (EFUSE_BLK3_DIN1_V << EFUSE_BLK3_DIN1_S)
#define EFUSE_BLK3_DIN1_V  0xFFFFFFFFU
#define EFUSE_BLK3_DIN1_S  0

/** EFUSE_BLK3_WDATA2_REG register */
#define EFUSE_BLK3_WDATA2_REG (DR_REG_EFUSE_BASE + 0xe0)
/** EFUSE_BLK3_DIN2 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK3
 */
#define EFUSE_BLK3_DIN2    0xFFFFFFFFU
#define EFUSE_BLK3_DIN2_M  (EFUSE_BLK3_DIN2_V << EFUSE_BLK3_DIN2_S)
#define EFUSE_BLK3_DIN2_V  0xFFFFFFFFU
#define EFUSE_BLK3_DIN2_S  0

/** EFUSE_BLK3_WDATA3_REG register */
#define EFUSE_BLK3_WDATA3_REG (DR_REG_EFUSE_BASE + 0xe4)
/** EFUSE_ADC1_TP_LOW : RW; bitpos: [6:0]; default: 0;
 *  ADC1 Two Point calibration low point. Only valid if EFUSE_RD_BLK3_PART_RESERVE
 */
#define EFUSE_ADC1_TP_LOW    0x0000007FU
#define EFUSE_ADC1_TP_LOW_M  (EFUSE_ADC1_TP_LOW_V << EFUSE_ADC1_TP_LOW_S)
#define EFUSE_ADC1_TP_LOW_V  0x0000007FU
#define EFUSE_ADC1_TP_LOW_S  0
/** EFUSE_ADC1_TP_HIGH : RW; bitpos: [15:7]; default: 0;
 *  ADC1 Two Point calibration high point. Only valid if EFUSE_RD_BLK3_PART_RESERVE
 */
#define EFUSE_ADC1_TP_HIGH    0x000001FFU
#define EFUSE_ADC1_TP_HIGH_M  (EFUSE_ADC1_TP_HIGH_V << EFUSE_ADC1_TP_HIGH_S)
#define EFUSE_ADC1_TP_HIGH_V  0x000001FFU
#define EFUSE_ADC1_TP_HIGH_S  7
/** EFUSE_ADC2_TP_LOW : RW; bitpos: [22:16]; default: 0;
 *  ADC2 Two Point calibration low point. Only valid if EFUSE_RD_BLK3_PART_RESERVE
 */
#define EFUSE_ADC2_TP_LOW    0x0000007FU
#define EFUSE_ADC2_TP_LOW_M  (EFUSE_ADC2_TP_LOW_V << EFUSE_ADC2_TP_LOW_S)
#define EFUSE_ADC2_TP_LOW_V  0x0000007FU
#define EFUSE_ADC2_TP_LOW_S  16
/** EFUSE_ADC2_TP_HIGH : RW; bitpos: [31:23]; default: 0;
 *  ADC2 Two Point calibration high point. Only valid if EFUSE_RD_BLK3_PART_RESERVE
 */
#define EFUSE_ADC2_TP_HIGH    0x000001FFU
#define EFUSE_ADC2_TP_HIGH_M  (EFUSE_ADC2_TP_HIGH_V << EFUSE_ADC2_TP_HIGH_S)
#define EFUSE_ADC2_TP_HIGH_V  0x000001FFU
#define EFUSE_ADC2_TP_HIGH_S  23

/** EFUSE_BLK3_WDATA4_REG register */
#define EFUSE_BLK3_WDATA4_REG (DR_REG_EFUSE_BASE + 0xe8)
/** EFUSE_SECURE_VERSION : R; bitpos: [31:0]; default: 0;
 *  Secure version for anti-rollback
 */
#define EFUSE_SECURE_VERSION    0xFFFFFFFFU
#define EFUSE_SECURE_VERSION_M  (EFUSE_SECURE_VERSION_V << EFUSE_SECURE_VERSION_S)
#define EFUSE_SECURE_VERSION_V  0xFFFFFFFFU
#define EFUSE_SECURE_VERSION_S  0

/** EFUSE_BLK3_WDATA5_REG register */
#define EFUSE_BLK3_WDATA5_REG (DR_REG_EFUSE_BASE + 0xec)
/** EFUSE_BLK3_DIN5 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK3
 */
#define EFUSE_BLK3_DIN5    0xFFFFFFFFU
#define EFUSE_BLK3_DIN5_M  (EFUSE_BLK3_DIN5_V << EFUSE_BLK3_DIN5_S)
#define EFUSE_BLK3_DIN5_V  0xFFFFFFFFU
#define EFUSE_BLK3_DIN5_S  0

/** EFUSE_BLK3_WDATA6_REG register */
#define EFUSE_BLK3_WDATA6_REG (DR_REG_EFUSE_BASE + 0xf0)
/** EFUSE_BLK3_DIN6 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK3
 */
#define EFUSE_BLK3_DIN6    0xFFFFFFFFU
#define EFUSE_BLK3_DIN6_M  (EFUSE_BLK3_DIN6_V << EFUSE_BLK3_DIN6_S)
#define EFUSE_BLK3_DIN6_V  0xFFFFFFFFU
#define EFUSE_BLK3_DIN6_S  0

/** EFUSE_BLK3_WDATA7_REG register */
#define EFUSE_BLK3_WDATA7_REG (DR_REG_EFUSE_BASE + 0xf4)
/** EFUSE_BLK3_DIN7 : RW; bitpos: [31:0]; default: 0;
 *  program for BLOCK3
 */
#define EFUSE_BLK3_DIN7    0xFFFFFFFFU
#define EFUSE_BLK3_DIN7_M  (EFUSE_BLK3_DIN7_V << EFUSE_BLK3_DIN7_S)
#define EFUSE_BLK3_DIN7_V  0xFFFFFFFFU
#define EFUSE_BLK3_DIN7_S  0

/** EFUSE_CLK_REG register */
#define EFUSE_CLK_REG (DR_REG_EFUSE_BASE + 0xf8)
/** EFUSE_CLK_SEL0 : RW; bitpos: [7:0]; default: 82;
 *  efuse timing configure
 */
#define EFUSE_CLK_SEL0    0x000000FFU
#define EFUSE_CLK_SEL0_M  (EFUSE_CLK_SEL0_V << EFUSE_CLK_SEL0_S)
#define EFUSE_CLK_SEL0_V  0x000000FFU
#define EFUSE_CLK_SEL0_S  0
/** EFUSE_CLK_SEL1 : RW; bitpos: [15:8]; default: 64;
 *  efuse timing configure
 */
#define EFUSE_CLK_SEL1    0x000000FFU
#define EFUSE_CLK_SEL1_M  (EFUSE_CLK_SEL1_V << EFUSE_CLK_SEL1_S)
#define EFUSE_CLK_SEL1_V  0x000000FFU
#define EFUSE_CLK_SEL1_S  8
/** EFUSE_CLK_EN : RW; bitpos: [16]; default: 0; */
#define EFUSE_CLK_EN    (BIT(16))
#define EFUSE_CLK_EN_M  (EFUSE_CLK_EN_V << EFUSE_CLK_EN_S)
#define EFUSE_CLK_EN_V  0x00000001U
#define EFUSE_CLK_EN_S  16

/** EFUSE_CONF_REG register */
#define EFUSE_CONF_REG (DR_REG_EFUSE_BASE + 0xfc)
/** EFUSE_OP_CODE : RW; bitpos: [15:0]; default: 0;
 *  efuse operation code
 */
#define EFUSE_OP_CODE    0x0000FFFFU
#define EFUSE_OP_CODE_M  (EFUSE_OP_CODE_V << EFUSE_OP_CODE_S)
#define EFUSE_OP_CODE_V  0x0000FFFFU
#define EFUSE_OP_CODE_S  0
/** EFUSE_FORCE_NO_WR_RD_DIS : RW; bitpos: [16]; default: 1; */
#define EFUSE_FORCE_NO_WR_RD_DIS    (BIT(16))
#define EFUSE_FORCE_NO_WR_RD_DIS_M  (EFUSE_FORCE_NO_WR_RD_DIS_V << EFUSE_FORCE_NO_WR_RD_DIS_S)
#define EFUSE_FORCE_NO_WR_RD_DIS_V  0x00000001U
#define EFUSE_FORCE_NO_WR_RD_DIS_S  16

/** EFUSE_STATUS_REG register */
#define EFUSE_STATUS_REG (DR_REG_EFUSE_BASE + 0x100)
/** EFUSE_DEBUG : R; bitpos: [31:0]; default: 0; */
#define EFUSE_DEBUG    0xFFFFFFFFU
#define EFUSE_DEBUG_M  (EFUSE_DEBUG_V << EFUSE_DEBUG_S)
#define EFUSE_DEBUG_V  0xFFFFFFFFU
#define EFUSE_DEBUG_S  0

/** EFUSE_CMD_REG register */
#define EFUSE_CMD_REG (DR_REG_EFUSE_BASE + 0x104)
/** EFUSE_READ_CMD : RW; bitpos: [0]; default: 0;
 *  command for read
 */
#define EFUSE_READ_CMD    (BIT(0))
#define EFUSE_READ_CMD_M  (EFUSE_READ_CMD_V << EFUSE_READ_CMD_S)
#define EFUSE_READ_CMD_V  0x00000001U
#define EFUSE_READ_CMD_S  0
/** EFUSE_PGM_CMD : RW; bitpos: [1]; default: 0;
 *  command for program
 */
#define EFUSE_PGM_CMD    (BIT(1))
#define EFUSE_PGM_CMD_M  (EFUSE_PGM_CMD_V << EFUSE_PGM_CMD_S)
#define EFUSE_PGM_CMD_V  0x00000001U
#define EFUSE_PGM_CMD_S  1

/** EFUSE_INT_RAW_REG register */
#define EFUSE_INT_RAW_REG (DR_REG_EFUSE_BASE + 0x108)
/** EFUSE_READ_DONE_INT_RAW : R; bitpos: [0]; default: 0;
 *  read done interrupt raw status
 */
#define EFUSE_READ_DONE_INT_RAW    (BIT(0))
#define EFUSE_READ_DONE_INT_RAW_M  (EFUSE_READ_DONE_INT_RAW_V << EFUSE_READ_DONE_INT_RAW_S)
#define EFUSE_READ_DONE_INT_RAW_V  0x00000001U
#define EFUSE_READ_DONE_INT_RAW_S  0
/** EFUSE_PGM_DONE_INT_RAW : R; bitpos: [1]; default: 0;
 *  program done interrupt raw status
 */
#define EFUSE_PGM_DONE_INT_RAW    (BIT(1))
#define EFUSE_PGM_DONE_INT_RAW_M  (EFUSE_PGM_DONE_INT_RAW_V << EFUSE_PGM_DONE_INT_RAW_S)
#define EFUSE_PGM_DONE_INT_RAW_V  0x00000001U
#define EFUSE_PGM_DONE_INT_RAW_S  1

/** EFUSE_INT_ST_REG register */
#define EFUSE_INT_ST_REG (DR_REG_EFUSE_BASE + 0x10c)
/** EFUSE_READ_DONE_INT_ST : R; bitpos: [0]; default: 0;
 *  read done interrupt status
 */
#define EFUSE_READ_DONE_INT_ST    (BIT(0))
#define EFUSE_READ_DONE_INT_ST_M  (EFUSE_READ_DONE_INT_ST_V << EFUSE_READ_DONE_INT_ST_S)
#define EFUSE_READ_DONE_INT_ST_V  0x00000001U
#define EFUSE_READ_DONE_INT_ST_S  0
/** EFUSE_PGM_DONE_INT_ST : R; bitpos: [1]; default: 0;
 *  program done interrupt status
 */
#define EFUSE_PGM_DONE_INT_ST    (BIT(1))
#define EFUSE_PGM_DONE_INT_ST_M  (EFUSE_PGM_DONE_INT_ST_V << EFUSE_PGM_DONE_INT_ST_S)
#define EFUSE_PGM_DONE_INT_ST_V  0x00000001U
#define EFUSE_PGM_DONE_INT_ST_S  1

/** EFUSE_INT_ENA_REG register */
#define EFUSE_INT_ENA_REG (DR_REG_EFUSE_BASE + 0x110)
/** EFUSE_READ_DONE_INT_ENA : RW; bitpos: [0]; default: 0;
 *  read done interrupt enable
 */
#define EFUSE_READ_DONE_INT_ENA    (BIT(0))
#define EFUSE_READ_DONE_INT_ENA_M  (EFUSE_READ_DONE_INT_ENA_V << EFUSE_READ_DONE_INT_ENA_S)
#define EFUSE_READ_DONE_INT_ENA_V  0x00000001U
#define EFUSE_READ_DONE_INT_ENA_S  0
/** EFUSE_PGM_DONE_INT_ENA : RW; bitpos: [1]; default: 0;
 *  program done interrupt enable
 */
#define EFUSE_PGM_DONE_INT_ENA    (BIT(1))
#define EFUSE_PGM_DONE_INT_ENA_M  (EFUSE_PGM_DONE_INT_ENA_V << EFUSE_PGM_DONE_INT_ENA_S)
#define EFUSE_PGM_DONE_INT_ENA_V  0x00000001U
#define EFUSE_PGM_DONE_INT_ENA_S  1

/** EFUSE_INT_CLR_REG register */
#define EFUSE_INT_CLR_REG (DR_REG_EFUSE_BASE + 0x114)
/** EFUSE_READ_DONE_INT_CLR : W; bitpos: [0]; default: 0;
 *  read done interrupt clear
 */
#define EFUSE_READ_DONE_INT_CLR    (BIT(0))
#define EFUSE_READ_DONE_INT_CLR_M  (EFUSE_READ_DONE_INT_CLR_V << EFUSE_READ_DONE_INT_CLR_S)
#define EFUSE_READ_DONE_INT_CLR_V  0x00000001U
#define EFUSE_READ_DONE_INT_CLR_S  0
/** EFUSE_PGM_DONE_INT_CLR : W; bitpos: [1]; default: 0;
 *  program done interrupt clear
 */
#define EFUSE_PGM_DONE_INT_CLR    (BIT(1))
#define EFUSE_PGM_DONE_INT_CLR_M  (EFUSE_PGM_DONE_INT_CLR_V << EFUSE_PGM_DONE_INT_CLR_S)
#define EFUSE_PGM_DONE_INT_CLR_V  0x00000001U
#define EFUSE_PGM_DONE_INT_CLR_S  1

/** EFUSE_DAC_CONF_REG register */
#define EFUSE_DAC_CONF_REG (DR_REG_EFUSE_BASE + 0x118)
/** EFUSE_DAC_CLK_DIV : RW; bitpos: [7:0]; default: 40;
 *  efuse timing configure
 */
#define EFUSE_DAC_CLK_DIV    0x000000FFU
#define EFUSE_DAC_CLK_DIV_M  (EFUSE_DAC_CLK_DIV_V << EFUSE_DAC_CLK_DIV_S)
#define EFUSE_DAC_CLK_DIV_V  0x000000FFU
#define EFUSE_DAC_CLK_DIV_S  0
/** EFUSE_DAC_CLK_PAD_SEL : RW; bitpos: [8]; default: 0; */
#define EFUSE_DAC_CLK_PAD_SEL    (BIT(8))
#define EFUSE_DAC_CLK_PAD_SEL_M  (EFUSE_DAC_CLK_PAD_SEL_V << EFUSE_DAC_CLK_PAD_SEL_S)
#define EFUSE_DAC_CLK_PAD_SEL_V  0x00000001U
#define EFUSE_DAC_CLK_PAD_SEL_S  8

/** EFUSE_DEC_STATUS_REG register */
#define EFUSE_DEC_STATUS_REG (DR_REG_EFUSE_BASE + 0x11c)
/** EFUSE_DEC_WARNINGS : R; bitpos: [11:0]; default: 0;
 *  the decode result of 3/4 coding scheme has warning
 */
#define EFUSE_DEC_WARNINGS    0x00000FFFU
#define EFUSE_DEC_WARNINGS_M  (EFUSE_DEC_WARNINGS_V << EFUSE_DEC_WARNINGS_S)
#define EFUSE_DEC_WARNINGS_V  0x00000FFFU
#define EFUSE_DEC_WARNINGS_S  0

/** EFUSE_DATE_REG register */
#define EFUSE_DATE_REG (DR_REG_EFUSE_BASE + 0x1fc)
/** EFUSE_DATE : RW; bitpos: [31:0]; default: 369370624; */
#define EFUSE_DATE    0xFFFFFFFFU
#define EFUSE_DATE_M  (EFUSE_DATE_V << EFUSE_DATE_S)
#define EFUSE_DATE_V  0xFFFFFFFFU
#define EFUSE_DATE_S  0

#ifdef __cplusplus
}
#endif
