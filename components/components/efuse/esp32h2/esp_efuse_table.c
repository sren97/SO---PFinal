/*
 * SPDX-FileCopyrightText: 2017-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include "esp_efuse.h"
#include <assert.h>
#include "esp_efuse_table.h"

// md5_digest_table 1dc5045e8a74c32825696ca314128499
// This file was generated from the file esp_efuse_table.csv. DO NOT CHANGE THIS FILE MANUALLY.
// If you want to change some fields, you need to change esp_efuse_table.csv file
// then run `efuse_common_table` or `efuse_custom_table` command it will generate this file.
// To show efuse_table run the command 'show_efuse_table'.

static const esp_efuse_desc_t WR_DIS[] = {
    {EFUSE_BLK0, 0, 32}, 	 // [] Disable programming of individual eFuses,
};

static const esp_efuse_desc_t WR_DIS_RD_DIS[] = {
    {EFUSE_BLK0, 0, 1}, 	 // [] wr_dis of RD_DIS,
};

static const esp_efuse_desc_t WR_DIS_DIS_ICACHE[] = {
    {EFUSE_BLK0, 2, 1}, 	 // [] wr_dis of DIS_ICACHE,
};

static const esp_efuse_desc_t WR_DIS_DIS_USB_JTAG[] = {
    {EFUSE_BLK0, 2, 1}, 	 // [] wr_dis of DIS_USB_JTAG,
};

static const esp_efuse_desc_t WR_DIS_POWERGLITCH_EN[] = {
    {EFUSE_BLK0, 2, 1}, 	 // [] wr_dis of POWERGLITCH_EN,
};

static const esp_efuse_desc_t WR_DIS_DIS_FORCE_DOWNLOAD[] = {
    {EFUSE_BLK0, 2, 1}, 	 // [] wr_dis of DIS_FORCE_DOWNLOAD,
};

static const esp_efuse_desc_t WR_DIS_SPI_DOWNLOAD_MSPI_DIS[] = {
    {EFUSE_BLK0, 2, 1}, 	 // [] wr_dis of SPI_DOWNLOAD_MSPI_DIS,
};

static const esp_efuse_desc_t WR_DIS_DIS_TWAI[] = {
    {EFUSE_BLK0, 2, 1}, 	 // [WR_DIS.DIS_CAN] wr_dis of DIS_TWAI,
};

static const esp_efuse_desc_t WR_DIS_JTAG_SEL_ENABLE[] = {
    {EFUSE_BLK0, 2, 1}, 	 // [] wr_dis of JTAG_SEL_ENABLE,
};

static const esp_efuse_desc_t WR_DIS_DIS_PAD_JTAG[] = {
    {EFUSE_BLK0, 2, 1}, 	 // [] wr_dis of DIS_PAD_JTAG,
};

static const esp_efuse_desc_t WR_DIS_DIS_DOWNLOAD_MANUAL_ENCRYPT[] = {
    {EFUSE_BLK0, 2, 1}, 	 // [] wr_dis of DIS_DOWNLOAD_MANUAL_ENCRYPT,
};

static const esp_efuse_desc_t WR_DIS_POWERGLITCH_EN1[] = {
    {EFUSE_BLK0, 2, 1}, 	 // [] wr_dis of POWERGLITCH_EN1,
};

static const esp_efuse_desc_t WR_DIS_WDT_DELAY_SEL[] = {
    {EFUSE_BLK0, 3, 1}, 	 // [] wr_dis of WDT_DELAY_SEL,
};

static const esp_efuse_desc_t WR_DIS_SPI_BOOT_CRYPT_CNT[] = {
    {EFUSE_BLK0, 4, 1}, 	 // [] wr_dis of SPI_BOOT_CRYPT_CNT,
};

static const esp_efuse_desc_t WR_DIS_SECURE_BOOT_KEY_REVOKE0[] = {
    {EFUSE_BLK0, 5, 1}, 	 // [] wr_dis of SECURE_BOOT_KEY_REVOKE0,
};

static const esp_efuse_desc_t WR_DIS_SECURE_BOOT_KEY_REVOKE1[] = {
    {EFUSE_BLK0, 6, 1}, 	 // [] wr_dis of SECURE_BOOT_KEY_REVOKE1,
};

static const esp_efuse_desc_t WR_DIS_SECURE_BOOT_KEY_REVOKE2[] = {
    {EFUSE_BLK0, 7, 1}, 	 // [] wr_dis of SECURE_BOOT_KEY_REVOKE2,
};

static const esp_efuse_desc_t WR_DIS_KEY_PURPOSE_0[] = {
    {EFUSE_BLK0, 8, 1}, 	 // [WR_DIS.KEY0_PURPOSE] wr_dis of KEY_PURPOSE_0,
};

static const esp_efuse_desc_t WR_DIS_KEY_PURPOSE_1[] = {
    {EFUSE_BLK0, 9, 1}, 	 // [WR_DIS.KEY1_PURPOSE] wr_dis of KEY_PURPOSE_1,
};

static const esp_efuse_desc_t WR_DIS_KEY_PURPOSE_2[] = {
    {EFUSE_BLK0, 10, 1}, 	 // [WR_DIS.KEY2_PURPOSE] wr_dis of KEY_PURPOSE_2,
};

static const esp_efuse_desc_t WR_DIS_KEY_PURPOSE_3[] = {
    {EFUSE_BLK0, 11, 1}, 	 // [WR_DIS.KEY3_PURPOSE] wr_dis of KEY_PURPOSE_3,
};

static const esp_efuse_desc_t WR_DIS_KEY_PURPOSE_4[] = {
    {EFUSE_BLK0, 12, 1}, 	 // [WR_DIS.KEY4_PURPOSE] wr_dis of KEY_PURPOSE_4,
};

static const esp_efuse_desc_t WR_DIS_KEY_PURPOSE_5[] = {
    {EFUSE_BLK0, 13, 1}, 	 // [WR_DIS.KEY5_PURPOSE] wr_dis of KEY_PURPOSE_5,
};

static const esp_efuse_desc_t WR_DIS_XTS_DPA_PSEUDO_LEVEL[] = {
    {EFUSE_BLK0, 14, 1}, 	 // [] wr_dis of XTS_DPA_PSEUDO_LEVEL,
};

static const esp_efuse_desc_t WR_DIS_SEC_DPA_LEVEL[] = {
    {EFUSE_BLK0, 14, 1}, 	 // [] wr_dis of SEC_DPA_LEVEL,
};

static const esp_efuse_desc_t WR_DIS_CRYPT_DPA_ENABLE[] = {
    {EFUSE_BLK0, 14, 1}, 	 // [] wr_dis of CRYPT_DPA_ENABLE,
};

static const esp_efuse_desc_t WR_DIS_SECURE_BOOT_EN[] = {
    {EFUSE_BLK0, 15, 1}, 	 // [] wr_dis of SECURE_BOOT_EN,
};

static const esp_efuse_desc_t WR_DIS_SECURE_BOOT_AGGRESSIVE_REVOKE[] = {
    {EFUSE_BLK0, 16, 1}, 	 // [] wr_dis of SECURE_BOOT_AGGRESSIVE_REVOKE,
};

static const esp_efuse_desc_t WR_DIS_ECDSA_CURVE_MODE[] = {
    {EFUSE_BLK0, 17, 1}, 	 // [] wr_dis of ECDSA_CURVE_MODE,
};

static const esp_efuse_desc_t WR_DIS_ECC_FORCE_CONST_TIME[] = {
    {EFUSE_BLK0, 17, 1}, 	 // [] wr_dis of ECC_FORCE_CONST_TIME,
};

static const esp_efuse_desc_t WR_DIS_FLASH_TPUW[] = {
    {EFUSE_BLK0, 18, 1}, 	 // [] wr_dis of FLASH_TPUW,
};

static const esp_efuse_desc_t WR_DIS_DIS_DOWNLOAD_MODE[] = {
    {EFUSE_BLK0, 18, 1}, 	 // [] wr_dis of DIS_DOWNLOAD_MODE,
};

static const esp_efuse_desc_t WR_DIS_DIS_DIRECT_BOOT[] = {
    {EFUSE_BLK0, 18, 1}, 	 // [] wr_dis of DIS_DIRECT_BOOT,
};

static const esp_efuse_desc_t WR_DIS_DIS_USB_SERIAL_JTAG_ROM_PRINT[] = {
    {EFUSE_BLK0, 18, 1}, 	 // [WR_DIS.DIS_USB_PRINT] wr_dis of DIS_USB_SERIAL_JTAG_ROM_PRINT,
};

static const esp_efuse_desc_t WR_DIS_DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE[] = {
    {EFUSE_BLK0, 18, 1}, 	 // [] wr_dis of DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE,
};

static const esp_efuse_desc_t WR_DIS_ENABLE_SECURITY_DOWNLOAD[] = {
    {EFUSE_BLK0, 18, 1}, 	 // [] wr_dis of ENABLE_SECURITY_DOWNLOAD,
};

static const esp_efuse_desc_t WR_DIS_UART_PRINT_CONTROL[] = {
    {EFUSE_BLK0, 18, 1}, 	 // [] wr_dis of UART_PRINT_CONTROL,
};

static const esp_efuse_desc_t WR_DIS_FORCE_SEND_RESUME[] = {
    {EFUSE_BLK0, 18, 1}, 	 // [] wr_dis of FORCE_SEND_RESUME,
};

static const esp_efuse_desc_t WR_DIS_SECURE_VERSION[] = {
    {EFUSE_BLK0, 18, 1}, 	 // [] wr_dis of SECURE_VERSION,
};

static const esp_efuse_desc_t WR_DIS_SECURE_BOOT_DISABLE_FAST_WAKE[] = {
    {EFUSE_BLK0, 18, 1}, 	 // [] wr_dis of SECURE_BOOT_DISABLE_FAST_WAKE,
};

static const esp_efuse_desc_t WR_DIS_HYS_EN_PAD0[] = {
    {EFUSE_BLK0, 19, 1}, 	 // [] wr_dis of HYS_EN_PAD0,
};

static const esp_efuse_desc_t WR_DIS_HYS_EN_PAD1[] = {
    {EFUSE_BLK0, 19, 1}, 	 // [] wr_dis of HYS_EN_PAD1,
};

static const esp_efuse_desc_t WR_DIS_BLK1[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of BLOCK1,
};

static const esp_efuse_desc_t WR_DIS_MAC[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [WR_DIS.MAC_FACTORY] wr_dis of MAC,
};

static const esp_efuse_desc_t WR_DIS_MAC_EXT[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of MAC_EXT,
};

static const esp_efuse_desc_t WR_DIS_RXIQ_VERSION[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of RXIQ_VERSION,
};

static const esp_efuse_desc_t WR_DIS_RXIQ_0[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of RXIQ_0,
};

static const esp_efuse_desc_t WR_DIS_RXIQ_1[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of RXIQ_1,
};

static const esp_efuse_desc_t WR_DIS_ACTIVE_HP_DBIAS[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of ACTIVE_HP_DBIAS,
};

static const esp_efuse_desc_t WR_DIS_ACTIVE_LP_DBIAS[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of ACTIVE_LP_DBIAS,
};

static const esp_efuse_desc_t WR_DIS_DSLP_DBIAS[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of DSLP_DBIAS,
};

static const esp_efuse_desc_t WR_DIS_DBIAS_VOL_GAP[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of DBIAS_VOL_GAP,
};

static const esp_efuse_desc_t WR_DIS_WAFER_VERSION_MINOR[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of WAFER_VERSION_MINOR,
};

static const esp_efuse_desc_t WR_DIS_WAFER_VERSION_MAJOR[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of WAFER_VERSION_MAJOR,
};

static const esp_efuse_desc_t WR_DIS_DISABLE_WAFER_VERSION_MAJOR[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of DISABLE_WAFER_VERSION_MAJOR,
};

static const esp_efuse_desc_t WR_DIS_FLASH_CAP[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of FLASH_CAP,
};

static const esp_efuse_desc_t WR_DIS_FLASH_TEMP[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of FLASH_TEMP,
};

static const esp_efuse_desc_t WR_DIS_FLASH_VENDOR[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of FLASH_VENDOR,
};

static const esp_efuse_desc_t WR_DIS_PKG_VERSION[] = {
    {EFUSE_BLK0, 20, 1}, 	 // [] wr_dis of PKG_VERSION,
};

static const esp_efuse_desc_t WR_DIS_SYS_DATA_PART1[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of BLOCK2,
};

static const esp_efuse_desc_t WR_DIS_OPTIONAL_UNIQUE_ID[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of OPTIONAL_UNIQUE_ID,
};

static const esp_efuse_desc_t WR_DIS_BLK_VERSION_MINOR[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of BLK_VERSION_MINOR,
};

static const esp_efuse_desc_t WR_DIS_BLK_VERSION_MAJOR[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of BLK_VERSION_MAJOR,
};

static const esp_efuse_desc_t WR_DIS_DISABLE_BLK_VERSION_MAJOR[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of DISABLE_BLK_VERSION_MAJOR,
};

static const esp_efuse_desc_t WR_DIS_TEMP_CALIB[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of TEMP_CALIB,
};

static const esp_efuse_desc_t WR_DIS_ADC1_AVE_INITCODE_ATTEN0[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_AVE_INITCODE_ATTEN0,
};

static const esp_efuse_desc_t WR_DIS_ADC1_AVE_INITCODE_ATTEN1[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_AVE_INITCODE_ATTEN1,
};

static const esp_efuse_desc_t WR_DIS_ADC1_AVE_INITCODE_ATTEN2[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_AVE_INITCODE_ATTEN2,
};

static const esp_efuse_desc_t WR_DIS_ADC1_AVE_INITCODE_ATTEN3[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_AVE_INITCODE_ATTEN3,
};

static const esp_efuse_desc_t WR_DIS_ADC1_HI_DOUT_ATTEN0[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_HI_DOUT_ATTEN0,
};

static const esp_efuse_desc_t WR_DIS_ADC1_HI_DOUT_ATTEN1[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_HI_DOUT_ATTEN1,
};

static const esp_efuse_desc_t WR_DIS_ADC1_HI_DOUT_ATTEN2[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_HI_DOUT_ATTEN2,
};

static const esp_efuse_desc_t WR_DIS_ADC1_HI_DOUT_ATTEN3[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_HI_DOUT_ATTEN3,
};

static const esp_efuse_desc_t WR_DIS_ADC1_CH0_ATTEN0_INITCODE_DIFF[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_CH0_ATTEN0_INITCODE_DIFF,
};

static const esp_efuse_desc_t WR_DIS_ADC1_CH1_ATTEN0_INITCODE_DIFF[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_CH1_ATTEN0_INITCODE_DIFF,
};

static const esp_efuse_desc_t WR_DIS_ADC1_CH2_ATTEN0_INITCODE_DIFF[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_CH2_ATTEN0_INITCODE_DIFF,
};

static const esp_efuse_desc_t WR_DIS_ADC1_CH3_ATTEN0_INITCODE_DIFF[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_CH3_ATTEN0_INITCODE_DIFF,
};

static const esp_efuse_desc_t WR_DIS_ADC1_CH4_ATTEN0_INITCODE_DIFF[] = {
    {EFUSE_BLK0, 21, 1}, 	 // [] wr_dis of ADC1_CH4_ATTEN0_INITCODE_DIFF,
};

static const esp_efuse_desc_t WR_DIS_BLOCK_USR_DATA[] = {
    {EFUSE_BLK0, 22, 1}, 	 // [WR_DIS.USER_DATA] wr_dis of BLOCK_USR_DATA,
};

static const esp_efuse_desc_t WR_DIS_CUSTOM_MAC[] = {
    {EFUSE_BLK0, 22, 1}, 	 // [WR_DIS.MAC_CUSTOM WR_DIS.USER_DATA_MAC_CUSTOM] wr_dis of CUSTOM_MAC,
};

static const esp_efuse_desc_t WR_DIS_BLOCK_KEY0[] = {
    {EFUSE_BLK0, 23, 1}, 	 // [WR_DIS.KEY0] wr_dis of BLOCK_KEY0,
};

static const esp_efuse_desc_t WR_DIS_BLOCK_KEY1[] = {
    {EFUSE_BLK0, 24, 1}, 	 // [WR_DIS.KEY1] wr_dis of BLOCK_KEY1,
};

static const esp_efuse_desc_t WR_DIS_BLOCK_KEY2[] = {
    {EFUSE_BLK0, 25, 1}, 	 // [WR_DIS.KEY2] wr_dis of BLOCK_KEY2,
};

static const esp_efuse_desc_t WR_DIS_BLOCK_KEY3[] = {
    {EFUSE_BLK0, 26, 1}, 	 // [WR_DIS.KEY3] wr_dis of BLOCK_KEY3,
};

static const esp_efuse_desc_t WR_DIS_BLOCK_KEY4[] = {
    {EFUSE_BLK0, 27, 1}, 	 // [WR_DIS.KEY4] wr_dis of BLOCK_KEY4,
};

static const esp_efuse_desc_t WR_DIS_BLOCK_KEY5[] = {
    {EFUSE_BLK0, 28, 1}, 	 // [WR_DIS.KEY5] wr_dis of BLOCK_KEY5,
};

static const esp_efuse_desc_t WR_DIS_BLOCK_SYS_DATA2[] = {
    {EFUSE_BLK0, 29, 1}, 	 // [WR_DIS.SYS_DATA_PART2] wr_dis of BLOCK_SYS_DATA2,
};

static const esp_efuse_desc_t WR_DIS_USB_EXCHG_PINS[] = {
    {EFUSE_BLK0, 30, 1}, 	 // [] wr_dis of USB_EXCHG_PINS,
};

static const esp_efuse_desc_t WR_DIS_VDD_SPI_AS_GPIO[] = {
    {EFUSE_BLK0, 30, 1}, 	 // [] wr_dis of VDD_SPI_AS_GPIO,
};

static const esp_efuse_desc_t WR_DIS_SOFT_DIS_JTAG[] = {
    {EFUSE_BLK0, 31, 1}, 	 // [] wr_dis of SOFT_DIS_JTAG,
};

static const esp_efuse_desc_t RD_DIS[] = {
    {EFUSE_BLK0, 32, 7}, 	 // [] Disable reading from BlOCK4-10,
};

static const esp_efuse_desc_t RD_DIS_BLOCK_KEY0[] = {
    {EFUSE_BLK0, 32, 1}, 	 // [RD_DIS.KEY0] rd_dis of BLOCK_KEY0,
};

static const esp_efuse_desc_t RD_DIS_BLOCK_KEY1[] = {
    {EFUSE_BLK0, 33, 1}, 	 // [RD_DIS.KEY1] rd_dis of BLOCK_KEY1,
};

static const esp_efuse_desc_t RD_DIS_BLOCK_KEY2[] = {
    {EFUSE_BLK0, 34, 1}, 	 // [RD_DIS.KEY2] rd_dis of BLOCK_KEY2,
};

static const esp_efuse_desc_t RD_DIS_BLOCK_KEY3[] = {
    {EFUSE_BLK0, 35, 1}, 	 // [RD_DIS.KEY3] rd_dis of BLOCK_KEY3,
};

static const esp_efuse_desc_t RD_DIS_BLOCK_KEY4[] = {
    {EFUSE_BLK0, 36, 1}, 	 // [RD_DIS.KEY4] rd_dis of BLOCK_KEY4,
};

static const esp_efuse_desc_t RD_DIS_BLOCK_KEY5[] = {
    {EFUSE_BLK0, 37, 1}, 	 // [RD_DIS.KEY5] rd_dis of BLOCK_KEY5,
};

static const esp_efuse_desc_t RD_DIS_BLOCK_SYS_DATA2[] = {
    {EFUSE_BLK0, 38, 1}, 	 // [RD_DIS.SYS_DATA_PART2] rd_dis of BLOCK_SYS_DATA2,
};

static const esp_efuse_desc_t DIS_ICACHE[] = {
    {EFUSE_BLK0, 40, 1}, 	 // [] Represents whether icache is disabled or enabled. 1: disabled. 0: enabled,
};

static const esp_efuse_desc_t DIS_USB_JTAG[] = {
    {EFUSE_BLK0, 41, 1}, 	 // [] Represents whether the function of usb switch to jtag is disabled or enabled. 1: disabled. 0: enabled,
};

static const esp_efuse_desc_t POWERGLITCH_EN[] = {
    {EFUSE_BLK0, 42, 1}, 	 // [] Represents whether power glitch function is enabled. 1: enabled. 0: disabled,
};

static const esp_efuse_desc_t DIS_FORCE_DOWNLOAD[] = {
    {EFUSE_BLK0, 44, 1}, 	 // [] Represents whether the function that forces chip into download mode is disabled or enabled. 1: disabled. 0: enabled,
};

static const esp_efuse_desc_t SPI_DOWNLOAD_MSPI_DIS[] = {
    {EFUSE_BLK0, 45, 1}, 	 // [] Represents whether SPI0 controller during boot_mode_download is disabled or enabled. 1: disabled. 0: enabled,
};

static const esp_efuse_desc_t DIS_TWAI[] = {
    {EFUSE_BLK0, 46, 1}, 	 // [DIS_CAN] Represents whether TWAI function is disabled or enabled. 1: disabled. 0: enabled,
};

static const esp_efuse_desc_t JTAG_SEL_ENABLE[] = {
    {EFUSE_BLK0, 47, 1}, 	 // [] Set this bit to enable selection between usb_to_jtag and pad_to_jtag through strapping gpio25 when both EFUSE_DIS_PAD_JTAG and EFUSE_DIS_USB_JTAG are equal to 0,
};

static const esp_efuse_desc_t SOFT_DIS_JTAG[] = {
    {EFUSE_BLK0, 48, 3}, 	 // [] Represents whether JTAG is disabled in soft way. Odd number: disabled. Even number: enabled,
};

static const esp_efuse_desc_t DIS_PAD_JTAG[] = {
    {EFUSE_BLK0, 51, 1}, 	 // [] Represents whether JTAG is disabled in the hard way(permanently). 1: disabled. 0: enabled,
};

static const esp_efuse_desc_t DIS_DOWNLOAD_MANUAL_ENCRYPT[] = {
    {EFUSE_BLK0, 52, 1}, 	 // [] Represents whether flash encrypt function is disabled or enabled(except in SPI boot mode). 1: disabled. 0: enabled,
};

static const esp_efuse_desc_t USB_EXCHG_PINS[] = {
    {EFUSE_BLK0, 57, 1}, 	 // [] Represents whether the D+ and D- pins is exchanged. 1: exchanged. 0: not exchanged,
};

static const esp_efuse_desc_t VDD_SPI_AS_GPIO[] = {
    {EFUSE_BLK0, 58, 1}, 	 // [] Represents whether vdd spi pin is functioned as gpio. 1: functioned. 0: not functioned,
};

static const esp_efuse_desc_t ECDSA_CURVE_MODE[] = {
    {EFUSE_BLK0, 59, 2}, 	 // [] Configures the curve of ECDSA calculation: 0: only enable P256. 1: only enable P192. 2: both enable P256 and P192. 3: only enable P256,
};

static const esp_efuse_desc_t ECC_FORCE_CONST_TIME[] = {
    {EFUSE_BLK0, 61, 1}, 	 // [] Set this bit to permanently turn on ECC const-time mode,
};

static const esp_efuse_desc_t XTS_DPA_PSEUDO_LEVEL[] = {
    {EFUSE_BLK0, 62, 2}, 	 // [] Set this bit to control the xts pseudo-round anti-dpa attack function: 0: controlled by register. 1-3: the higher the value is; the more pseudo-rounds are inserted to the xts-aes calculation,
};

static const esp_efuse_desc_t WDT_DELAY_SEL[] = {
    {EFUSE_BLK0, 80, 2}, 	 // [] Represents whether RTC watchdog timeout threshold is selected at startup. 1: selected. 0: not selected,
};

static const esp_efuse_desc_t SPI_BOOT_CRYPT_CNT[] = {
    {EFUSE_BLK0, 82, 3}, 	 // [] Enables flash encryption when 1 or 3 bits are set and disables otherwise {0: "Disable"; 1: "Enable"; 3: "Disable"; 7: "Enable"},
};

static const esp_efuse_desc_t SECURE_BOOT_KEY_REVOKE0[] = {
    {EFUSE_BLK0, 85, 1}, 	 // [] Revoke 1st secure boot key,
};

static const esp_efuse_desc_t SECURE_BOOT_KEY_REVOKE1[] = {
    {EFUSE_BLK0, 86, 1}, 	 // [] Revoke 2nd secure boot key,
};

static const esp_efuse_desc_t SECURE_BOOT_KEY_REVOKE2[] = {
    {EFUSE_BLK0, 87, 1}, 	 // [] Revoke 3rd secure boot key,
};

static const esp_efuse_desc_t KEY_PURPOSE_0[] = {
    {EFUSE_BLK0, 88, 4}, 	 // [KEY0_PURPOSE] Represents the purpose of Key0,
};

static const esp_efuse_desc_t KEY_PURPOSE_1[] = {
    {EFUSE_BLK0, 92, 4}, 	 // [KEY1_PURPOSE] Represents the purpose of Key1,
};

static const esp_efuse_desc_t KEY_PURPOSE_2[] = {
    {EFUSE_BLK0, 96, 4}, 	 // [KEY2_PURPOSE] Represents the purpose of Key2,
};

static const esp_efuse_desc_t KEY_PURPOSE_3[] = {
    {EFUSE_BLK0, 100, 4}, 	 // [KEY3_PURPOSE] Represents the purpose of Key3,
};

static const esp_efuse_desc_t KEY_PURPOSE_4[] = {
    {EFUSE_BLK0, 104, 4}, 	 // [KEY4_PURPOSE] Represents the purpose of Key4,
};

static const esp_efuse_desc_t KEY_PURPOSE_5[] = {
    {EFUSE_BLK0, 108, 4}, 	 // [KEY5_PURPOSE] Represents the purpose of Key5,
};

static const esp_efuse_desc_t SEC_DPA_LEVEL[] = {
    {EFUSE_BLK0, 112, 2}, 	 // [] Represents the spa secure level by configuring the clock random divide mode,
};

static const esp_efuse_desc_t CRYPT_DPA_ENABLE[] = {
    {EFUSE_BLK0, 115, 1}, 	 // [] Represents whether anti-dpa attack is enabled. 1:enabled. 0: disabled,
};

static const esp_efuse_desc_t SECURE_BOOT_EN[] = {
    {EFUSE_BLK0, 116, 1}, 	 // [] Represents whether secure boot is enabled or disabled. 1: enabled. 0: disabled,
};

static const esp_efuse_desc_t SECURE_BOOT_AGGRESSIVE_REVOKE[] = {
    {EFUSE_BLK0, 117, 1}, 	 // [] Represents whether revoking aggressive secure boot is enabled or disabled. 1: enabled. 0: disabled,
};

static const esp_efuse_desc_t POWERGLITCH_EN1[] = {
    {EFUSE_BLK0, 118, 5}, 	 // [] Set these bits to enable power glitch function when chip power on,
};

static const esp_efuse_desc_t FLASH_TPUW[] = {
    {EFUSE_BLK0, 124, 4}, 	 // [] Represents the flash waiting time after power-up; in unit of ms. When the value less than 15; the waiting time is the programmed value. Otherwise; the waiting time is 2 times the programmed value,
};

static const esp_efuse_desc_t DIS_DOWNLOAD_MODE[] = {
    {EFUSE_BLK0, 128, 1}, 	 // [] Represents whether Download mode is disabled or enabled. 1: disabled. 0: enabled,
};

static const esp_efuse_desc_t DIS_DIRECT_BOOT[] = {
    {EFUSE_BLK0, 129, 1}, 	 // [] Represents whether direct boot mode is disabled or enabled. 1: disabled. 0: enabled,
};

static const esp_efuse_desc_t DIS_USB_SERIAL_JTAG_ROM_PRINT[] = {
    {EFUSE_BLK0, 130, 1}, 	 // [DIS_USB_PRINT] Set this bit to disable USB-Serial-JTAG print during rom boot,
};

static const esp_efuse_desc_t DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE[] = {
    {EFUSE_BLK0, 132, 1}, 	 // [] Represents whether the USB-Serial-JTAG download function is disabled or enabled. 1: disabled. 0: enabled,
};

static const esp_efuse_desc_t ENABLE_SECURITY_DOWNLOAD[] = {
    {EFUSE_BLK0, 133, 1}, 	 // [] Represents whether security download is enabled or disabled. 1: enabled. 0: disabled,
};

static const esp_efuse_desc_t UART_PRINT_CONTROL[] = {
    {EFUSE_BLK0, 134, 2}, 	 // [] Set the default UARTboot message output mode {0: "Enable"; 1: "Enable when GPIO8 is low at reset"; 2: "Enable when GPIO8 is high at reset"; 3: "Disable"},
};

static const esp_efuse_desc_t FORCE_SEND_RESUME[] = {
    {EFUSE_BLK0, 136, 1}, 	 // [] Represents whether ROM code is forced to send a resume command during SPI boot. 1: forced. 0:not forced,
};

static const esp_efuse_desc_t SECURE_VERSION[] = {
    {EFUSE_BLK0, 137, 16}, 	 // [] Represents the version used by ESP-IDF anti-rollback feature,
};

static const esp_efuse_desc_t SECURE_BOOT_DISABLE_FAST_WAKE[] = {
    {EFUSE_BLK0, 153, 1}, 	 // [] Represents whether FAST VERIFY ON WAKE is disabled or enabled when Secure Boot is enabled. 1: disabled. 0: enabled,
};

static const esp_efuse_desc_t HYS_EN_PAD0[] = {
    {EFUSE_BLK0, 154, 6}, 	 // [] Set bits to enable hysteresis function of PAD0~5,
};

static const esp_efuse_desc_t HYS_EN_PAD1[] = {
    {EFUSE_BLK0, 160, 22}, 	 // [] Set bits to enable hysteresis function of PAD6~27,
};

static const esp_efuse_desc_t MAC[] = {
    {EFUSE_BLK1, 40, 8}, 	 // [MAC_FACTORY] MAC address,
    {EFUSE_BLK1, 32, 8}, 	 // [MAC_FACTORY] MAC address,
    {EFUSE_BLK1, 24, 8}, 	 // [MAC_FACTORY] MAC address,
    {EFUSE_BLK1, 16, 8}, 	 // [MAC_FACTORY] MAC address,
    {EFUSE_BLK1, 8, 8}, 	 // [MAC_FACTORY] MAC address,
    {EFUSE_BLK1, 0, 8}, 	 // [MAC_FACTORY] MAC address,
};

static const esp_efuse_desc_t MAC_EXT[] = {
    {EFUSE_BLK1, 56, 8}, 	 // [] Stores the extended bits of MAC address,
    {EFUSE_BLK1, 48, 8}, 	 // [] Stores the extended bits of MAC address,
};

static const esp_efuse_desc_t RXIQ_VERSION[] = {
    {EFUSE_BLK1, 64, 3}, 	 // [] Stores RF Calibration data. RXIQ version,
};

static const esp_efuse_desc_t RXIQ_0[] = {
    {EFUSE_BLK1, 67, 7}, 	 // [] Stores RF Calibration data. RXIQ data 0,
};

static const esp_efuse_desc_t RXIQ_1[] = {
    {EFUSE_BLK1, 74, 7}, 	 // [] Stores RF Calibration data. RXIQ data 1,
};

static const esp_efuse_desc_t ACTIVE_HP_DBIAS[] = {
    {EFUSE_BLK1, 81, 5}, 	 // [] Stores the PMU active hp dbias,
};

static const esp_efuse_desc_t ACTIVE_LP_DBIAS[] = {
    {EFUSE_BLK1, 86, 5}, 	 // [] Stores the PMU active lp dbias,
};

static const esp_efuse_desc_t DSLP_DBIAS[] = {
    {EFUSE_BLK1, 91, 4}, 	 // [] Stores the PMU sleep dbias,
};

static const esp_efuse_desc_t DBIAS_VOL_GAP[] = {
    {EFUSE_BLK1, 95, 5}, 	 // [] Stores the low 1 bit of dbias_vol_gap,
};

static const esp_efuse_desc_t WAFER_VERSION_MINOR[] = {
    {EFUSE_BLK1, 114, 3}, 	 // [] Stores the wafer version minor,
};

static const esp_efuse_desc_t WAFER_VERSION_MAJOR[] = {
    {EFUSE_BLK1, 117, 2}, 	 // [] Stores the wafer version major,
};

static const esp_efuse_desc_t DISABLE_WAFER_VERSION_MAJOR[] = {
    {EFUSE_BLK1, 119, 1}, 	 // [] Disables check of wafer version major,
};

static const esp_efuse_desc_t FLASH_CAP[] = {
    {EFUSE_BLK1, 120, 3}, 	 // [] Stores the flash cap,
};

static const esp_efuse_desc_t FLASH_TEMP[] = {
    {EFUSE_BLK1, 123, 2}, 	 // [] Stores the flash temp,
};

static const esp_efuse_desc_t FLASH_VENDOR[] = {
    {EFUSE_BLK1, 125, 3}, 	 // [] Stores the flash vendor,
};

static const esp_efuse_desc_t PKG_VERSION[] = {
    {EFUSE_BLK1, 128, 3}, 	 // [] Package version,
};

static const esp_efuse_desc_t OPTIONAL_UNIQUE_ID[] = {
    {EFUSE_BLK2, 0, 128}, 	 // [] Optional unique 128-bit ID,
};

static const esp_efuse_desc_t BLK_VERSION_MINOR[] = {
    {EFUSE_BLK2, 130, 3}, 	 // [] BLK_VERSION_MINOR of BLOCK2. 1: RF Calibration data in BLOCK1,
};

static const esp_efuse_desc_t BLK_VERSION_MAJOR[] = {
    {EFUSE_BLK2, 133, 2}, 	 // [] BLK_VERSION_MAJOR of BLOCK2,
};

static const esp_efuse_desc_t DISABLE_BLK_VERSION_MAJOR[] = {
    {EFUSE_BLK2, 135, 1}, 	 // [] Disables check of blk version major,
};

static const esp_efuse_desc_t TEMP_CALIB[] = {
    {EFUSE_BLK2, 136, 9}, 	 // [] Temperature calibration data,
};

static const esp_efuse_desc_t ADC1_AVE_INITCODE_ATTEN0[] = {
    {EFUSE_BLK2, 145, 10}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_AVE_INITCODE_ATTEN1[] = {
    {EFUSE_BLK2, 155, 10}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_AVE_INITCODE_ATTEN2[] = {
    {EFUSE_BLK2, 165, 10}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_AVE_INITCODE_ATTEN3[] = {
    {EFUSE_BLK2, 175, 10}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_HI_DOUT_ATTEN0[] = {
    {EFUSE_BLK2, 185, 10}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_HI_DOUT_ATTEN1[] = {
    {EFUSE_BLK2, 195, 10}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_HI_DOUT_ATTEN2[] = {
    {EFUSE_BLK2, 205, 10}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_HI_DOUT_ATTEN3[] = {
    {EFUSE_BLK2, 215, 10}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_CH0_ATTEN0_INITCODE_DIFF[] = {
    {EFUSE_BLK2, 225, 4}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_CH1_ATTEN0_INITCODE_DIFF[] = {
    {EFUSE_BLK2, 229, 4}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_CH2_ATTEN0_INITCODE_DIFF[] = {
    {EFUSE_BLK2, 233, 4}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_CH3_ATTEN0_INITCODE_DIFF[] = {
    {EFUSE_BLK2, 237, 4}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t ADC1_CH4_ATTEN0_INITCODE_DIFF[] = {
    {EFUSE_BLK2, 241, 4}, 	 // [] ADC1 calibration data,
};

static const esp_efuse_desc_t USER_DATA[] = {
    {EFUSE_BLK3, 0, 256}, 	 // [BLOCK_USR_DATA] User data,
};

static const esp_efuse_desc_t USER_DATA_MAC_CUSTOM[] = {
    {EFUSE_BLK3, 200, 48}, 	 // [MAC_CUSTOM CUSTOM_MAC] Custom MAC,
};

static const esp_efuse_desc_t KEY0[] = {
    {EFUSE_BLK4, 0, 256}, 	 // [BLOCK_KEY0] Key0 or user data,
};

static const esp_efuse_desc_t KEY1[] = {
    {EFUSE_BLK5, 0, 256}, 	 // [BLOCK_KEY1] Key1 or user data,
};

static const esp_efuse_desc_t KEY2[] = {
    {EFUSE_BLK6, 0, 256}, 	 // [BLOCK_KEY2] Key2 or user data,
};

static const esp_efuse_desc_t KEY3[] = {
    {EFUSE_BLK7, 0, 256}, 	 // [BLOCK_KEY3] Key3 or user data,
};

static const esp_efuse_desc_t KEY4[] = {
    {EFUSE_BLK8, 0, 256}, 	 // [BLOCK_KEY4] Key4 or user data,
};

static const esp_efuse_desc_t KEY5[] = {
    {EFUSE_BLK9, 0, 256}, 	 // [BLOCK_KEY5] Key5 or user data,
};

static const esp_efuse_desc_t SYS_DATA_PART2[] = {
    {EFUSE_BLK10, 0, 256}, 	 // [BLOCK_SYS_DATA2] System data part 2 (reserved),
};





const esp_efuse_desc_t* ESP_EFUSE_WR_DIS[] = {
    &WR_DIS[0],    		// [] Disable programming of individual eFuses
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_RD_DIS[] = {
    &WR_DIS_RD_DIS[0],    		// [] wr_dis of RD_DIS
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DIS_ICACHE[] = {
    &WR_DIS_DIS_ICACHE[0],    		// [] wr_dis of DIS_ICACHE
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DIS_USB_JTAG[] = {
    &WR_DIS_DIS_USB_JTAG[0],    		// [] wr_dis of DIS_USB_JTAG
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_POWERGLITCH_EN[] = {
    &WR_DIS_POWERGLITCH_EN[0],    		// [] wr_dis of POWERGLITCH_EN
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DIS_FORCE_DOWNLOAD[] = {
    &WR_DIS_DIS_FORCE_DOWNLOAD[0],    		// [] wr_dis of DIS_FORCE_DOWNLOAD
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SPI_DOWNLOAD_MSPI_DIS[] = {
    &WR_DIS_SPI_DOWNLOAD_MSPI_DIS[0],    		// [] wr_dis of SPI_DOWNLOAD_MSPI_DIS
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DIS_TWAI[] = {
    &WR_DIS_DIS_TWAI[0],    		// [WR_DIS.DIS_CAN] wr_dis of DIS_TWAI
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_JTAG_SEL_ENABLE[] = {
    &WR_DIS_JTAG_SEL_ENABLE[0],    		// [] wr_dis of JTAG_SEL_ENABLE
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DIS_PAD_JTAG[] = {
    &WR_DIS_DIS_PAD_JTAG[0],    		// [] wr_dis of DIS_PAD_JTAG
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DIS_DOWNLOAD_MANUAL_ENCRYPT[] = {
    &WR_DIS_DIS_DOWNLOAD_MANUAL_ENCRYPT[0],    		// [] wr_dis of DIS_DOWNLOAD_MANUAL_ENCRYPT
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_POWERGLITCH_EN1[] = {
    &WR_DIS_POWERGLITCH_EN1[0],    		// [] wr_dis of POWERGLITCH_EN1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_WDT_DELAY_SEL[] = {
    &WR_DIS_WDT_DELAY_SEL[0],    		// [] wr_dis of WDT_DELAY_SEL
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SPI_BOOT_CRYPT_CNT[] = {
    &WR_DIS_SPI_BOOT_CRYPT_CNT[0],    		// [] wr_dis of SPI_BOOT_CRYPT_CNT
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SECURE_BOOT_KEY_REVOKE0[] = {
    &WR_DIS_SECURE_BOOT_KEY_REVOKE0[0],    		// [] wr_dis of SECURE_BOOT_KEY_REVOKE0
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SECURE_BOOT_KEY_REVOKE1[] = {
    &WR_DIS_SECURE_BOOT_KEY_REVOKE1[0],    		// [] wr_dis of SECURE_BOOT_KEY_REVOKE1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SECURE_BOOT_KEY_REVOKE2[] = {
    &WR_DIS_SECURE_BOOT_KEY_REVOKE2[0],    		// [] wr_dis of SECURE_BOOT_KEY_REVOKE2
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_KEY_PURPOSE_0[] = {
    &WR_DIS_KEY_PURPOSE_0[0],    		// [WR_DIS.KEY0_PURPOSE] wr_dis of KEY_PURPOSE_0
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_KEY_PURPOSE_1[] = {
    &WR_DIS_KEY_PURPOSE_1[0],    		// [WR_DIS.KEY1_PURPOSE] wr_dis of KEY_PURPOSE_1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_KEY_PURPOSE_2[] = {
    &WR_DIS_KEY_PURPOSE_2[0],    		// [WR_DIS.KEY2_PURPOSE] wr_dis of KEY_PURPOSE_2
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_KEY_PURPOSE_3[] = {
    &WR_DIS_KEY_PURPOSE_3[0],    		// [WR_DIS.KEY3_PURPOSE] wr_dis of KEY_PURPOSE_3
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_KEY_PURPOSE_4[] = {
    &WR_DIS_KEY_PURPOSE_4[0],    		// [WR_DIS.KEY4_PURPOSE] wr_dis of KEY_PURPOSE_4
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_KEY_PURPOSE_5[] = {
    &WR_DIS_KEY_PURPOSE_5[0],    		// [WR_DIS.KEY5_PURPOSE] wr_dis of KEY_PURPOSE_5
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_XTS_DPA_PSEUDO_LEVEL[] = {
    &WR_DIS_XTS_DPA_PSEUDO_LEVEL[0],    		// [] wr_dis of XTS_DPA_PSEUDO_LEVEL
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SEC_DPA_LEVEL[] = {
    &WR_DIS_SEC_DPA_LEVEL[0],    		// [] wr_dis of SEC_DPA_LEVEL
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_CRYPT_DPA_ENABLE[] = {
    &WR_DIS_CRYPT_DPA_ENABLE[0],    		// [] wr_dis of CRYPT_DPA_ENABLE
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SECURE_BOOT_EN[] = {
    &WR_DIS_SECURE_BOOT_EN[0],    		// [] wr_dis of SECURE_BOOT_EN
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SECURE_BOOT_AGGRESSIVE_REVOKE[] = {
    &WR_DIS_SECURE_BOOT_AGGRESSIVE_REVOKE[0],    		// [] wr_dis of SECURE_BOOT_AGGRESSIVE_REVOKE
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ECDSA_CURVE_MODE[] = {
    &WR_DIS_ECDSA_CURVE_MODE[0],    		// [] wr_dis of ECDSA_CURVE_MODE
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ECC_FORCE_CONST_TIME[] = {
    &WR_DIS_ECC_FORCE_CONST_TIME[0],    		// [] wr_dis of ECC_FORCE_CONST_TIME
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_FLASH_TPUW[] = {
    &WR_DIS_FLASH_TPUW[0],    		// [] wr_dis of FLASH_TPUW
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DIS_DOWNLOAD_MODE[] = {
    &WR_DIS_DIS_DOWNLOAD_MODE[0],    		// [] wr_dis of DIS_DOWNLOAD_MODE
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DIS_DIRECT_BOOT[] = {
    &WR_DIS_DIS_DIRECT_BOOT[0],    		// [] wr_dis of DIS_DIRECT_BOOT
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DIS_USB_SERIAL_JTAG_ROM_PRINT[] = {
    &WR_DIS_DIS_USB_SERIAL_JTAG_ROM_PRINT[0],    		// [WR_DIS.DIS_USB_PRINT] wr_dis of DIS_USB_SERIAL_JTAG_ROM_PRINT
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE[] = {
    &WR_DIS_DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE[0],    		// [] wr_dis of DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ENABLE_SECURITY_DOWNLOAD[] = {
    &WR_DIS_ENABLE_SECURITY_DOWNLOAD[0],    		// [] wr_dis of ENABLE_SECURITY_DOWNLOAD
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_UART_PRINT_CONTROL[] = {
    &WR_DIS_UART_PRINT_CONTROL[0],    		// [] wr_dis of UART_PRINT_CONTROL
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_FORCE_SEND_RESUME[] = {
    &WR_DIS_FORCE_SEND_RESUME[0],    		// [] wr_dis of FORCE_SEND_RESUME
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SECURE_VERSION[] = {
    &WR_DIS_SECURE_VERSION[0],    		// [] wr_dis of SECURE_VERSION
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SECURE_BOOT_DISABLE_FAST_WAKE[] = {
    &WR_DIS_SECURE_BOOT_DISABLE_FAST_WAKE[0],    		// [] wr_dis of SECURE_BOOT_DISABLE_FAST_WAKE
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_HYS_EN_PAD0[] = {
    &WR_DIS_HYS_EN_PAD0[0],    		// [] wr_dis of HYS_EN_PAD0
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_HYS_EN_PAD1[] = {
    &WR_DIS_HYS_EN_PAD1[0],    		// [] wr_dis of HYS_EN_PAD1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_BLK1[] = {
    &WR_DIS_BLK1[0],    		// [] wr_dis of BLOCK1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_MAC[] = {
    &WR_DIS_MAC[0],    		// [WR_DIS.MAC_FACTORY] wr_dis of MAC
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_MAC_EXT[] = {
    &WR_DIS_MAC_EXT[0],    		// [] wr_dis of MAC_EXT
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_RXIQ_VERSION[] = {
    &WR_DIS_RXIQ_VERSION[0],    		// [] wr_dis of RXIQ_VERSION
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_RXIQ_0[] = {
    &WR_DIS_RXIQ_0[0],    		// [] wr_dis of RXIQ_0
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_RXIQ_1[] = {
    &WR_DIS_RXIQ_1[0],    		// [] wr_dis of RXIQ_1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ACTIVE_HP_DBIAS[] = {
    &WR_DIS_ACTIVE_HP_DBIAS[0],    		// [] wr_dis of ACTIVE_HP_DBIAS
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ACTIVE_LP_DBIAS[] = {
    &WR_DIS_ACTIVE_LP_DBIAS[0],    		// [] wr_dis of ACTIVE_LP_DBIAS
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DSLP_DBIAS[] = {
    &WR_DIS_DSLP_DBIAS[0],    		// [] wr_dis of DSLP_DBIAS
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DBIAS_VOL_GAP[] = {
    &WR_DIS_DBIAS_VOL_GAP[0],    		// [] wr_dis of DBIAS_VOL_GAP
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_WAFER_VERSION_MINOR[] = {
    &WR_DIS_WAFER_VERSION_MINOR[0],    		// [] wr_dis of WAFER_VERSION_MINOR
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_WAFER_VERSION_MAJOR[] = {
    &WR_DIS_WAFER_VERSION_MAJOR[0],    		// [] wr_dis of WAFER_VERSION_MAJOR
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DISABLE_WAFER_VERSION_MAJOR[] = {
    &WR_DIS_DISABLE_WAFER_VERSION_MAJOR[0],    		// [] wr_dis of DISABLE_WAFER_VERSION_MAJOR
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_FLASH_CAP[] = {
    &WR_DIS_FLASH_CAP[0],    		// [] wr_dis of FLASH_CAP
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_FLASH_TEMP[] = {
    &WR_DIS_FLASH_TEMP[0],    		// [] wr_dis of FLASH_TEMP
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_FLASH_VENDOR[] = {
    &WR_DIS_FLASH_VENDOR[0],    		// [] wr_dis of FLASH_VENDOR
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_PKG_VERSION[] = {
    &WR_DIS_PKG_VERSION[0],    		// [] wr_dis of PKG_VERSION
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SYS_DATA_PART1[] = {
    &WR_DIS_SYS_DATA_PART1[0],    		// [] wr_dis of BLOCK2
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_OPTIONAL_UNIQUE_ID[] = {
    &WR_DIS_OPTIONAL_UNIQUE_ID[0],    		// [] wr_dis of OPTIONAL_UNIQUE_ID
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_BLK_VERSION_MINOR[] = {
    &WR_DIS_BLK_VERSION_MINOR[0],    		// [] wr_dis of BLK_VERSION_MINOR
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_BLK_VERSION_MAJOR[] = {
    &WR_DIS_BLK_VERSION_MAJOR[0],    		// [] wr_dis of BLK_VERSION_MAJOR
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_DISABLE_BLK_VERSION_MAJOR[] = {
    &WR_DIS_DISABLE_BLK_VERSION_MAJOR[0],    		// [] wr_dis of DISABLE_BLK_VERSION_MAJOR
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_TEMP_CALIB[] = {
    &WR_DIS_TEMP_CALIB[0],    		// [] wr_dis of TEMP_CALIB
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_AVE_INITCODE_ATTEN0[] = {
    &WR_DIS_ADC1_AVE_INITCODE_ATTEN0[0],    		// [] wr_dis of ADC1_AVE_INITCODE_ATTEN0
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_AVE_INITCODE_ATTEN1[] = {
    &WR_DIS_ADC1_AVE_INITCODE_ATTEN1[0],    		// [] wr_dis of ADC1_AVE_INITCODE_ATTEN1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_AVE_INITCODE_ATTEN2[] = {
    &WR_DIS_ADC1_AVE_INITCODE_ATTEN2[0],    		// [] wr_dis of ADC1_AVE_INITCODE_ATTEN2
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_AVE_INITCODE_ATTEN3[] = {
    &WR_DIS_ADC1_AVE_INITCODE_ATTEN3[0],    		// [] wr_dis of ADC1_AVE_INITCODE_ATTEN3
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_HI_DOUT_ATTEN0[] = {
    &WR_DIS_ADC1_HI_DOUT_ATTEN0[0],    		// [] wr_dis of ADC1_HI_DOUT_ATTEN0
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_HI_DOUT_ATTEN1[] = {
    &WR_DIS_ADC1_HI_DOUT_ATTEN1[0],    		// [] wr_dis of ADC1_HI_DOUT_ATTEN1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_HI_DOUT_ATTEN2[] = {
    &WR_DIS_ADC1_HI_DOUT_ATTEN2[0],    		// [] wr_dis of ADC1_HI_DOUT_ATTEN2
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_HI_DOUT_ATTEN3[] = {
    &WR_DIS_ADC1_HI_DOUT_ATTEN3[0],    		// [] wr_dis of ADC1_HI_DOUT_ATTEN3
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_CH0_ATTEN0_INITCODE_DIFF[] = {
    &WR_DIS_ADC1_CH0_ATTEN0_INITCODE_DIFF[0],    		// [] wr_dis of ADC1_CH0_ATTEN0_INITCODE_DIFF
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_CH1_ATTEN0_INITCODE_DIFF[] = {
    &WR_DIS_ADC1_CH1_ATTEN0_INITCODE_DIFF[0],    		// [] wr_dis of ADC1_CH1_ATTEN0_INITCODE_DIFF
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_CH2_ATTEN0_INITCODE_DIFF[] = {
    &WR_DIS_ADC1_CH2_ATTEN0_INITCODE_DIFF[0],    		// [] wr_dis of ADC1_CH2_ATTEN0_INITCODE_DIFF
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_CH3_ATTEN0_INITCODE_DIFF[] = {
    &WR_DIS_ADC1_CH3_ATTEN0_INITCODE_DIFF[0],    		// [] wr_dis of ADC1_CH3_ATTEN0_INITCODE_DIFF
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_ADC1_CH4_ATTEN0_INITCODE_DIFF[] = {
    &WR_DIS_ADC1_CH4_ATTEN0_INITCODE_DIFF[0],    		// [] wr_dis of ADC1_CH4_ATTEN0_INITCODE_DIFF
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_BLOCK_USR_DATA[] = {
    &WR_DIS_BLOCK_USR_DATA[0],    		// [WR_DIS.USER_DATA] wr_dis of BLOCK_USR_DATA
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_CUSTOM_MAC[] = {
    &WR_DIS_CUSTOM_MAC[0],    		// [WR_DIS.MAC_CUSTOM WR_DIS.USER_DATA_MAC_CUSTOM] wr_dis of CUSTOM_MAC
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_BLOCK_KEY0[] = {
    &WR_DIS_BLOCK_KEY0[0],    		// [WR_DIS.KEY0] wr_dis of BLOCK_KEY0
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_BLOCK_KEY1[] = {
    &WR_DIS_BLOCK_KEY1[0],    		// [WR_DIS.KEY1] wr_dis of BLOCK_KEY1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_BLOCK_KEY2[] = {
    &WR_DIS_BLOCK_KEY2[0],    		// [WR_DIS.KEY2] wr_dis of BLOCK_KEY2
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_BLOCK_KEY3[] = {
    &WR_DIS_BLOCK_KEY3[0],    		// [WR_DIS.KEY3] wr_dis of BLOCK_KEY3
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_BLOCK_KEY4[] = {
    &WR_DIS_BLOCK_KEY4[0],    		// [WR_DIS.KEY4] wr_dis of BLOCK_KEY4
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_BLOCK_KEY5[] = {
    &WR_DIS_BLOCK_KEY5[0],    		// [WR_DIS.KEY5] wr_dis of BLOCK_KEY5
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_BLOCK_SYS_DATA2[] = {
    &WR_DIS_BLOCK_SYS_DATA2[0],    		// [WR_DIS.SYS_DATA_PART2] wr_dis of BLOCK_SYS_DATA2
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_USB_EXCHG_PINS[] = {
    &WR_DIS_USB_EXCHG_PINS[0],    		// [] wr_dis of USB_EXCHG_PINS
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_VDD_SPI_AS_GPIO[] = {
    &WR_DIS_VDD_SPI_AS_GPIO[0],    		// [] wr_dis of VDD_SPI_AS_GPIO
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WR_DIS_SOFT_DIS_JTAG[] = {
    &WR_DIS_SOFT_DIS_JTAG[0],    		// [] wr_dis of SOFT_DIS_JTAG
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_RD_DIS[] = {
    &RD_DIS[0],    		// [] Disable reading from BlOCK4-10
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_RD_DIS_BLOCK_KEY0[] = {
    &RD_DIS_BLOCK_KEY0[0],    		// [RD_DIS.KEY0] rd_dis of BLOCK_KEY0
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_RD_DIS_BLOCK_KEY1[] = {
    &RD_DIS_BLOCK_KEY1[0],    		// [RD_DIS.KEY1] rd_dis of BLOCK_KEY1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_RD_DIS_BLOCK_KEY2[] = {
    &RD_DIS_BLOCK_KEY2[0],    		// [RD_DIS.KEY2] rd_dis of BLOCK_KEY2
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_RD_DIS_BLOCK_KEY3[] = {
    &RD_DIS_BLOCK_KEY3[0],    		// [RD_DIS.KEY3] rd_dis of BLOCK_KEY3
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_RD_DIS_BLOCK_KEY4[] = {
    &RD_DIS_BLOCK_KEY4[0],    		// [RD_DIS.KEY4] rd_dis of BLOCK_KEY4
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_RD_DIS_BLOCK_KEY5[] = {
    &RD_DIS_BLOCK_KEY5[0],    		// [RD_DIS.KEY5] rd_dis of BLOCK_KEY5
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_RD_DIS_BLOCK_SYS_DATA2[] = {
    &RD_DIS_BLOCK_SYS_DATA2[0],    		// [RD_DIS.SYS_DATA_PART2] rd_dis of BLOCK_SYS_DATA2
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DIS_ICACHE[] = {
    &DIS_ICACHE[0],    		// [] Represents whether icache is disabled or enabled. 1: disabled. 0: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DIS_USB_JTAG[] = {
    &DIS_USB_JTAG[0],    		// [] Represents whether the function of usb switch to jtag is disabled or enabled. 1: disabled. 0: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_POWERGLITCH_EN[] = {
    &POWERGLITCH_EN[0],    		// [] Represents whether power glitch function is enabled. 1: enabled. 0: disabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DIS_FORCE_DOWNLOAD[] = {
    &DIS_FORCE_DOWNLOAD[0],    		// [] Represents whether the function that forces chip into download mode is disabled or enabled. 1: disabled. 0: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SPI_DOWNLOAD_MSPI_DIS[] = {
    &SPI_DOWNLOAD_MSPI_DIS[0],    		// [] Represents whether SPI0 controller during boot_mode_download is disabled or enabled. 1: disabled. 0: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DIS_TWAI[] = {
    &DIS_TWAI[0],    		// [DIS_CAN] Represents whether TWAI function is disabled or enabled. 1: disabled. 0: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_JTAG_SEL_ENABLE[] = {
    &JTAG_SEL_ENABLE[0],    		// [] Set this bit to enable selection between usb_to_jtag and pad_to_jtag through strapping gpio25 when both EFUSE_DIS_PAD_JTAG and EFUSE_DIS_USB_JTAG are equal to 0
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SOFT_DIS_JTAG[] = {
    &SOFT_DIS_JTAG[0],    		// [] Represents whether JTAG is disabled in soft way. Odd number: disabled. Even number: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DIS_PAD_JTAG[] = {
    &DIS_PAD_JTAG[0],    		// [] Represents whether JTAG is disabled in the hard way(permanently). 1: disabled. 0: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DIS_DOWNLOAD_MANUAL_ENCRYPT[] = {
    &DIS_DOWNLOAD_MANUAL_ENCRYPT[0],    		// [] Represents whether flash encrypt function is disabled or enabled(except in SPI boot mode). 1: disabled. 0: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_USB_EXCHG_PINS[] = {
    &USB_EXCHG_PINS[0],    		// [] Represents whether the D+ and D- pins is exchanged. 1: exchanged. 0: not exchanged
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_VDD_SPI_AS_GPIO[] = {
    &VDD_SPI_AS_GPIO[0],    		// [] Represents whether vdd spi pin is functioned as gpio. 1: functioned. 0: not functioned
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ECDSA_CURVE_MODE[] = {
    &ECDSA_CURVE_MODE[0],    		// [] Configures the curve of ECDSA calculation: 0: only enable P256. 1: only enable P192. 2: both enable P256 and P192. 3: only enable P256
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ECC_FORCE_CONST_TIME[] = {
    &ECC_FORCE_CONST_TIME[0],    		// [] Set this bit to permanently turn on ECC const-time mode
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_XTS_DPA_PSEUDO_LEVEL[] = {
    &XTS_DPA_PSEUDO_LEVEL[0],    		// [] Set this bit to control the xts pseudo-round anti-dpa attack function: 0: controlled by register. 1-3: the higher the value is; the more pseudo-rounds are inserted to the xts-aes calculation
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WDT_DELAY_SEL[] = {
    &WDT_DELAY_SEL[0],    		// [] Represents whether RTC watchdog timeout threshold is selected at startup. 1: selected. 0: not selected
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SPI_BOOT_CRYPT_CNT[] = {
    &SPI_BOOT_CRYPT_CNT[0],    		// [] Enables flash encryption when 1 or 3 bits are set and disables otherwise {0: "Disable"; 1: "Enable"; 3: "Disable"; 7: "Enable"}
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SECURE_BOOT_KEY_REVOKE0[] = {
    &SECURE_BOOT_KEY_REVOKE0[0],    		// [] Revoke 1st secure boot key
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SECURE_BOOT_KEY_REVOKE1[] = {
    &SECURE_BOOT_KEY_REVOKE1[0],    		// [] Revoke 2nd secure boot key
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SECURE_BOOT_KEY_REVOKE2[] = {
    &SECURE_BOOT_KEY_REVOKE2[0],    		// [] Revoke 3rd secure boot key
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY_PURPOSE_0[] = {
    &KEY_PURPOSE_0[0],    		// [KEY0_PURPOSE] Represents the purpose of Key0
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY_PURPOSE_1[] = {
    &KEY_PURPOSE_1[0],    		// [KEY1_PURPOSE] Represents the purpose of Key1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY_PURPOSE_2[] = {
    &KEY_PURPOSE_2[0],    		// [KEY2_PURPOSE] Represents the purpose of Key2
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY_PURPOSE_3[] = {
    &KEY_PURPOSE_3[0],    		// [KEY3_PURPOSE] Represents the purpose of Key3
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY_PURPOSE_4[] = {
    &KEY_PURPOSE_4[0],    		// [KEY4_PURPOSE] Represents the purpose of Key4
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY_PURPOSE_5[] = {
    &KEY_PURPOSE_5[0],    		// [KEY5_PURPOSE] Represents the purpose of Key5
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SEC_DPA_LEVEL[] = {
    &SEC_DPA_LEVEL[0],    		// [] Represents the spa secure level by configuring the clock random divide mode
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_CRYPT_DPA_ENABLE[] = {
    &CRYPT_DPA_ENABLE[0],    		// [] Represents whether anti-dpa attack is enabled. 1:enabled. 0: disabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SECURE_BOOT_EN[] = {
    &SECURE_BOOT_EN[0],    		// [] Represents whether secure boot is enabled or disabled. 1: enabled. 0: disabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SECURE_BOOT_AGGRESSIVE_REVOKE[] = {
    &SECURE_BOOT_AGGRESSIVE_REVOKE[0],    		// [] Represents whether revoking aggressive secure boot is enabled or disabled. 1: enabled. 0: disabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_POWERGLITCH_EN1[] = {
    &POWERGLITCH_EN1[0],    		// [] Set these bits to enable power glitch function when chip power on
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_FLASH_TPUW[] = {
    &FLASH_TPUW[0],    		// [] Represents the flash waiting time after power-up; in unit of ms. When the value less than 15; the waiting time is the programmed value. Otherwise; the waiting time is 2 times the programmed value
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DIS_DOWNLOAD_MODE[] = {
    &DIS_DOWNLOAD_MODE[0],    		// [] Represents whether Download mode is disabled or enabled. 1: disabled. 0: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DIS_DIRECT_BOOT[] = {
    &DIS_DIRECT_BOOT[0],    		// [] Represents whether direct boot mode is disabled or enabled. 1: disabled. 0: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DIS_USB_SERIAL_JTAG_ROM_PRINT[] = {
    &DIS_USB_SERIAL_JTAG_ROM_PRINT[0],    		// [DIS_USB_PRINT] Set this bit to disable USB-Serial-JTAG print during rom boot
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE[] = {
    &DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE[0],    		// [] Represents whether the USB-Serial-JTAG download function is disabled or enabled. 1: disabled. 0: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ENABLE_SECURITY_DOWNLOAD[] = {
    &ENABLE_SECURITY_DOWNLOAD[0],    		// [] Represents whether security download is enabled or disabled. 1: enabled. 0: disabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_UART_PRINT_CONTROL[] = {
    &UART_PRINT_CONTROL[0],    		// [] Set the default UARTboot message output mode {0: "Enable"; 1: "Enable when GPIO8 is low at reset"; 2: "Enable when GPIO8 is high at reset"; 3: "Disable"}
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_FORCE_SEND_RESUME[] = {
    &FORCE_SEND_RESUME[0],    		// [] Represents whether ROM code is forced to send a resume command during SPI boot. 1: forced. 0:not forced
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SECURE_VERSION[] = {
    &SECURE_VERSION[0],    		// [] Represents the version used by ESP-IDF anti-rollback feature
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SECURE_BOOT_DISABLE_FAST_WAKE[] = {
    &SECURE_BOOT_DISABLE_FAST_WAKE[0],    		// [] Represents whether FAST VERIFY ON WAKE is disabled or enabled when Secure Boot is enabled. 1: disabled. 0: enabled
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_HYS_EN_PAD0[] = {
    &HYS_EN_PAD0[0],    		// [] Set bits to enable hysteresis function of PAD0~5
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_HYS_EN_PAD1[] = {
    &HYS_EN_PAD1[0],    		// [] Set bits to enable hysteresis function of PAD6~27
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_MAC[] = {
    &MAC[0],    		// [MAC_FACTORY] MAC address
    &MAC[1],    		// [MAC_FACTORY] MAC address
    &MAC[2],    		// [MAC_FACTORY] MAC address
    &MAC[3],    		// [MAC_FACTORY] MAC address
    &MAC[4],    		// [MAC_FACTORY] MAC address
    &MAC[5],    		// [MAC_FACTORY] MAC address
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_MAC_EXT[] = {
    &MAC_EXT[0],    		// [] Stores the extended bits of MAC address
    &MAC_EXT[1],    		// [] Stores the extended bits of MAC address
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_RXIQ_VERSION[] = {
    &RXIQ_VERSION[0],    		// [] Stores RF Calibration data. RXIQ version
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_RXIQ_0[] = {
    &RXIQ_0[0],    		// [] Stores RF Calibration data. RXIQ data 0
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_RXIQ_1[] = {
    &RXIQ_1[0],    		// [] Stores RF Calibration data. RXIQ data 1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ACTIVE_HP_DBIAS[] = {
    &ACTIVE_HP_DBIAS[0],    		// [] Stores the PMU active hp dbias
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ACTIVE_LP_DBIAS[] = {
    &ACTIVE_LP_DBIAS[0],    		// [] Stores the PMU active lp dbias
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DSLP_DBIAS[] = {
    &DSLP_DBIAS[0],    		// [] Stores the PMU sleep dbias
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DBIAS_VOL_GAP[] = {
    &DBIAS_VOL_GAP[0],    		// [] Stores the low 1 bit of dbias_vol_gap
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WAFER_VERSION_MINOR[] = {
    &WAFER_VERSION_MINOR[0],    		// [] Stores the wafer version minor
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_WAFER_VERSION_MAJOR[] = {
    &WAFER_VERSION_MAJOR[0],    		// [] Stores the wafer version major
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DISABLE_WAFER_VERSION_MAJOR[] = {
    &DISABLE_WAFER_VERSION_MAJOR[0],    		// [] Disables check of wafer version major
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_FLASH_CAP[] = {
    &FLASH_CAP[0],    		// [] Stores the flash cap
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_FLASH_TEMP[] = {
    &FLASH_TEMP[0],    		// [] Stores the flash temp
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_FLASH_VENDOR[] = {
    &FLASH_VENDOR[0],    		// [] Stores the flash vendor
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_PKG_VERSION[] = {
    &PKG_VERSION[0],    		// [] Package version
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_OPTIONAL_UNIQUE_ID[] = {
    &OPTIONAL_UNIQUE_ID[0],    		// [] Optional unique 128-bit ID
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_BLK_VERSION_MINOR[] = {
    &BLK_VERSION_MINOR[0],    		// [] BLK_VERSION_MINOR of BLOCK2. 1: RF Calibration data in BLOCK1
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_BLK_VERSION_MAJOR[] = {
    &BLK_VERSION_MAJOR[0],    		// [] BLK_VERSION_MAJOR of BLOCK2
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_DISABLE_BLK_VERSION_MAJOR[] = {
    &DISABLE_BLK_VERSION_MAJOR[0],    		// [] Disables check of blk version major
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_TEMP_CALIB[] = {
    &TEMP_CALIB[0],    		// [] Temperature calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_AVE_INITCODE_ATTEN0[] = {
    &ADC1_AVE_INITCODE_ATTEN0[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_AVE_INITCODE_ATTEN1[] = {
    &ADC1_AVE_INITCODE_ATTEN1[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_AVE_INITCODE_ATTEN2[] = {
    &ADC1_AVE_INITCODE_ATTEN2[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_AVE_INITCODE_ATTEN3[] = {
    &ADC1_AVE_INITCODE_ATTEN3[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_HI_DOUT_ATTEN0[] = {
    &ADC1_HI_DOUT_ATTEN0[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_HI_DOUT_ATTEN1[] = {
    &ADC1_HI_DOUT_ATTEN1[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_HI_DOUT_ATTEN2[] = {
    &ADC1_HI_DOUT_ATTEN2[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_HI_DOUT_ATTEN3[] = {
    &ADC1_HI_DOUT_ATTEN3[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_CH0_ATTEN0_INITCODE_DIFF[] = {
    &ADC1_CH0_ATTEN0_INITCODE_DIFF[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_CH1_ATTEN0_INITCODE_DIFF[] = {
    &ADC1_CH1_ATTEN0_INITCODE_DIFF[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_CH2_ATTEN0_INITCODE_DIFF[] = {
    &ADC1_CH2_ATTEN0_INITCODE_DIFF[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_CH3_ATTEN0_INITCODE_DIFF[] = {
    &ADC1_CH3_ATTEN0_INITCODE_DIFF[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_ADC1_CH4_ATTEN0_INITCODE_DIFF[] = {
    &ADC1_CH4_ATTEN0_INITCODE_DIFF[0],    		// [] ADC1 calibration data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_USER_DATA[] = {
    &USER_DATA[0],    		// [BLOCK_USR_DATA] User data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_USER_DATA_MAC_CUSTOM[] = {
    &USER_DATA_MAC_CUSTOM[0],    		// [MAC_CUSTOM CUSTOM_MAC] Custom MAC
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY0[] = {
    &KEY0[0],    		// [BLOCK_KEY0] Key0 or user data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY1[] = {
    &KEY1[0],    		// [BLOCK_KEY1] Key1 or user data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY2[] = {
    &KEY2[0],    		// [BLOCK_KEY2] Key2 or user data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY3[] = {
    &KEY3[0],    		// [BLOCK_KEY3] Key3 or user data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY4[] = {
    &KEY4[0],    		// [BLOCK_KEY4] Key4 or user data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_KEY5[] = {
    &KEY5[0],    		// [BLOCK_KEY5] Key5 or user data
    NULL
};

const esp_efuse_desc_t* ESP_EFUSE_SYS_DATA_PART2[] = {
    &SYS_DATA_PART2[0],    		// [BLOCK_SYS_DATA2] System data part 2 (reserved)
    NULL
};
