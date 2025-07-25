/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "soc/interrupts.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /* HP peripherals */
    PERIPH_LEDC_MODULE = 0,
    PERIPH_UART0_MODULE,
    PERIPH_UART1_MODULE,
    PERIPH_USB_DEVICE_MODULE,
    PERIPH_I2C0_MODULE,
    PERIPH_I2S0_MODULE,
    PERIPH_TIMG0_MODULE,
    PERIPH_TIMG1_MODULE,
    PERIPH_UHCI0_MODULE,
    PERIPH_RMT_MODULE,
    PERIPH_PCNT_MODULE,
    PERIPH_SPI_MODULE,  //SPI1
    PERIPH_SPI2_MODULE, //SPI2
    PERIPH_TWAI0_MODULE,
    PERIPH_TWAI1_MODULE,
    PERIPH_RNG_MODULE,
    PERIPH_RSA_MODULE,
    PERIPH_AES_MODULE,
    PERIPH_SHA_MODULE,
    PERIPH_ECC_MODULE,
    PERIPH_HMAC_MODULE,
    PERIPH_DS_MODULE,
    PERIPH_SDIO_SLAVE_MODULE,
    PERIPH_GDMA_MODULE,
    PERIPH_MCPWM0_MODULE,
    PERIPH_ETM_MODULE,
    PERIPH_PARLIO_MODULE,
    PERIPH_SYSTIMER_MODULE,
    PERIPH_SARADC_MODULE,
    PERIPH_TEMPSENSOR_MODULE,
    PERIPH_ASSIST_DEBUG_MODULE,
    /* LP peripherals */
    PERIPH_LP_I2C0_MODULE,
    PERIPH_LP_UART0_MODULE,
    /* Peripherals clock managed by the modem_clock driver must be listed last in the enumeration */
    PERIPH_WIFI_MODULE,
    PERIPH_BT_MODULE,
    PERIPH_IEEE802154_MODULE,
    PERIPH_COEX_MODULE,
    PERIPH_PHY_MODULE,
    PERIPH_ANA_I2C_MASTER_MODULE,
    PERIPH_MODEM_ETM_MODULE,
    PERIPH_MODEM_ADC_COMMON_FE_MODULE,
    PERIPH_MODULE_MAX
    /*  !!! Don't append soc modules here !!! */
} periph_module_t;

#define PERIPH_MODEM_MODULE_MIN PERIPH_WIFI_MODULE
#define PERIPH_MODEM_MODULE_MAX PERIPH_MODEM_ADC_COMMON_FE_MODULE
#define PERIPH_MODEM_MODULE_NUM (PERIPH_MODEM_MODULE_MAX - PERIPH_MODEM_MODULE_MIN + 1)
#define IS_MODEM_MODULE(periph)  ((periph>=PERIPH_MODEM_MODULE_MIN) && (periph<=PERIPH_MODEM_MODULE_MAX))

#ifdef __cplusplus
}
#endif
