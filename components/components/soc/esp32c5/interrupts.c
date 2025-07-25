/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc/interrupts.h"

const char *const esp_isr_names[] = {
    [ETS_WIFI_MAC_INTR_SOURCE] = "WIFI_MAC",
    [ETS_WIFI_MAC_NMI_SOURCE] = "WIFI_MAC_NMI",
    [ETS_WIFI_PWR_INTR_SOURCE] = "WIFI_PWR",
    [ETS_WIFI_BB_INTR_SOURCE] = "WIFI_BB",
    [ETS_BT_MAC_INTR_SOURCE] = "BT_MAC",
    [ETS_BT_BB_INTR_SOURCE] = "BT_BB",
    [ETS_BT_BB_NMI_SOURCE] = "BT_BB_NMI",
    [ETS_LP_TIMER_INTR_SOURCE] = "LP_TIMER",
    [ETS_COEX_INTR_SOURCE] = "COEX",
    [ETS_BLE_TIMER_INTR_SOURCE] = "BLE_TIMER",
    [ETS_BLE_SEC_INTR_SOURCE] = "BLE_SEC",
    [ETS_I2C_MASTER_SOURCE] = "I2C_MASTER",
    [ETS_ZB_MAC_INTR_SOURCE] = "ZB_MAC",
    [ETS_PMU_INTR_SOURCE] = "PMU",
    [ETS_EFUSE_INTR_SOURCE] = "EFUSE",
    [ETS_LP_RTC_TIMER_INTR_SOURCE] = "LP_RTC_TIMER",
    [ETS_LP_UART_INTR_SOURCE] = "LP_UART",
    [ETS_LP_I2C_INTR_SOURCE] = "LP_I2C",
    [ETS_LP_WDT_INTR_SOURCE] = "LP_WDT",
    [ETS_LP_PERI_TIMEOUT_INTR_SOURCE] = "LP_PERI_TIMEOUT",
    [ETS_LP_APM_M0_INTR_SOURCE] = "LP_APM_M0",
    [ETS_LP_APM_M1_INTR_SOURCE] = "LP_APM_M1",
    [ETS_HUK_INTR_SOURCE] = "HUK",
    [ETS_FROM_CPU_INTR0_SOURCE] = "FROM_CPU_INTR0",
    [ETS_FROM_CPU_INTR1_SOURCE] = "FROM_CPU_INTR1",
    [ETS_FROM_CPU_INTR2_SOURCE] = "FROM_CPU_INTR2",
    [ETS_FROM_CPU_INTR3_SOURCE] = "FROM_CPU_INTR3",
    [ETS_ASSIST_DEBUG_INTR_SOURCE] = "ASSIST_DEBUG",
    [ETS_TRACE_INTR_SOURCE] = "TRACE",
    [ETS_CACHE_INTR_SOURCE] = "CACHE",
    [ETS_CPU_PERI_TIMEOUT_INTR_SOURCE] = "CPU_PERI_TIMEOUT",
    [ETS_GPIO_INTR_SOURCE] = "GPIO_INTR",
    [ETS_GPIO_NMI_SOURCE] = "GPIO_NMI",
    [ETS_PAU_INTR_SOURCE] = "PAU",
    [ETS_HP_PERI_TIMEOUT_INTR_SOURCE] = "HP_PERI_TIMEOUT",
    [ETS_MODEM_PERI_TIMEOUT_INTR_SOURCE] = "MODEM_PERI_TIMEOUT",
    [ETS_HP_APM_M0_INTR_SOURCE] = "HP_APM_M0",
    [ETS_HP_APM_M1_INTR_SOURCE] = "HP_APM_M1",
    [ETS_HP_APM_M2_INTR_SOURCE] = "HP_APM_M2",
    [ETS_HP_APM_M3_INTR_SOURCE] = "HP_APM_M3",
    [ETS_HP_APM_M4_INTR_SOURCE] = "HP_APM_M4",
    [ETS_LP_APM0_INTR_SOURCE] = "LP_APM0",
    [ETS_MSPI_INTR_SOURCE] = "MSPI",
    [ETS_I2S0_INTR_SOURCE] = "I2S0",
    [ETS_UHCI0_INTR_SOURCE] = "UHCI0",
    [ETS_UART0_INTR_SOURCE] = "UART0",
    [ETS_UART1_INTR_SOURCE] = "UART1",
    [ETS_LEDC_INTR_SOURCE] = "LEDC",
    [ETS_TWAI0_INTR_SOURCE] = "TWAI0",
    [ETS_TWAI0_TIMER_INTR_SOURCE] = "TWAI0_TIMER",
    [ETS_TWAI1_INTR_SOURCE] = "TWAI1",
    [ETS_TWAI1_TIMER_INTR_SOURCE] = "TWAI1_TIMER",
    [ETS_USB_SERIAL_JTAG_INTR_SOURCE] = "USB_SERIAL_JTAG",
    [ETS_RMT_INTR_SOURCE] = "RMT",
    [ETS_I2C_EXT0_INTR_SOURCE] = "I2C_EXT0",
    [ETS_TG0_T0_LEVEL_INTR_SOURCE] = "TG0_T0_LEVEL",
    [ETS_TG0_WDT_LEVEL_INTR_SOURCE] = "TG0_WDT_LEVEL",
    [ETS_TG1_T0_LEVEL_INTR_SOURCE] = "TG1_T0_LEVEL",
    [ETS_TG1_WDT_LEVEL_INTR_SOURCE] = "TG1_WDT_LEVEL",
    [ETS_SYSTIMER_TARGET0_INTR_SOURCE] = "SYSTIMER_TARGET0",
    [ETS_SYSTIMER_TARGET1_INTR_SOURCE] = "SYSTIMER_TARGET1",
    [ETS_SYSTIMER_TARGET2_INTR_SOURCE] = "SYSTIMER_TARGET2",
    [ETS_APB_ADC_INTR_SOURCE] = "APB_ADC",
    [ETS_MCPWM0_INTR_SOURCE] = "MCPWM0",
    [ETS_PCNT_INTR_SOURCE] = "PCNT",
    [ETS_PARL_IO_TX_INTR_SOURCE] = "PARL_IO_TX",
    [ETS_PARL_IO_RX_INTR_SOURCE] = "PARL_IO_RX",
    [ETS_DMA_IN_CH0_INTR_SOURCE] = "DMA_IN_CH0",
    [ETS_DMA_IN_CH1_INTR_SOURCE] = "DMA_IN_CH1",
    [ETS_DMA_IN_CH2_INTR_SOURCE] = "DMA_IN_CH2",
    [ETS_DMA_OUT_CH0_INTR_SOURCE] = "DMA_OUT_CH0",
    [ETS_DMA_OUT_CH1_INTR_SOURCE] = "DMA_OUT_CH1",
    [ETS_DMA_OUT_CH2_INTR_SOURCE] = "DMA_OUT_CH2",
    [ETS_GPSPI2_INTR_SOURCE] = "GPSPI2",
    [ETS_AES_INTR_SOURCE] = "AES",
    [ETS_SHA_INTR_SOURCE] = "SHA",
    [ETS_RSA_INTR_SOURCE] = "RSA",
    [ETS_ECC_INTR_SOURCE] = "ECC",
    [ETS_ECDSA_INTR_SOURCE] = "ECDSA",
    [ETS_KM_INTR_SOURCE] = "KM"
};
