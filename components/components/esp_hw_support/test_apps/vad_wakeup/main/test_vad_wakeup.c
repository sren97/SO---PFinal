/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/lp_i2s.h"
#include "driver/lp_i2s_std.h"
#include "driver/i2s_std.h"
#include "ulp_lp_core_lp_vad_shared.h"
#include "unity.h"
#include "esp_timer.h"

#define TEST_I2S_FRAME_SIZE            (128)       // Frame numbers in every writing / reading
#define TEST_I2S_TRANS_SIZE            (4096)      // Trans size
#define TEST_LP_I2S_PIN_BCK            4
#define TEST_LP_I2S_PIN_WS             5
#define TEST_LP_I2S_PIN_DIN            6


extern const uint8_t test_vad_pcm_start[]            asm("_binary_test_vad_8k_pcm_start");
extern const uint8_t test_vad_pcm_end[]              asm("_binary_test_vad_8k_pcm_end");
static const char *TAG = "TEST_VAD";

static void s_hp_i2s_config(void)
{
    esp_err_t ret = ESP_FAIL;
    int pcm_size = test_vad_pcm_end - test_vad_pcm_start;
    printf("pcm_size: %d\n", pcm_size);

    i2s_chan_handle_t tx_handle = NULL;
    i2s_chan_config_t i2s_channel_config = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 16,
        .dma_frame_num = TEST_I2S_FRAME_SIZE,
        .auto_clear = false,
    };
    TEST_ESP_OK(i2s_new_channel(&i2s_channel_config, &tx_handle, NULL));


    i2s_std_config_t i2s_std_config = {
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_7,
            .ws   = GPIO_NUM_8,
            .dout = GPIO_NUM_21,
            .din  = -1,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    i2s_std_config.clk_cfg = (i2s_std_clk_config_t)I2S_STD_CLK_DEFAULT_CONFIG(16000);
    i2s_std_config.slot_cfg = (i2s_std_slot_config_t)I2S_STD_PCM_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    TEST_ESP_OK(i2s_channel_init_std_mode(tx_handle, &i2s_std_config));


    uint8_t *txbuf = (uint8_t *)heap_caps_calloc(1, TEST_I2S_TRANS_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    TEST_ASSERT(txbuf);

    uint8_t *prebuf = (uint8_t *)heap_caps_calloc(1, TEST_I2S_TRANS_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    TEST_ASSERT(prebuf);

    memcpy(prebuf, test_vad_pcm_start, TEST_I2S_TRANS_SIZE);
    memcpy(txbuf, test_vad_pcm_start, TEST_I2S_TRANS_SIZE);

    for (int i = 0; i < TEST_I2S_TRANS_SIZE; i++) {
        ESP_LOGD(TAG, "prebuf[%d]: %d", i, prebuf[i]);
        ESP_LOGD(TAG, "txbuf[%d]: %d", i, txbuf[i]);
    }

    size_t bytes_written = 0;
    TEST_ESP_OK(i2s_channel_preload_data(tx_handle, prebuf, TEST_I2S_TRANS_SIZE, &bytes_written));

    TEST_ESP_OK(i2s_channel_enable(tx_handle));

    while (1) {
        ret = i2s_channel_write(tx_handle, txbuf, TEST_I2S_TRANS_SIZE, &bytes_written, 0);
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            TEST_ESP_OK(ret);
        }
        ESP_LOGD(TAG, "bytes_written: %d", bytes_written);
        vTaskDelay(1);
    }
}

static void s_lp_vad_config(void)
{
    ESP_ERROR_CHECK(esp_sleep_enable_vad_wakeup());

    lp_i2s_chan_handle_t rx_handle = NULL;
    lp_i2s_chan_config_t config = {
        .id = 0,
        .role = I2S_ROLE_SLAVE,
        .threshold = 512,
    };
    TEST_ESP_OK(lp_i2s_new_channel(&config, NULL, &rx_handle));

    lp_i2s_std_config_t lp_std_cfg = {
        .pin_cfg = {
            .bck = TEST_LP_I2S_PIN_BCK,
            .ws = TEST_LP_I2S_PIN_WS,
            .din = TEST_LP_I2S_PIN_DIN,
        },
    };
    lp_std_cfg.slot_cfg = (lp_i2s_std_slot_config_t)LP_I2S_STD_PCM_SHORT_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    TEST_ESP_OK(lp_i2s_channel_init_std_mode(rx_handle, &lp_std_cfg));

    // LP VAD Init
    lp_vad_init_config_t init_config = {
        .lp_i2s_chan = rx_handle,
        .vad_config = {
            .init_frame_num = 100,
            .min_energy_thresh = 100,
            .speak_activity_thresh = 10,
            .non_speak_activity_thresh = 30,
            .min_speak_activity_thresh = 3,
            .max_speak_activity_thresh = 100,
        },
    };
    TEST_ESP_OK(lp_core_lp_vad_init(0, &init_config));
    TEST_ESP_OK(lp_i2s_channel_enable(rx_handle));
    TEST_ESP_OK(lp_core_lp_vad_enable(0));

    printf("Entering light sleep\n");
    /* To make sure the complete line is printed before entering sleep mode,
        * need to wait until UART TX FIFO is empty:
        */
    uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

    /* Enter sleep mode */
    esp_light_sleep_start();

    /* Determine wake up reason */
    const char* wakeup_reason;
    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_VAD:
            wakeup_reason = "vad";
            break;
        default:
            wakeup_reason = "other";
            TEST_ASSERT(false);
            break;
    }

    ESP_LOGI(TAG, "wakeup, reason: %s", wakeup_reason);
}

TEST_CASE_MULTIPLE_DEVICES("test LP VAD wakeup", "[vad][ignore][manual]", s_hp_i2s_config, s_lp_vad_config);
