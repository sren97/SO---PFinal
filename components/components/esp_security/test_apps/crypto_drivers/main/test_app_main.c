/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "unity.h"
#include "unity_test_runner.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#ifdef CONFIG_HEAP_TRACING
#include "memory_checks.h"
#include "esp_heap_trace.h"
#endif
#include "esp_crypto_lock.h"
#include "esp_partition.h"

#define TEST_TASK_PRIORITY 5

#define TEST_MEMORY_LEAK_THRESHOLD_DEFAULT -400
static int leak_threshold = TEST_MEMORY_LEAK_THRESHOLD_DEFAULT;

void set_leak_threshold(int threshold)
{
    leak_threshold = threshold;
}

static size_t before_free_8bit;
static size_t before_free_32bit;
static const char* TAG = "crypto_drivers_test_app";

static void check_leak(size_t before_free, size_t after_free, const char *type)
{
    ssize_t delta = after_free - before_free;
    printf("MALLOC_CAP_%s: Before %u bytes free, After %u bytes free (delta %d)\n", type, before_free, after_free, delta);
    TEST_ASSERT_MESSAGE(delta >= leak_threshold, "memory leak");
}

void setUp(void)
{
    // If heap tracing is enabled in kconfig, leak trace the test
#ifdef CONFIG_HEAP_TRACING
    setup_heap_record();
    heap_trace_start(HEAP_TRACE_LEAKS);
#endif

    leak_threshold = TEST_MEMORY_LEAK_THRESHOLD_DEFAULT;

#if SOC_KEY_MANAGER_SUPPORTED
    esp_crypto_ecc_lock_acquire();
    esp_crypto_sha_aes_lock_acquire();
    esp_crypto_ecc_lock_release();
    esp_crypto_sha_aes_lock_release();
    esp_crypto_key_manager_lock_acquire();
    esp_crypto_key_manager_lock_release();
    esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
#endif
    before_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    before_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
}

void tearDown(void)
{
    /*Give idle task time to clean up*/
    vTaskDelay(10);

#ifdef CONFIG_HEAP_TRACING
    heap_trace_stop();
    heap_trace_dump();
#endif

    size_t after_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t after_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
    check_leak(before_free_8bit, after_free_8bit, "8BIT");
    check_leak(before_free_32bit, after_free_32bit, "32BIT");

}

void app_main(void)
{
    vTaskPrioritySet(NULL, TEST_TASK_PRIORITY);
    ESP_LOGI(TAG, "Running crypto drivers test app");
    unity_run_menu();
}
