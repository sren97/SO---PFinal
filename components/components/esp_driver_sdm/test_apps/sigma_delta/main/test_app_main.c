/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "unity.h"
#include "unity_test_utils.h"
#include "esp_heap_caps.h"

// Some resources are lazy allocated in the driver, the threshold is left for that case
#define TEST_MEMORY_LEAK_THRESHOLD (300)

static size_t before_free_8bit;
static size_t before_free_32bit;

void setUp(void)
{
    before_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    before_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
}

void tearDown(void)
{
    size_t after_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t after_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
    printf("\n");
    unity_utils_check_leak(before_free_8bit, after_free_8bit, "8BIT", TEST_MEMORY_LEAK_THRESHOLD);
    unity_utils_check_leak(before_free_32bit, after_free_32bit, "32BIT", TEST_MEMORY_LEAK_THRESHOLD);
}

void app_main(void)
{
    //  ____  ____  __  __   _____         _
    // / ___||  _ \|  \/  | |_   _|__  ___| |_
    // \___ \| | | | |\/| |   | |/ _ \/ __| __|
    //  ___) | |_| | |  | |   | |  __/\__ \ |_
    // |____/|____/|_|  |_|   |_|\___||___/\__|
    printf("  ____  ____  __  __   _____         _   \r\n");
    printf(" / ___||  _ \\|  \\/  | |_   _|__  ___| |_ \r\n");
    printf(" \\___ \\| | | | |\\/| |   | |/ _ \\/ __| __|\r\n");
    printf("  ___) | |_| | |  | |   | |  __/\\__ \\ |_ \r\n");
    printf(" |____/|____/|_|  |_|   |_|\\___||___/\\__|\r\n");
    unity_run_menu();
}
