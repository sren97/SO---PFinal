/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc/touch_sensor_channel.h"

/* Store IO number corresponding to the Touch Sensor channel number. */
const int touch_sensor_channel_io_map[] = {
    TOUCH_PAD_NUM0_GPIO_NUM,
    TOUCH_PAD_NUM1_GPIO_NUM,
    TOUCH_PAD_NUM2_GPIO_NUM,
    TOUCH_PAD_NUM3_GPIO_NUM,
    TOUCH_PAD_NUM4_GPIO_NUM,
    TOUCH_PAD_NUM5_GPIO_NUM,
    TOUCH_PAD_NUM6_GPIO_NUM,
    TOUCH_PAD_NUM7_GPIO_NUM,
    TOUCH_PAD_NUM8_GPIO_NUM,
    TOUCH_PAD_NUM9_GPIO_NUM
};
