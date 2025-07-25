/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#warning "This file is deprecated, as __containerof is available via libbsd. To use __containerof, include string.h."

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_IDF_TARGET_LINUX && !defined(__containerof)
#define __containerof(ptr, type, member) ({         \
    const typeof( ((type *)0)->member ) *__mptr = (ptr); \
    (type *)( (char *)__mptr - offsetof(type,member) );})
#endif

#ifdef __cplusplus
}
#endif
