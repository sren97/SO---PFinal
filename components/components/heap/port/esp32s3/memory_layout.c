/*
 * SPDX-FileCopyrightText: 2019-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "soc/soc.h"
#include "soc/dport_reg.h"
#include "soc/tracemem_config.h"
#include "heap_memory_layout.h"
#include "esp_heap_caps.h"

/**
 * @brief Memory type descriptors. These describe the capabilities of a type of memory in the SoC.
 * Each type of memory map consists of one or more regions in the address space.
 * Each type contains an array of prioritized capabilities.
 * Types with later entries are only taken if earlier ones can't fulfill the memory request.
 *
 * - For a normal malloc (MALLOC_CAP_DEFAULT), give away the DRAM-only memory first, then pass off any dual-use IRAM regions, finally eat into the application memory.
 * - For a malloc where 32-bit-aligned-only access is okay, first allocate IRAM, then DRAM, finally application IRAM.
 * - Application mallocs (PIDx) will allocate IRAM first, if possible, then DRAM.
 * - Most other malloc caps only fit in one region anyway.
 *
 */

/* Index of memory in `soc_memory_types[]` */
enum {
    SOC_MEMORY_TYPE_DIRAM   = 0,
    SOC_MEMORY_TYPE_DRAM    = 1,
    SOC_MEMORY_TYPE_IRAM    = 2,
    SOC_MEMORY_TYPE_SPIRAM  = 3,
    SOC_MEMORY_TYPE_RTCRAM  = 4,
    SOC_MEMORY_TYPE_NUM,
};

/* COMMON_CAPS is the set of attributes common to all types of memory (except I/D cache data memory) on this chip */
#define ESP32S3_MEM_COMMON_CAPS (MALLOC_CAP_DEFAULT | MALLOC_CAP_32BIT | MALLOC_CAP_8BIT)


#ifdef CONFIG_ESP_SYSTEM_MEMPROT_FEATURE
#define MALLOC_DIRAM_BASE_CAPS      ESP32S3_MEM_COMMON_CAPS | MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_RETENTION
#define MALLOC_RTCRAM_BASE_CAPS     ESP32S3_MEM_COMMON_CAPS | MALLOC_CAP_INTERNAL
#else
#define MALLOC_DIRAM_BASE_CAPS      ESP32S3_MEM_COMMON_CAPS | MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_RETENTION | MALLOC_CAP_EXEC
#define MALLOC_RTCRAM_BASE_CAPS     ESP32S3_MEM_COMMON_CAPS | MALLOC_CAP_INTERNAL | MALLOC_CAP_EXEC
#endif

// The memory used for SIMD instructions requires the bus of its memory regions be able to transfer the data in 128-bit

/**
 * Defined the attributes and allocation priority of each memory on the chip,
 * The heap allocator will traverse all types of memory types in column High Priority Matching and match the specified caps at first,
 * if no memory caps matched or the allocation is failed, it will go to columns Medium Priority Matching and Low Priority Matching
 * in turn to continue matching.
 */
const soc_memory_type_desc_t soc_memory_types[SOC_MEMORY_TYPE_NUM] = {
/*                           Mem Type Name | High Priority Matching                    | Medium Priority Matching                                                         | Low Priority Matching */
    [SOC_MEMORY_TYPE_DIRAM]  = { "RAM",    { MALLOC_DIRAM_BASE_CAPS | MALLOC_CAP_SIMD,   0,                                                                                 0 }},
    [SOC_MEMORY_TYPE_DRAM]   = { "DRAM",   { 0,                                          ESP32S3_MEM_COMMON_CAPS | MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_SIMD,  0 }},
    [SOC_MEMORY_TYPE_IRAM]   = { "IRAM",   { MALLOC_CAP_EXEC,                            MALLOC_CAP_32BIT | MALLOC_CAP_INTERNAL,                                            0 }},
    [SOC_MEMORY_TYPE_SPIRAM] = { "SPIRAM", { MALLOC_CAP_SPIRAM,                          0,                                                                                 ESP32S3_MEM_COMMON_CAPS | MALLOC_CAP_SIMD }},
    [SOC_MEMORY_TYPE_RTCRAM] = { "RTCRAM", { MALLOC_CAP_RTCRAM,                          0,                                                                                 MALLOC_RTCRAM_BASE_CAPS }},
};

const size_t soc_memory_type_count = sizeof(soc_memory_types) / sizeof(soc_memory_type_desc_t);

/**
 * @brief Region descriptors. These describe all regions of memory available, and map them to a type in the above type.
 *
 * @note Because of requirements in the coalescing code which merges adjacent regions,
 *       this list should always be sorted from low to high by start address.
 *
 */

/**
 * Register the shared buffer area of the last memory block into the heap during heap initialization
 */
#define APP_USABLE_DRAM_END           (SOC_ROM_STACK_START - SOC_ROM_STACK_SIZE)

const soc_memory_region_t soc_memory_regions[] = {
#if CONFIG_ESP32S3_INSTRUCTION_CACHE_16KB && !defined(CONFIG_ESP_SYSTEM_MEMPROT_FEATURE)
    { 0x40374000,           0x4000,                                     SOC_MEMORY_TYPE_IRAM,   0,                                      false}, //Level 1, IRAM
#endif
    { 0x3FC88000,           0x8000,                                     SOC_MEMORY_TYPE_DIRAM,  0x40378000,                             false}, //Level 2, IDRAM, can be used as trace memory
    { 0x3FC90000,           0x10000,                                    SOC_MEMORY_TYPE_DIRAM,  0x40380000,                             false}, //Level 3, IDRAM, can be used as trace memory
    { 0x3FCA0000,           0x10000,                                    SOC_MEMORY_TYPE_DIRAM,  0x40390000,                             false}, //Level 4, IDRAM, can be used as trace memory
    { 0x3FCB0000,           0x10000,                                    SOC_MEMORY_TYPE_DIRAM,  0x403A0000,                             false}, //Level 5, IDRAM, can be used as trace memory
    { 0x3FCC0000,           0x10000,                                    SOC_MEMORY_TYPE_DIRAM,  0x403B0000,                             false}, //Level 6, IDRAM, can be used as trace memory
    { 0x3FCD0000,           0x10000,                                    SOC_MEMORY_TYPE_DIRAM,  0x403C0000,                             false}, //Level 7, IDRAM, can be used as trace memory
    { 0x3FCE0000,           (APP_USABLE_DRAM_END-0x3FCE0000),           SOC_MEMORY_TYPE_DIRAM,  0x403D0000,                             false}, //Level 8, IDRAM, can be used as trace memory,
    { APP_USABLE_DRAM_END,  (SOC_DIRAM_DRAM_HIGH-APP_USABLE_DRAM_END),  SOC_MEMORY_TYPE_DIRAM,  MAP_DRAM_TO_IRAM(APP_USABLE_DRAM_END),  true},  //Level 8, IDRAM, can be used as trace memory, ROM reserved area, recycled by heap allocator in app_main task
#if CONFIG_ESP32S3_DATA_CACHE_16KB || CONFIG_ESP32S3_DATA_CACHE_32KB
    { 0x3FCF0000,           0x8000,                                     SOC_MEMORY_TYPE_DRAM,   0,                                      false}, //Level 9, DRAM, DMA is accessible but retention DMA is inaccessible
#endif
#if CONFIG_ESP32S3_DATA_CACHE_16KB
    { 0x3C000000,           0x4000,                                     SOC_MEMORY_TYPE_DRAM,   0,                                      false}, //Level 10, DRAM, DMA is accessible but retention DMA is inaccessible
#endif
#ifdef CONFIG_SPIRAM
    { SOC_EXTRAM_DATA_LOW,  SOC_EXTRAM_DATA_SIZE,                       SOC_MEMORY_TYPE_SPIRAM, 0,                                      false}, //SPI SRAM, if available
#endif
#ifdef CONFIG_ESP_SYSTEM_ALLOW_RTC_FAST_MEM_AS_HEAP
    { 0x600fe000,           0x2000,                                     SOC_MEMORY_TYPE_RTCRAM, 0,                                      false}, //Fast RTC memory
#endif
};

const size_t soc_memory_region_count = sizeof(soc_memory_regions) / sizeof(soc_memory_region_t);

extern int _data_start, _heap_start, _iram_start, _iram_end, _rtc_force_fast_end, _rtc_noinit_end; // defined in sections.ld.in
extern int _rtc_reserved_start, _rtc_reserved_end;

/**
 * Reserved memory regions.
 * These are removed from the soc_memory_regions array when heaps are created.
 *
 */

// Static data region. DRAM used by data+bss and possibly rodata
SOC_RESERVE_MEMORY_REGION((intptr_t)&_data_start, (intptr_t)&_heap_start, dram_data);

// ESP32S3 has a big D/IRAM region, the part used by code is reserved
// The address of the D/I bus are in the same order, directly shift IRAM address to get reserved DRAM address
#define I_D_OFFSET (SOC_DIRAM_IRAM_LOW - SOC_DIRAM_DRAM_LOW)
// .text region in diram. DRAM used by text (shared with IBUS).
SOC_RESERVE_MEMORY_REGION((intptr_t)&_iram_start - I_D_OFFSET, (intptr_t)&_iram_end - I_D_OFFSET, iram_code);

#if CONFIG_ESP32S3_INSTRUCTION_CACHE_16KB
SOC_RESERVE_MEMORY_REGION((intptr_t)&_iram_start, (intptr_t)&_iram_end, iram_code_2);
#endif

#ifdef CONFIG_SPIRAM
/* Reserve the whole possible SPIRAM region here, spiram.c will add some or all of this
 * memory to heap depending on the actual SPIRAM chip size. */
SOC_RESERVE_MEMORY_REGION( SOC_EXTRAM_DATA_LOW, SOC_EXTRAM_DATA_HIGH, extram_data_region);
#endif

#if CONFIG_ESP32S3_TRACEMEM_RESERVE_DRAM > 0
SOC_RESERVE_MEMORY_REGION(TRACEMEM_BLK0_ADDR, TRACEMEM_BLK0_ADDR + CONFIG_ESP32S3_TRACEMEM_RESERVE_DRAM / 2, trace_mem0);
SOC_RESERVE_MEMORY_REGION(TRACEMEM_BLK1_ADDR, TRACEMEM_BLK1_ADDR + CONFIG_ESP32S3_TRACEMEM_RESERVE_DRAM / 2, trace_mem1);
#endif

// RTC Fast RAM region
#ifdef CONFIG_ESP_SYSTEM_ALLOW_RTC_FAST_MEM_AS_HEAP
#ifdef CONFIG_ESP32S3_RTCDATA_IN_FAST_MEM
SOC_RESERVE_MEMORY_REGION(SOC_RTC_DRAM_LOW, (intptr_t)&_rtc_noinit_end, rtcram_data);
#else
SOC_RESERVE_MEMORY_REGION(SOC_RTC_DRAM_LOW, (intptr_t)&_rtc_force_fast_end, rtcram_data);
#endif
#endif

SOC_RESERVE_MEMORY_REGION((intptr_t)&_rtc_reserved_start, (intptr_t)&_rtc_reserved_end, rtc_reserved_data);
