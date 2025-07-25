/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

// MSPI IOMUX PINs
#define MSPI_FUNC_NUM               0
#define MSPI_IOMUX_PIN_NUM_CS1      26
#define MSPI_IOMUX_PIN_NUM_HD       27
#define MSPI_IOMUX_PIN_NUM_WP       28
#define MSPI_IOMUX_PIN_NUM_CS0      29
#define MSPI_IOMUX_PIN_NUM_CLK      30
#define MSPI_IOMUX_PIN_NUM_MISO     31
#define MSPI_IOMUX_PIN_NUM_MOSI     32
#define MSPI_IOMUX_PIN_NUM_D4       33
#define MSPI_IOMUX_PIN_NUM_D5       34
#define MSPI_IOMUX_PIN_NUM_D6       35
#define MSPI_IOMUX_PIN_NUM_D7       36
#define MSPI_IOMUX_PIN_NUM_DQS      37

// There are 2 sets of GPIO pins which could be routed to FSPICS0, FSPICLK, FSPID, FSPIQ, FSPIHD, FSPIWP.
// However, there is only one set of GPIO pins which could be routed to FSPIIO4, FSPIIO5, FSPIIO6, FSPIIO7.
// As default (when we are not going to use Octal SPI), we make use of SPI2_FUNC_NUM to route one of the 2 sets of GPIO pins to FSPICS0 ~ FSPIWP as follows.
#define SPI2_FUNC_NUM               4
#define SPI2_IOMUX_PIN_NUM_HD       9
#define SPI2_IOMUX_PIN_NUM_CS       10
#define SPI2_IOMUX_PIN_NUM_MOSI     11
#define SPI2_IOMUX_PIN_NUM_CLK      12
#define SPI2_IOMUX_PIN_NUM_MISO     13
#define SPI2_IOMUX_PIN_NUM_WP       14

// When using Octal SPI, we make use of SPI2_FUNC_NUM_OCT to route them as follows.
#define SPI2_FUNC_NUM_OCT           2
#define SPI2_IOMUX_PIN_NUM_HD_OCT   33
#define SPI2_IOMUX_PIN_NUM_CS_OCT   34
#define SPI2_IOMUX_PIN_NUM_MOSI_OCT 35
#define SPI2_IOMUX_PIN_NUM_CLK_OCT  36
#define SPI2_IOMUX_PIN_NUM_MISO_OCT 37
#define SPI2_IOMUX_PIN_NUM_WP_OCT   38
#define SPI2_IOMUX_PIN_NUM_IO4_OCT  10
#define SPI2_IOMUX_PIN_NUM_IO5_OCT  11
#define SPI2_IOMUX_PIN_NUM_IO6_OCT  12
#define SPI2_IOMUX_PIN_NUM_IO7_OCT  13

//SPI3 has no iomux pins

//Following Macros are deprecated. Please use the Macros above
#define FSPI_FUNC_NUM           SPI2_FUNC_NUM
#define FSPI_IOMUX_PIN_NUM_HD   SPI2_IOMUX_PIN_NUM_HD
#define FSPI_IOMUX_PIN_NUM_CS   SPI2_IOMUX_PIN_NUM_CS
#define FSPI_IOMUX_PIN_NUM_MOSI SPI2_IOMUX_PIN_NUM_MOSI
#define FSPI_IOMUX_PIN_NUM_CLK  SPI2_IOMUX_PIN_NUM_CLK
#define FSPI_IOMUX_PIN_NUM_MISO SPI2_IOMUX_PIN_NUM_MISO
#define FSPI_IOMUX_PIN_NUM_WP   SPI2_IOMUX_PIN_NUM_WP
