/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _TEST_COMMON_SPI_H_
#define _TEST_COMMON_SPI_H_

#include <esp_types.h>
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "unity.h"
#include "param_test.h"
#include "soc/io_mux_reg.h"
#include "sdkconfig.h"
#include "soc/spi_periph.h"
#include "driver/spi_master.h"
#include "test_dualboard_utils.h"

// All the tests using the header should use this definition as much as possible,
// so that the working host can be changed easily in the future.

#define TEST_SPI_PERIPH_NUM     (SOC_SPI_PERIPH_NUM - 1)

#if CONFIG_IDF_TARGET_ESP32C6   // cs_pin conflict with uart pin
#define PIN_NUM_MISO            SPI2_IOMUX_PIN_NUM_MISO
#define PIN_NUM_MOSI            SPI2_IOMUX_PIN_NUM_MOSI
#define PIN_NUM_CLK             SPI2_IOMUX_PIN_NUM_CLK
#define PIN_NUM_CS              10  //the IOMUX pin of SPI2 CS0&CS1 is Pin_16&17 which conflict with UART Tx&Rx Pin
#define PIN_NUM_WP              SPI2_IOMUX_PIN_NUM_WP
#define PIN_NUM_HD              SPI2_IOMUX_PIN_NUM_HD
#else
#define PIN_NUM_MISO            SPI2_IOMUX_PIN_NUM_MISO
#define PIN_NUM_MOSI            SPI2_IOMUX_PIN_NUM_MOSI
#define PIN_NUM_CLK             SPI2_IOMUX_PIN_NUM_CLK
#define PIN_NUM_CS              SPI2_IOMUX_PIN_NUM_CS
#define PIN_NUM_WP              SPI2_IOMUX_PIN_NUM_WP
#define PIN_NUM_HD              SPI2_IOMUX_PIN_NUM_HD
#endif

#if (TEST_SPI_PERIPH_NUM >= 2)  // esp32, s2, s3
#define TEST_SPI_HOST           SPI2_HOST
#define TEST_SLAVE_HOST         SPI3_HOST

#define MASTER_IOMUX_PIN_MISO   SPI2_IOMUX_PIN_NUM_MISO
#define MASTER_IOMUX_PIN_MOSI   SPI2_IOMUX_PIN_NUM_MOSI
#define MASTER_IOMUX_PIN_SCLK   SPI2_IOMUX_PIN_NUM_CLK
#define MASTER_IOMUX_PIN_CS     SPI2_IOMUX_PIN_NUM_CS
#define MASTER_IOMUX_PIN_WP     SPI2_IOMUX_PIN_NUM_WP
#define MASTER_IOMUX_PIN_HD     SPI2_IOMUX_PIN_NUM_HD
#else
#define TEST_SPI_HOST           SPI2_HOST
#define TEST_SLAVE_HOST         SPI2_HOST
#endif

#if CONFIG_IDF_TARGET_ESP32     // spi3 have iomux pin only on esp32
#define SLAVE_IOMUX_PIN_MISO    SPI3_IOMUX_PIN_NUM_MISO
#define SLAVE_IOMUX_PIN_MOSI    SPI3_IOMUX_PIN_NUM_MOSI
#define SLAVE_IOMUX_PIN_SCLK    SPI3_IOMUX_PIN_NUM_CLK
#define SLAVE_IOMUX_PIN_CS      SPI3_IOMUX_PIN_NUM_CS
#define SLAVE_IOMUX_PIN_WP      SPI3_IOMUX_PIN_NUM_WP
#define SLAVE_IOMUX_PIN_HD      SPI3_IOMUX_PIN_NUM_HD

#define UNCONNECTED_PIN         27
#define INPUT_ONLY_PIN          34
#define GPIO_DELAY              (12.5*2)
#define ESP_SPI_SLAVE_TV        (12.5*3.5)
#define WIRE_DELAY              12.5

#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
#define SLAVE_IOMUX_PIN_MISO    -1
#define SLAVE_IOMUX_PIN_MOSI    -1
#define SLAVE_IOMUX_PIN_SCLK    -1
#define SLAVE_IOMUX_PIN_CS      -1
#define SLAVE_IOMUX_PIN_WP      -1
#define SLAVE_IOMUX_PIN_HD      -1

#define UNCONNECTED_PIN         41
#define INPUT_ONLY_PIN          46
#define GPIO_DELAY              0
#define ESP_SPI_SLAVE_TV        0
#define WIRE_DELAY              12.5

#else
#define GPIO_DELAY              0
#define ESP_SPI_SLAVE_TV        0
#define WIRE_DELAY              12.5
#endif  //CONFIG_IDF_TARGET_ESP32

#define FUNC_SPI    SPI2_FUNC_NUM
#define FUNC_GPIO   PIN_FUNC_GPIO

//Delay information
#define TV_INT_CONNECT_GPIO     (ESP_SPI_SLAVE_TV+GPIO_DELAY)
#define TV_INT_CONNECT          (ESP_SPI_SLAVE_TV)
//when connecting to another board, the delay is usually increased by 12.5ns
#define TV_WITH_ESP_SLAVE_GPIO  (TV_INT_CONNECT_GPIO+WIRE_DELAY)
#define TV_WITH_ESP_SLAVE       (TV_INT_CONNECT+WIRE_DELAY)

//currently ESP32 slave only supports up to 20MHz, but 40MHz on the same board
#define ESP_SPI_SLAVE_MAX_FREQ      20 * 1000 * 1000
#define ESP_SPI_SLAVE_MAX_FREQ_SYNC 40 * 1000 * 1000

#define TEST_DMA_MAX_SIZE       4092///< length of each transaction with dma
#define MAX_TEST_SIZE           16  ///< in this test we run several transactions, this is the maximum trans that can be run
#define PSET_NAME_LEN           30  ///< length of each param set name

//test low frequency, high frequency until freq limit for worst case (both GPIO)
#define TEST_FREQ_DEFAULT(){    \
         1 * 1000 * 1000,       \
         8 * 1000 * 1000,       \
         9 * 1000 * 1000,       \
        10 * 1000 * 1000,       \
        11 * 1000 * 1000,       \
        13 * 1000 * 1000,       \
        16 * 1000 * 1000,       \
        20 * 1000 * 1000,       \
        26 * 1000 * 1000,       \
        40 * 1000 * 1000,       \
        80 * 1000 * 1000,       \
        0,\
    }

//default bus config for tests
#define SPI_BUS_TEST_DEFAULT_CONFIG() {\
        .miso_io_num=PIN_NUM_MISO, \
        .mosi_io_num=PIN_NUM_MOSI,\
        .sclk_io_num=PIN_NUM_CLK,\
        .quadwp_io_num=-1,\
        .quadhd_io_num=-1\
    }

//default device config for master devices
#define SPI_DEVICE_TEST_DEFAULT_CONFIG()    {\
        .clock_speed_hz=10*1000*1000,\
        .mode=0,\
        .spics_io_num=PIN_NUM_CS,\
        .queue_size=16,\
        .pre_cb=NULL,  \
        .cs_ena_pretrans = 0,\
        .cs_ena_posttrans = 0,\
        .input_delay_ns = 62.5,\
    }

//default device config for slave devices
#define SPI_SLAVE_TEST_DEFAULT_CONFIG() {\
        .mode=0,\
        .spics_io_num=PIN_NUM_CS,\
        .queue_size=3,\
        .flags=0,\
    }

//default device config for slave hd devices
#define SPI_SLOT_TEST_DEFAULT_CONFIG() {\
        .spics_io_num = PIN_NUM_CS, \
        .flags = 0, \
        .mode = 0, \
        .command_bits = 8,\
        .address_bits = 8,\
        .dummy_bits = 8,\
        .queue_size = 10,\
    }

typedef enum {
    FULL_DUPLEX = 0,
    HALF_DUPLEX_MISO = 1,
    HALF_DUPLEX_MOSI = 2,
} spi_dup_t;

/*-------- slave task related stuff -----------*/
typedef struct {
    uint32_t len;
    uint8_t* tx_start;
    uint8_t data[1];
} slave_rxdata_t;

typedef struct {
    uint32_t len;
    const uint8_t *start;
} slave_txdata_t;

typedef struct {
    spi_host_device_t spi;
    RingbufHandle_t data_received;
    QueueHandle_t data_to_send;
} spi_slave_task_context_t;

// test data for master and slave
extern uint8_t spitest_master_send[];
extern uint8_t spitest_slave_send[];

//tags for master and slave app
extern const char MASTER_TAG[];
extern const char SLAVE_TAG[];

//parameter set definition
typedef struct {
    const char pset_name[PSET_NAME_LEN];
    /*The test work till the frequency below,
     *set the frequency to higher and remove checks in the driver to know how fast the system can run.
     */
    const int *freq_list;     // list of tested frequency, terminated by 0
    int freq_limit;     //freq larger (not equal) than this will be ignored
    spi_dup_t dup;
    int mode;
    bool length_aligned;
    int test_size;

    int master_limit;   // the master disable dummy bits and discard readings over this freq
    bool master_iomux;
    int master_dma_chan;

    bool slave_iomux;
    int slave_dma_chan;
    int slave_tv_ns;
    bool slave_unaligned_addr;
} spitest_param_set_t;

//context definition for the parameterized test
typedef struct {
    uint8_t master_rxbuf[480];
    spi_transaction_t master_trans[MAX_TEST_SIZE];
    TaskHandle_t handle_slave;
    spi_slave_task_context_t slave_context;
    slave_txdata_t slave_trans[MAX_TEST_SIZE];
} spitest_context_t;

// fill default value of spitest_param_set_t
void spitest_def_param(void* arg);

// functions for slave task
esp_err_t init_slave_context(spi_slave_task_context_t *context, spi_host_device_t host);
void deinit_slave_context(spi_slave_task_context_t *context);
void spitest_slave_task(void* arg);

//called by slave, pull-up all pins used by slave
void slave_pull_up(const spi_bus_config_t* cfg, int spics_io_num);

// to access data of pre-defined transactions.
void spitest_init_transactions(const spitest_param_set_t *cfg, spitest_context_t* context);

// print data from a transaction
void spitest_master_print_data(spi_transaction_t *t, int rxlength);
void spitest_slave_print_data(slave_rxdata_t *t, bool print_rxdata);
// Check whether master and slave data match
esp_err_t spitest_check_data(int len, spi_transaction_t *master_t, slave_rxdata_t *slave_t, bool check_master_data, bool check_slave_len, bool check_slave_data);

#define spitest_cmp_or_dump(expected, actual, len) ({\
    int r = memcmp(expected, actual, len);\
    if (r != 0) {\
        ESP_LOG_BUFFER_HEXDUMP("actual ", actual, len, ESP_LOG_WARN);\
        ESP_LOG_BUFFER_HEXDUMP("expected", expected, len, ESP_LOG_INFO);\
        TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, actual, len);\
    }\
    r;\
})

static inline int get_trans_len(spi_dup_t dup, spi_transaction_t *master_t)
{
    if (dup != HALF_DUPLEX_MISO) {
        return master_t->length;
    } else {
        return master_t->rxlength;
    }
}
//remove device from bus and free the bus
void master_free_device_bus(spi_device_handle_t spi);

//use this function to fix the output source when assign multiple functions to a same pin
void spitest_gpio_output_sel(uint32_t gpio_num, int func, uint32_t signal_idx);

//use this function to fix the input source when assign multiple functions to a same pin
void spitest_gpio_input_sel(uint32_t gpio_num, int func, uint32_t signal_idx);

//Note this cs_num is the ID of the connected devices' ID, e.g. if 2 devices are connected to the bus,
//then the cs_num of the 1st and 2nd devices are 0 and 1 respectively.
void same_pin_func_sel(spi_bus_config_t bus, spi_device_interface_config_t dev, uint8_t cs_num);

#endif  //_TEST_COMMON_SPI_H_
