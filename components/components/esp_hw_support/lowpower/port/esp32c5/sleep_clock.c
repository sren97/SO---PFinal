/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_private/sleep_clock.h"
#include "soc/pcr_reg.h"
#include "soc/pmu_reg.h"
#include "soc/rtc.h"
#include "modem/modem_syscon_reg.h"
#include "modem/modem_lpcon_reg.h"
#include "soc/i2c_ana_mst_reg.h"
#include "soc/regi2c_defs.h"
#include "soc/chip_revision.h"
#include "hal/efuse_hal.h"

static const char *TAG = "sleep_clock";

esp_err_t sleep_clock_system_retention_init(void *arg)
{
    const static sleep_retention_entries_config_t pcr_regs_retention[] = {
        /* Enable i2c master clock */
        [0] = { .config = REGDMA_LINK_WRITE_INIT(REGDMA_PCR_LINK(0),    MODEM_LPCON_CLK_CONF_REG,   MODEM_LPCON_CLK_I2C_MST_EN,     MODEM_LPCON_CLK_I2C_MST_EN_M,   1, 0), .owner = ENTRY(0) },
        /* Start BBPLL self-calibration */
        [1] = { .config = REGDMA_LINK_WRITE_INIT(REGDMA_PCR_LINK(1),    I2C_ANA_MST_ANA_CONF0_REG,  0,                              I2C_MST_BBPLL_STOP_FORCE_HIGH,  1, 0), .owner = ENTRY(0) },
        [2] = { .config = REGDMA_LINK_WRITE_INIT(REGDMA_PCR_LINK(2),    I2C_ANA_MST_ANA_CONF0_REG,  I2C_MST_BBPLL_STOP_FORCE_LOW,   I2C_MST_BBPLL_STOP_FORCE_LOW,   1, 0), .owner = ENTRY(0) },
        /* Wait calibration done */
        [3] = { .config = REGDMA_LINK_WAIT_INIT (REGDMA_PCR_LINK(3),    I2C_ANA_MST_ANA_CONF0_REG,  I2C_MST_BBPLL_CAL_DONE,         I2C_MST_BBPLL_CAL_DONE,         1, 0), .owner = ENTRY(0) },
        /* Stop BBPLL self-calibration */
        [4] = { .config = REGDMA_LINK_WRITE_INIT(REGDMA_PCR_LINK(4),    I2C_ANA_MST_ANA_CONF0_REG,  0,                              I2C_MST_BBPLL_STOP_FORCE_LOW,   1, 0), .owner = ENTRY(0) },
        [5] = { .config = REGDMA_LINK_WRITE_INIT(REGDMA_PCR_LINK(5),    I2C_ANA_MST_ANA_CONF0_REG,  I2C_MST_BBPLL_STOP_FORCE_HIGH,  I2C_MST_BBPLL_STOP_FORCE_HIGH,  1, 0), .owner = ENTRY(0) },
        /* Clock configuration retention */
        [6] = { .config = REGDMA_LINK_WAIT_INIT (REGDMA_PCR_LINK(6),    PMU_CLK_STATE0_REG,         PMU_STABLE_XPD_BBPLL_STATE,     PMU_STABLE_XPD_BBPLL_STATE_M,   1, 0),  .owner = ENTRY(0)},             /* Wait PMU_WAIT_XTL_STABLE done */
        [7] = { .config = REGDMA_LINK_WRITE_INIT(REGDMA_PCR_LINK(7),    PCR_AHB_FREQ_CONF_REG,      0,                              PCR_AHB_DIV_NUM,                1, 0),  .owner = ENTRY(0) | ENTRY(1) }, /* Set AHB bus frequency to XTAL frequency */
        [8] = { .config = REGDMA_LINK_WRITE_INIT(REGDMA_PCR_LINK(8),    PCR_BUS_CLK_UPDATE_REG,     1,                              PCR_BUS_CLOCK_UPDATE,           1, 0),  .owner = ENTRY(0) | ENTRY(1) },
#if CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
        [9] = { .config = REGDMA_LINK_ADDR_MAP_INIT(REGDMA_PCR_LINK(9), DR_REG_PCR_BASE,            DR_REG_PCR_BASE,    75, 0, 0,   0xffffffff, 0xffffffff, 0x200007f7, 0x0), .owner = ENTRY(0) | ENTRY(1) },
#endif
    };
    esp_err_t err = sleep_retention_entries_create(pcr_regs_retention, ARRAY_SIZE(pcr_regs_retention), REGDMA_LINK_PRI_SYS_CLK, SLEEP_RETENTION_MODULE_CLOCK_SYSTEM);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to allocate memory for system (PCR) retention");

    const static sleep_retention_entries_config_t modem_ahb_config[] = {
        [0] = { .config = REGDMA_LINK_WRITE_INIT (REGDMA_PCR_LINK(4), PCR_AHB_FREQ_CONF_REG,  3, PCR_AHB_DIV_NUM,      1, 0), .owner = ENTRY(1) }, /* Set AHB bus frequency to 40 MHz under PMU MODEM state */
        [1] = { .config = REGDMA_LINK_WRITE_INIT (REGDMA_PCR_LINK(5), PCR_BUS_CLK_UPDATE_REG, 1, PCR_BUS_CLOCK_UPDATE, 1, 0), .owner = ENTRY(1) },
    };
    err = sleep_retention_entries_create(modem_ahb_config, ARRAY_SIZE(modem_ahb_config), REGDMA_LINK_PRI_4, SLEEP_RETENTION_MODULE_CLOCK_SYSTEM);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to allocate memory for system (PCR) retention, 4 level priority");

#if SOC_PM_SUPPORT_PMU_MODEM_STATE && CONFIG_ESP_WIFI_ENHANCED_LIGHT_SLEEP && CONFIG_XTAL_FREQ_AUTO
    uint32_t xtal_freq_mhz = (uint32_t)rtc_clk_xtal_freq_get();
    if (xtal_freq_mhz == SOC_XTAL_FREQ_48M) {
        /* For the 48 MHz main XTAL, we need regdma to configured BBPLL by exec
         * the PHY_I2C_MST_CMD_TYPE_BBPLL_CFG command from PHY i2c master
         * command memory */
        sleep_retention_entries_config_t bbpll_config[] = {
            [0] = { .config = REGDMA_LINK_WRITE_INIT   (REGDMA_PCR_LINK(6), MODEM_LPCON_CLK_CONF_REG,         MODEM_LPCON_CLK_I2C_MST_EN, MODEM_LPCON_CLK_I2C_MST_EN_M, 1, 0), .owner = ENTRY(1) }, /* I2C MST enable */
            [1] = { .config = REGDMA_LINK_WRITE_INIT   (REGDMA_PCR_LINK(7), I2C_ANA_MST_I2C_BURST_CONF_REG,   0,                          0xffffffff,                   1, 0), .owner = ENTRY(1) },
            [2] = { .config = REGDMA_LINK_WAIT_INIT    (REGDMA_PCR_LINK(8), I2C_ANA_MST_I2C_BURST_STATUS_REG, I2C_ANA_MST_BURST_DONE,     0x1,                          1, 0), .owner = ENTRY(1) },
            [3] = { .config = REGDMA_LINK_WRITE_INIT   (REGDMA_PCR_LINK(9), MODEM_LPCON_CLK_CONF_REG,         0,                          MODEM_LPCON_CLK_I2C_MST_EN_M, 1, 0), .owner = ENTRY(1) }, /* I2C MST disable */
        };
        extern uint32_t phy_ana_i2c_master_burst_bbpll_config(void);
        bbpll_config[1].config.write_wait.value = phy_ana_i2c_master_burst_bbpll_config();
        err = sleep_retention_entries_create(bbpll_config, ARRAY_SIZE(bbpll_config), REGDMA_LINK_PRI_SYS_CLK, SLEEP_RETENTION_MODULE_CLOCK_SYSTEM);
        ESP_RETURN_ON_ERROR(err, TAG, "failed to allocate memory for bbpll configure, 0 level priority");
    }
#endif

#if CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
    /* On ESP32-C5 ECO1, clearing BIT(31) of PCR_FPGA_DEBUG_REG (it's located
     * in TOP domain) is used to fix the issue where the modem module fails
     * to transmit and receive packets due to the loss of The modem root clock
     * caused by automatic clock gating during soc root clock source switching.
     * For detailed information, refer to IDF-11064 */
    if (ESP_CHIP_REV_ABOVE(efuse_hal_chip_revision(), 1)) {
        const static sleep_retention_entries_config_t rootclk_workaround[] = {
            [0] = { .config = REGDMA_LINK_CONTINUOUS_INIT(REGDMA_PCR_LINK(9), PCR_FPGA_DEBUG_REG, PCR_FPGA_DEBUG_REG, 1, 0, 0), .owner = ENTRY(0) | ENTRY(1) }
        };
        err = sleep_retention_entries_create(rootclk_workaround, ARRAY_SIZE(rootclk_workaround), 1, SLEEP_RETENTION_MODULE_CLOCK_SYSTEM);
        ESP_RETURN_ON_ERROR(err, TAG, "failed to allocate memory for modem root clock workaround, 1 level priority");
    }
#endif

    ESP_LOGI(TAG, "System Power, Clock and Reset sleep retention initialization");
    return ESP_OK;
}

#if CONFIG_MAC_BB_PD || CONFIG_BT_LE_SLEEP_ENABLE || CONFIG_IEEE802154_SLEEP_ENABLE
esp_err_t sleep_clock_modem_retention_init(void *arg)
{
    #define N_REGS_SYSCON() (((MODEM_SYSCON_MEM_RF2_CONF_REG - MODEM_SYSCON_TEST_CONF_REG) / 4) + 1)

    const static sleep_retention_entries_config_t modem_regs_retention[] = {
        [0] = { .config = REGDMA_LINK_CONTINUOUS_INIT(REGDMA_MODEMSYSCON_LINK(0), MODEM_SYSCON_TEST_CONF_REG, MODEM_SYSCON_TEST_CONF_REG, N_REGS_SYSCON(), 0, 0), .owner = ENTRY(0) | ENTRY(1) }, /* MODEM SYSCON */
    };

    esp_err_t err = sleep_retention_entries_create(modem_regs_retention, ARRAY_SIZE(modem_regs_retention), REGDMA_LINK_PRI_MODEM_CLK, SLEEP_RETENTION_MODULE_CLOCK_MODEM);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to allocate memory for modem (SYSCON) retention, 1 level priority");
    ESP_LOGI(TAG, "Modem Power, Clock and Reset sleep retention initialization");
    return ESP_OK;

    #undef N_REGS_SYSCON
}
#endif

bool clock_domain_pd_allowed(void)
{
    const sleep_retention_module_bitmap_t inited_modules = sleep_retention_get_inited_modules();
    const sleep_retention_module_bitmap_t created_modules = sleep_retention_get_created_modules();
    const sleep_retention_module_bitmap_t sys_clk_dep_modules = (sleep_retention_module_bitmap_t){ .bitmap[SLEEP_RETENTION_MODULE_SYS_PERIPH >> 5] = BIT(SLEEP_RETENTION_MODULE_SYS_PERIPH % 32) };

    /* The clock and reset of MODEM (WiFi, BLE and 15.4) modules are managed
     * through MODEM_SYSCON, when one or more MODEMs are initialized, it is
     * necessary to check the state of CLOCK_MODEM to determine MODEM domain on
     * or off. The clock and reset of digital peripherals are managed through
     * PCR, with TOP domain similar to MODEM domain. */
    sleep_retention_module_bitmap_t modem_clk_dep_modules = (sleep_retention_module_bitmap_t){ .bitmap = { 0 } };
#if SOC_WIFI_SUPPORTED
    modem_clk_dep_modules.bitmap[SLEEP_RETENTION_MODULE_WIFI_MAC >> 5] |= BIT(SLEEP_RETENTION_MODULE_WIFI_MAC % 32);
    modem_clk_dep_modules.bitmap[SLEEP_RETENTION_MODULE_WIFI_BB >> 5] |= BIT(SLEEP_RETENTION_MODULE_WIFI_BB % 32);
#endif
#if SOC_BT_SUPPORTED
    modem_clk_dep_modules.bitmap[SLEEP_RETENTION_MODULE_BLE_MAC >> 5] |= BIT(SLEEP_RETENTION_MODULE_BLE_MAC % 32);
    modem_clk_dep_modules.bitmap[SLEEP_RETENTION_MODULE_BT_BB >> 5] |= BIT(SLEEP_RETENTION_MODULE_BT_BB % 32);
#endif
#if SOC_IEEE802154_SUPPORTED
    modem_clk_dep_modules.bitmap[SLEEP_RETENTION_MODULE_802154_MAC >> 5] |= BIT(SLEEP_RETENTION_MODULE_802154_MAC % 32);
    modem_clk_dep_modules.bitmap[SLEEP_RETENTION_MODULE_BT_BB >> 5] |= BIT(SLEEP_RETENTION_MODULE_BT_BB % 32);
#endif

    const sleep_retention_module_bitmap_t null_module = (sleep_retention_module_bitmap_t){ .bitmap = { 0 } };

    sleep_retention_module_bitmap_t mask = (sleep_retention_module_bitmap_t){ .bitmap = { 0 } };
    const sleep_retention_module_bitmap_t system_modules = sleep_retention_module_bitmap_and(inited_modules, sys_clk_dep_modules);
    if (!sleep_retention_module_bitmap_eq(system_modules, null_module)) {
        mask.bitmap[SLEEP_RETENTION_MODULE_CLOCK_SYSTEM >> 5] |= BIT(SLEEP_RETENTION_MODULE_CLOCK_SYSTEM % 32);
    }
#if SOC_WIFI_SUPPORTED || SOC_BT_SUPPORTED || SOC_IEEE802154_SUPPORTED
    const sleep_retention_module_bitmap_t modem_modules = sleep_retention_module_bitmap_and(inited_modules, modem_clk_dep_modules);
    if (!sleep_retention_module_bitmap_eq(modem_modules, null_module)) {
        mask.bitmap[SLEEP_RETENTION_MODULE_CLOCK_MODEM >> 5] |= BIT(SLEEP_RETENTION_MODULE_CLOCK_MODEM % 32);
    }
#endif

    const sleep_retention_module_bitmap_t clock_domain_inited_modules = sleep_retention_module_bitmap_and(inited_modules, mask);
    const sleep_retention_module_bitmap_t clock_domain_created_modules = sleep_retention_module_bitmap_and(created_modules, mask);
    return sleep_retention_module_bitmap_eq(clock_domain_inited_modules, clock_domain_created_modules);
}

ESP_SYSTEM_INIT_FN(sleep_clock_startup_init, SECONDARY, BIT(0), 106)
{
    sleep_retention_module_init_param_t init_param = {
        .cbs       = { .create = { .handle = sleep_clock_system_retention_init, .arg = NULL } },
        .attribute = SLEEP_RETENTION_MODULE_ATTR_PASSIVE
    };
    sleep_retention_module_init(SLEEP_RETENTION_MODULE_CLOCK_SYSTEM, &init_param);

#if CONFIG_MAC_BB_PD || CONFIG_BT_LE_SLEEP_ENABLE || CONFIG_IEEE802154_SLEEP_ENABLE
    init_param = (sleep_retention_module_init_param_t) {
        .cbs       = { .create = { .handle = sleep_clock_modem_retention_init, .arg = NULL } },
        .depends.bitmap[SLEEP_RETENTION_MODULE_CLOCK_SYSTEM >> 5] = BIT(SLEEP_RETENTION_MODULE_CLOCK_SYSTEM % 32),
        .attribute = SLEEP_RETENTION_MODULE_ATTR_PASSIVE
    };
    sleep_retention_module_init(SLEEP_RETENTION_MODULE_CLOCK_MODEM, &init_param);
#endif
    return ESP_OK;
}
