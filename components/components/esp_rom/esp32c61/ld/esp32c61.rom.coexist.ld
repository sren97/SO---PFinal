/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* ROM function interface esp32c61.rom.coexist.ld for esp32c61
 *
 *
 * Generated from ./target/esp32c6lite/interface-esp32c6lite.yml md5sum 27eb0efac0883ee622c22767242c9457
 *
 * Compatible with ROM where ECO version equal or greater to 0.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED. DO NOT EDIT.
 */

/***************************************
 Group rom_coexist
 ***************************************/

/* Functions */
esp_coex_rom_version_get = 0x40000a30;
coex_bt_release = 0x40000a34;
coex_bt_request = 0x40000a38;
coex_core_ble_conn_dyn_prio_get = 0x40000a3c;
coex_core_event_duration_get = 0x40000a40;
coex_core_pti_get = 0x40000a44;
coex_core_release = 0x40000a48;
coex_core_request = 0x40000a4c;
coex_core_status_get = 0x40000a50;
coex_core_timer_idx_get = 0x40000a54;
coex_event_duration_get = 0x40000a58;
coex_hw_timer_disable = 0x40000a5c;
coex_hw_timer_enable = 0x40000a60;
coex_hw_timer_set = 0x40000a64;
coex_schm_interval_set = 0x40000a68;
coex_schm_lock = 0x40000a6c;
coex_schm_unlock = 0x40000a70;
/*coex_status_get = 0x40000a74;*/
coex_wifi_release = 0x40000a78;
esp_coex_ble_conn_dynamic_prio_get = 0x40000a7c;
/* Data (.data, .bss, .rodata) */
coex_env_ptr = 0x4084ffc4;
coex_pti_tab_ptr = 0x4084ffc0;
coex_schm_env_ptr = 0x4084ffbc;
coexist_funcs = 0x4084ffb8;
g_coa_funcs_p = 0x4084ffb4;
g_coex_param_ptr = 0x4084ffb0;
