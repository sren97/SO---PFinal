/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BTC_GATTC_H__
#define __BTC_GATTC_H__

#include "btc/btc_task.h"
#include "esp_bt_defs.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"

typedef enum {
    BTC_GATTC_ACT_APP_REGISTER = 0,
    BTC_GATTC_ACT_APP_UNREGISTER,
    BTC_GATTC_ACT_OPEN,
#if (BLE_50_FEATURE_SUPPORT == TRUE)
    BTC_GATTC_ACT_AUX_OPEN,
#endif // #if (BLE_50_FEATURE_SUPPORT == TRUE)
    BTC_GATTC_ACT_CLOSE,
    BTC_GATTC_ACT_CFG_MTU,
    BTC_GATTC_ACT_SEARCH_SERVICE,
    BTC_GATTC_ACT_READ_CHAR,
    BTC_GATTC_ACT_READ_MULTIPLE_CHAR,
    BTC_GATTC_ACT_READ_MULTIPLE_VARIABLE_CHAR,
    BTC_GATTC_ACT_READ_CHAR_DESCR,
    BTC_GATTC_ACT_READ_BY_TYPE,
    BTC_GATTC_ACT_WRITE_CHAR,
    BTC_GATTC_ACT_WRITE_CHAR_DESCR,
    BTC_GATTC_ACT_PREPARE_WRITE,
    BTC_GATTC_ACT_PREPARE_WRITE_CHAR_DESCR,
    BTC_GATTC_ACT_EXECUTE_WRITE,
    BTC_GATTC_ACT_REG_FOR_NOTIFY,
    BTC_GATTC_ACT_UNREG_FOR_NOTIFY,
    BTC_GATTC_ACT_CACHE_REFRESH,
    BTC_GATTC_ACT_CACHE_ASSOC,
    BTC_GATTC_ATC_CACHE_GET_ADDR_LIST,
    BTC_GATTC_ACT_CACHE_CLEAN,
} btc_gattc_act_t;

/* btc_ble_gattc_args_t */
typedef union {
    //BTC_GATTC_ACT_APP_REGISTER,
    struct app_reg_arg {
        uint16_t app_id;
    } app_reg;
    //BTC_GATTC_ACT_APP_UNREGISTER,
    struct app_unreg_arg {
        esp_gatt_if_t gattc_if;
    } app_unreg;
    //BTC_GATTC_ACT_OPEN,
    struct open_arg {
        esp_gatt_if_t gattc_if;
        esp_bd_addr_t remote_bda;
        esp_ble_addr_type_t remote_addr_type;
        bool is_direct;
        bool is_aux;
        esp_ble_addr_type_t own_addr_type;
        esp_ble_phy_mask_t phy_mask;
        esp_ble_conn_params_t phy_1m_conn_params;
        esp_ble_conn_params_t phy_2m_conn_params;
        esp_ble_conn_params_t phy_coded_conn_params;
    } open;
    //BTC_GATTC_ACT_CLOSE,
    struct close_arg {
        uint16_t conn_id;
    } close;
    //BTC_GATTC_ACT_CFG_MTU,
    struct cfg_mtu_arg {
        uint16_t conn_id;
    } cfg_mtu;
    //BTC_GATTC_ACT_SEARCH_SERVICE,
    struct search_srvc_arg {
        uint16_t conn_id;
        bool filter_uuid_enable;
        esp_bt_uuid_t filter_uuid;
    } search_srvc;
    //BTC_GATTC_ACT_GET_CHAR,
    struct get_char_arg {
        uint16_t conn_id;
        uint16_t handle;
    } get_char;
    //BTC_GATTC_ACT_GET_DESCR,
    struct get_descr_arg {
        uint16_t conn_id;
        uint16_t handle;
    } get_descr;
    //BTC_GATTC_ACT_GET_FIRST_INCL_SERVICE,
    struct get_first_incl_srvc_arg {
        uint16_t conn_id;
        uint16_t handle;
    } get_first_incl_srvc;
    //BTC_GATTC_ACT_GET_NEXT_INCL_SERVICE,
    struct get_next_incl_srvc_arg {
        uint16_t conn_id;
        uint16_t handle;
    } get_next_incl_srvc;
    //BTC_GATTC_ACT_READ_CHAR,
    struct read_char_arg {
        uint16_t conn_id;
        uint16_t handle;
        esp_gatt_auth_req_t auth_req;
    } read_char;
    //BTC_GATTC_ACT_READ_MULTIPLE_CHAR
    struct read_multiple_arg {
        uint16_t conn_id;
        uint8_t  num_attr;
        uint16_t handles[ESP_GATT_MAX_READ_MULTI_HANDLES];
        esp_gatt_auth_req_t auth_req;
    } read_multiple;
    //BTC_GATTC_ACT_READ_CHAR_DESCR,
    struct read_descr_arg {
        uint16_t conn_id;
        uint16_t handle;
        esp_gatt_auth_req_t auth_req;
    } read_descr;
    // BTC_GATTC_ACT_READ_BY_TYPE
    struct read_by_type_arg {
        uint16_t conn_id;
        uint16_t s_handle;
        uint16_t e_handle;
        esp_bt_uuid_t uuid;
        esp_gatt_auth_req_t auth_req;
    } read_by_type;
    //BTC_GATTC_ACT_WRITE_CHAR,
    struct write_char_arg {
        uint16_t conn_id;
        uint16_t value_len;
        uint16_t handle;
        uint8_t *value;
        esp_gatt_write_type_t write_type;
        esp_gatt_auth_req_t auth_req;
    } write_char;
    //BTC_GATTC_ACT_WRITE_CHAR_DESCR,
    struct write_descr_arg {
        uint16_t conn_id;
        uint16_t value_len;
        uint16_t handle;
        uint8_t *value;
        esp_gatt_write_type_t write_type;
        esp_gatt_auth_req_t auth_req;
    } write_descr;
    //BTC_GATTC_ACT_PREPARE_WRITE,
    struct prep_write_arg {
        uint16_t conn_id;
        uint16_t handle;
        uint16_t offset;
        uint16_t value_len;
        uint8_t *value;
        esp_gatt_auth_req_t auth_req;
    } prep_write;
    //BTC_GATTC_ACT_PREPARE_WRITE_CHAR_DESCR,
    struct prep_write_descr_arg {
        uint16_t conn_id;
        uint16_t handle;
        uint16_t offset;
        uint16_t value_len;
        uint8_t *value;
        esp_gatt_auth_req_t auth_req;
    } prep_write_descr;
    //BTC_GATTC_ACT_EXECUTE_WRITE,
    struct exec_write_arg {
        uint16_t conn_id;
        bool is_execute;
    } exec_write;
    //BTC_GATTC_ACT_REG_FOR_NOTIFY,
    struct reg_for_notify_arg {
        esp_gatt_if_t gattc_if;
        esp_bd_addr_t remote_bda;
        uint16_t handle;
    } reg_for_notify;
    //BTC_GATTC_ACT_UNREG_FOR_NOTIFY
    struct unreg_for_notify_arg {
        esp_gatt_if_t gattc_if;
        esp_bd_addr_t remote_bda;
        uint16_t handle;
    } unreg_for_notify;
    //BTC_GATTC_ACT_CACHE_REFRESH,
    struct cache_refresh_arg {
        esp_bd_addr_t remote_bda;
    } cache_refresh;
    //BTC_GATTC_ACT_CACHE_ASSOC
    struct cache_assoc_arg {
        esp_gatt_if_t gattc_if;
        esp_bd_addr_t src_addr;
        esp_bd_addr_t assoc_addr;
        bool is_assoc;
    } cache_assoc;
    //BTC_GATTC_ATC_CACHE_GET_ADDR_LIST
    struct cache_get_addr_list_arg {
        esp_gatt_if_t gattc_if;
    }get_addr_list;
    //BTC_GATTC_ACT_CACHE_CLEAN,
    struct cache_clean_arg {
        esp_bd_addr_t remote_bda;
    } cache_clean;
} btc_ble_gattc_args_t;

void btc_gattc_call_handler(btc_msg_t *msg);
void btc_gattc_cb_handler(btc_msg_t *msg);
void btc_gattc_arg_deep_copy(btc_msg_t *msg, void *p_dest, void *p_src);
void btc_gattc_arg_deep_free(btc_msg_t *msg);
esp_gatt_status_t btc_ble_gattc_get_service(uint16_t conn_id, esp_bt_uuid_t *svc_uuid,
                                            esp_gattc_service_elem_t *result,
                                            uint16_t *count, uint16_t offset);
esp_gatt_status_t btc_ble_gattc_get_all_char(uint16_t conn_id,
                                             uint16_t start_handle,
                                             uint16_t end_handle,
                                             esp_gattc_char_elem_t *result,
                                             uint16_t *count, uint16_t offset);
esp_gatt_status_t btc_ble_gattc_get_all_descr(uint16_t conn_id,
                                              uint16_t char_handle,
                                              esp_gattc_descr_elem_t *result,
                                              uint16_t *count, uint16_t offset);
esp_gatt_status_t btc_ble_gattc_get_char_by_uuid(uint16_t conn_id,
                                                 uint16_t start_handle,
                                                 uint16_t end_handle,
                                                 esp_bt_uuid_t char_uuid,
                                                 esp_gattc_char_elem_t *result,
                                                 uint16_t *count);
esp_gatt_status_t btc_ble_gattc_get_descr_by_uuid(uint16_t conn_id,
                                                  uint16_t start_handle,
                                                  uint16_t end_handle,
                                                  esp_bt_uuid_t char_uuid,
                                                  esp_bt_uuid_t descr_uuid,
                                                  esp_gattc_descr_elem_t *result,
                                                  uint16_t *count);

esp_gatt_status_t btc_ble_gattc_get_descr_by_char_handle(uint16_t conn_id,
                                                         uint16_t char_handle,
                                                         esp_bt_uuid_t descr_uuid,
                                                         esp_gattc_descr_elem_t *result,
                                                         uint16_t *count);

esp_gatt_status_t btc_ble_gattc_get_include_service(uint16_t conn_id,
                                                    uint16_t start_handle,
                                                    uint16_t end_handle,
                                                    esp_bt_uuid_t *incl_uuid,
                                                    esp_gattc_incl_svc_elem_t *result,
                                                    uint16_t *count);

esp_gatt_status_t btc_ble_gattc_get_attr_count(uint16_t conn_id,
                                               esp_gatt_db_attr_type_t type,
                                               uint16_t start_handle,
                                               uint16_t end_handle,
                                               uint16_t char_handle,
                                               uint16_t *count);

esp_gatt_status_t btc_ble_gattc_get_db(uint16_t conn_id, uint16_t start_handle, uint16_t end_handle,
                                       esp_gattc_db_elem_t *db, uint16_t *count);




#endif /* __BTC_GATTC_H__ */
