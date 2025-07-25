/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_bt_defs.h"
#include "esp_gap_bt_api.h"
#include "btc_gap_bt.h"
#include "btc/btc_storage.h"
#include "bta/bta_api.h"
#include "common/bt_trace.h"
#include "common/bt_target.h"
#include "btc/btc_manage.h"
#include "btc/btc_util.h"
#include "osi/allocator.h"
#include "bta/bta_dm_co.h"

#if (BTC_GAP_BT_INCLUDED == TRUE)

#define COD_UNCLASSIFIED ((0x1F) << 8)

#define BTC_STORAGE_FILL_PROPERTY(p_prop, t, l, p_v) \
         (p_prop)->type = t;(p_prop)->len = l; (p_prop)->val = (p_v);

static void bte_search_devices_evt(tBTA_DM_SEARCH_EVT event, tBTA_DM_SEARCH *p_data);
static void bte_dm_search_services_evt(tBTA_DM_SEARCH_EVT event, tBTA_DM_SEARCH *p_data);
static void bte_dm_remote_service_record_evt(tBTA_DM_SEARCH_EVT event, tBTA_DM_SEARCH *p_data);
static void search_services_copy_cb(btc_msg_t *msg, void *p_dest, void *p_src);
static void search_service_record_copy_cb(btc_msg_t *msg, void *p_dest, void *p_src);

static bool btc_gap_bt_inquiry_in_progress = false;

static inline void btc_gap_bt_cb_to_app(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    esp_bt_gap_cb_t cb = (esp_bt_gap_cb_t)btc_profile_cb_get(BTC_PID_GAP_BT);
    if (cb) {
        cb(event, param);
    }
}

static void btc_bt_set_scan_mode(esp_bt_connection_mode_t c_mode, esp_bt_discovery_mode_t d_mode)
{
    tBTA_DM_DISC disc_mode;
    tBTA_DM_CONN conn_mode;

    switch (c_mode) {
    case ESP_BT_NON_CONNECTABLE:
        conn_mode = BTA_DM_NON_CONN;
        break;
    case ESP_BT_CONNECTABLE:
        conn_mode = BTA_DM_CONN;
        break;
    default:
        BTC_TRACE_WARNING("invalid connection mode (0x%x)", c_mode);
        return;
    }

    switch (d_mode) {
    case ESP_BT_NON_DISCOVERABLE:
        disc_mode = BTA_DM_NON_DISC;
        break;
    case ESP_BT_LIMITED_DISCOVERABLE:
        disc_mode = BTA_DM_LIMITED_DISC;
        break;
    case ESP_BT_GENERAL_DISCOVERABLE:
        disc_mode = BTA_DM_GENERAL_DISC;
        break;
    default:
        BTC_TRACE_WARNING("invalid discovery mode (0x%x)", d_mode);
        return;
    }

    BTA_DmSetVisibility(disc_mode, conn_mode, BTA_DM_IGNORE, BTA_DM_IGNORE);
    return;
}

static void btc_gap_bt_start_discovery(btc_gap_bt_args_t *arg)
{
    tBTA_DM_INQ inq_params;
    tBTA_SERVICE_MASK services = 0;

    BTIF_TRACE_EVENT("%s", __FUNCTION__);

    inq_params.mode = (arg->start_disc.mode == ESP_BT_INQ_MODE_GENERAL_INQUIRY) ?
                      BTA_DM_GENERAL_INQUIRY : BTA_DM_LIMITED_INQUIRY;
    inq_params.duration = arg->start_disc.inq_len;
    inq_params.max_resps = arg->start_disc.num_rsps;

    inq_params.report_dup = TRUE;
    inq_params.filter_type = BTA_DM_INQ_CLR;
    /* TODO: Filter device by BDA needs to be implemented here */

    /* Will be enabled to TRUE once inquiry busy level has been received */
    btc_gap_bt_inquiry_in_progress = FALSE;
    /* find nearby devices */
    BTA_DmSearch(&inq_params, services, bte_search_devices_evt);

    return;
}

static void btc_gap_bt_cancel_discovery(void)
{
    BTA_DmSearchCancel();
}

static void btc_gap_bt_get_remote_services(bt_bdaddr_t *remote_bda)
{
    BTA_DmDiscover(remote_bda->address, BTA_ALL_SERVICE_MASK,
                   bte_dm_search_services_evt, TRUE);
}

static void btc_gap_bt_get_remote_service_record(btc_gap_bt_args_t *arg)
{
    esp_bt_uuid_t *uuid = &arg->get_rmt_srv_rcd.uuid;
    bt_bdaddr_t *remote_bda = &arg->get_rmt_srv_rcd.bda;

    tSDP_UUID sdp_uuid;

    sdp_uuid.len = uuid->len;
    memcpy(&sdp_uuid.uu, &uuid->uuid, uuid->len);

    BTA_DmDiscoverUUID(remote_bda->address, &sdp_uuid,
                       bte_dm_remote_service_record_evt, TRUE);
}


/*******************************************************************************
**
** Function         search_devices_copy_cb
**
** Description      Deep copy callback for search devices event
**
** Returns          void
**
*******************************************************************************/
static void search_devices_copy_cb(btc_msg_t *msg, void *p_dest, void *p_src)
{
    tBTA_DM_SEARCH_PARAM *p_dest_data =  (tBTA_DM_SEARCH_PARAM *) p_dest;
    tBTA_DM_SEARCH_PARAM *p_src_data =  (tBTA_DM_SEARCH_PARAM *) p_src;
    if (!p_src) {
        return;
    }
    p_dest_data->p_data = (void *)osi_malloc(p_dest_data->len);
    memset(p_dest_data->p_data, 0x00, p_dest_data->len);
    memcpy(p_dest_data->p_data, p_src_data->p_data, p_dest_data->len);

    if ( p_dest_data->len > sizeof(tBTA_DM_SEARCH)){
        switch (p_dest_data->event) {
        case BTA_DM_INQ_RES_EVT: {
            if (p_src_data->p_data->inq_res.p_eir) {
                p_dest_data->p_data->inq_res.p_eir = (UINT8 *)(p_dest_data->p_data) + sizeof(tBTA_DM_SEARCH);
                memcpy(p_dest_data->p_data->inq_res.p_eir, p_src_data->p_data->inq_res.p_eir, HCI_EXT_INQ_RESPONSE_LEN);
            }
        }
        break;

        case BTA_DM_DISC_RES_EVT: {
            if (p_src_data->p_data->disc_res.raw_data_size && p_src_data->p_data->disc_res.p_raw_data) {
                p_dest_data->p_data->disc_res.p_raw_data = (UINT8 *)(p_dest_data->p_data) + sizeof(tBTA_DM_SEARCH);
                memcpy(p_dest_data->p_data->disc_res.p_raw_data,
                       p_src_data->p_data->disc_res.p_raw_data,
                       p_src_data->p_data->disc_res.raw_data_size);
            }
        }
        break;
        }
    }
}

/*******************************************************************************
**
** Function         search_service_record_copy_cb
**
** Description      Deep copy callback for search service record event
**
** Returns          void
**
*******************************************************************************/
static void search_service_record_copy_cb(btc_msg_t *msg, void *p_dest, void *p_src)
{
    tBTA_DM_SEARCH_PARAM *p_dest_data =  (tBTA_DM_SEARCH_PARAM *) p_dest;
    tBTA_DM_SEARCH_PARAM *p_src_data =  (tBTA_DM_SEARCH_PARAM *) p_src;

    if (!p_src) {
        return;
    }
    p_dest_data->p_data = osi_malloc(p_dest_data->len);
    memset(p_dest_data->p_data, 0x00, p_dest_data->len);
    memcpy(p_dest_data->p_data, p_src_data->p_data, p_dest_data->len);
    if ( p_dest_data->len > sizeof(tBTA_DM_SEARCH)){
        switch (p_dest_data->event) {
        case BTA_DM_DISC_RES_EVT: {
            if (p_src_data->p_data->disc_res.p_raw_data && p_src_data->p_data->disc_res.raw_data_size > 0) {
                p_dest_data->p_data->disc_res.p_raw_data = (UINT8 *)(p_dest_data->p_data) + sizeof(tBTA_DM_SEARCH);
                memcpy(p_dest_data->p_data->disc_res.p_raw_data,
                       p_src_data->p_data->disc_res.p_raw_data,
                       p_src_data->p_data->disc_res.raw_data_size);
            }
        }
        break;

        default:
            break;
        }
    }
}

/*******************************************************************************
**
** Function         check_eir_remote_name
**
** Description      Check if remote name is in the EIR data
**
** Returns          TRUE if remote name found
**                  Populate p_remote_name, if provided and remote name found
**
*******************************************************************************/
static BOOLEAN check_eir_remote_name(tBTA_DM_SEARCH *p_search_data,
                                     UINT8 *p_remote_name, UINT8 *p_remote_name_len)
{
    UINT8 *p_eir_remote_name = NULL;
    UINT8 remote_name_len = 0;

    /* Check EIR for remote name and services */
    if (p_search_data->inq_res.p_eir) {
        p_eir_remote_name = BTM_CheckEirData(p_search_data->inq_res.p_eir,
                                             BTM_EIR_COMPLETE_LOCAL_NAME_TYPE, &remote_name_len);
        if (!p_eir_remote_name) {
            p_eir_remote_name = BTM_CheckEirData(p_search_data->inq_res.p_eir,
                                                 BTM_EIR_SHORTENED_LOCAL_NAME_TYPE, &remote_name_len);
        }

        if (p_eir_remote_name) {
            if (remote_name_len > BD_NAME_LEN) {
                remote_name_len = BD_NAME_LEN;
            }

            if (p_remote_name && p_remote_name_len) {
                memcpy(p_remote_name, p_eir_remote_name, remote_name_len);
                *(p_remote_name + remote_name_len) = 0;
                *p_remote_name_len = remote_name_len;
            }

            return TRUE;
        }
    }

    return FALSE;

}

/*******************************************************************************
**
** Function         bte_search_devices_evt
**
** Description      Switches context from BTE to BTIF for DM search events
**
** Returns          void
**
*******************************************************************************/
static void bte_search_devices_evt(tBTA_DM_SEARCH_EVT event, tBTA_DM_SEARCH *p_data)
{
    tBTA_DM_SEARCH_PARAM search;
    search.event = event;
    search.p_data = p_data;

    UINT16 param_len = 0;

    if (p_data) {
        param_len += sizeof(tBTA_DM_SEARCH);
    }
    /* Allocate buffer to hold the pointers (deep copy). The pointers will point to the end of the tBTA_DM_SEARCH */
    switch (event) {
    case BTA_DM_INQ_RES_EVT: {
        if (p_data->inq_res.p_eir) {
            param_len += HCI_EXT_INQ_RESPONSE_LEN;
        }
    }
    break;

    case BTA_DM_DISC_RES_EVT: {
        if (p_data->disc_res.raw_data_size && p_data->disc_res.p_raw_data) {
            param_len += p_data->disc_res.raw_data_size;
        }
    }
    break;
    }

    /* if remote name is available in EIR, set the flag so that stack doesn't trigger RNR */
    if (event == BTA_DM_INQ_RES_EVT) {
        p_data->inq_res.remt_name_not_required = check_eir_remote_name(p_data, NULL, NULL);
    }

    search.len = param_len;
    do {
        btc_msg_t msg;
        msg.sig = BTC_SIG_API_CB;
        msg.pid = BTC_PID_GAP_BT;
        msg.act = BTC_GAP_BT_SEARCH_DEVICES_EVT;

        btc_transfer_context(&msg, &search, sizeof(tBTA_DM_SEARCH_PARAM), search_devices_copy_cb, NULL);
    } while (0);
}

static void btc_gap_bt_search_devices_evt(tBTA_DM_SEARCH_PARAM *p_data)
{
    switch (p_data->event) {
    case BTA_DM_DISC_RES_EVT: {
        /* remote name update */
        uint32_t bdname_len = strlen((const char *)p_data->p_data->disc_res.bd_name);
        if (bdname_len) {
            esp_bt_gap_dev_prop_t prop[1];

            BTC_STORAGE_FILL_PROPERTY(&prop[0], ESP_BT_GAP_DEV_PROP_BDNAME, bdname_len + 1, p_data->p_data->disc_res.bd_name);

            esp_bt_gap_cb_param_t param;
            bdcpy(param.disc_res.bda, p_data->p_data->disc_res.bd_addr);
            param.disc_res.num_prop = 1;
            param.disc_res.prop = prop;
            btc_gap_bt_cb_to_app(ESP_BT_GAP_DISC_RES_EVT, &param);
        }
        break;
    }
    case BTA_DM_INQ_RES_EVT: {
        /* inquiry result */
        uint32_t cod = devclass2uint (p_data->p_data->inq_res.dev_class);

        if (cod == 0) {
            BTC_TRACE_DEBUG("%s cod is 0, set as unclassified", __func__);
            cod = COD_UNCLASSIFIED;
        }

        do {
            esp_bt_gap_dev_prop_t prop[3];
            int num_prop = 0;

            memset(prop, 0, sizeof(prop));
            BTC_STORAGE_FILL_PROPERTY(&prop[0], ESP_BT_GAP_DEV_PROP_COD, sizeof(cod), &cod);
            num_prop++;

            BTC_STORAGE_FILL_PROPERTY(&prop[1], ESP_BT_GAP_DEV_PROP_RSSI, 1, &(p_data->p_data->inq_res.rssi));
            num_prop++;

            if (p_data->p_data->inq_res.p_eir) {
                BTC_STORAGE_FILL_PROPERTY(&prop[2], ESP_BT_GAP_DEV_PROP_EIR, HCI_EXT_INQ_RESPONSE_LEN, p_data->p_data->inq_res.p_eir);
                num_prop++;
            }

            /* Callback to notify upper layer of device */
            esp_bt_gap_cb_param_t param;
            bdcpy(param.disc_res.bda, p_data->p_data->inq_res.bd_addr);
            param.disc_res.num_prop = num_prop;
            param.disc_res.prop = prop;
            btc_gap_bt_cb_to_app(ESP_BT_GAP_DISC_RES_EVT, &param);
        } while (0);
    }
    break;

    case BTA_DM_INQ_CMPL_EVT:
        break;
    case BTA_DM_DISC_CMPL_EVT: {
        esp_bt_gap_cb_param_t param;
        param.disc_st_chg.state = ESP_BT_GAP_DISCOVERY_STOPPED;
        btc_gap_bt_cb_to_app(ESP_BT_GAP_DISC_STATE_CHANGED_EVT, &param);
        break;
    }
    case BTA_DM_SEARCH_CANCEL_CMPL_EVT: {
        /* if inquiry is not in progress and we get a cancel event, then
         * it means we are done with inquiry, but remote_name fetches are in
         * progress
         *
         * if inquiry is in progress, then we don't want to act on this cancel_cmpl_evt
         * but instead wait for the cancel_cmpl_evt_via the busy level
         */
        if (btc_gap_bt_inquiry_in_progress == false) {
            esp_bt_gap_cb_param_t param;
            param.disc_st_chg.state = ESP_BT_GAP_DISCOVERY_STOPPED;
            btc_gap_bt_cb_to_app(ESP_BT_GAP_DISC_STATE_CHANGED_EVT, &param);
        }
        break;
    }
    }
}
/*******************************************************************************
**
** Function         btc_gap_bt_search_service_record
**
** Description      Executes search service record event in btif context
**
** Returns          void
**
*******************************************************************************/
static void btc_gap_bt_search_service_record(char *p_param)
{
    tBTA_DM_SEARCH_PARAM *p_data = (tBTA_DM_SEARCH_PARAM *)p_param;

    switch (p_data->event) {
    case BTA_DM_DISC_RES_EVT: {
        esp_bt_gap_cb_param_t param;
        memcpy(param.rmt_srvcs.bda, p_data->p_data->disc_res.bd_addr, BD_ADDR_LEN);
        if (p_data->p_data->disc_res.p_raw_data && p_data->p_data->disc_res.raw_data_size > 0) {
            param.rmt_srvc_rec.stat = ESP_BT_STATUS_SUCCESS;
            // param.rmt_srvc_rec.raw_data_size = p_data->p_data->disc_res.raw_data_size;
            // param.rmt_srvc_rec.raw_data = p_data->p_data->disc_res.p_raw_data;
        } else {
            param.rmt_srvc_rec.stat = ESP_BT_STATUS_FAIL;
            // param.rmt_srvc_rec.raw_data_size = 0;
            // param.rmt_srvc_rec.raw_data = NULL;
        }
        btc_gap_bt_cb_to_app(ESP_BT_GAP_RMT_SRVC_REC_EVT, &param);
    }
    break;
    case BTA_DM_DISC_CMPL_EVT:
    default:
        break;
    }
}


/*******************************************************************************
**
** Function         bte_dm_remote_service_record_evt
**
** Description      Switches context from BTE to BTC for DM search service
**                  record event
**
** Returns          void
**
*******************************************************************************/
static void bte_dm_remote_service_record_evt(tBTA_DM_SEARCH_EVT event, tBTA_DM_SEARCH *p_data)
{
    tBTA_DM_SEARCH_PARAM search;
    search.event = event;
    search.p_data = p_data;
    UINT16 param_len = 0;

    if (p_data) {
        param_len += sizeof(tBTA_DM_SEARCH);
    }
    /* Allocate buffer to hold the pointers (deep copy). The pointers will point to the end of the tBTA_DM_SEARCH */
    if (event == BTA_DM_DISC_RES_EVT) {
        if (p_data->disc_res.raw_data_size && p_data->disc_res.p_raw_data) {
            param_len += p_data->disc_res.raw_data_size;
        }
    }
    search.len = param_len;
    do {
        btc_msg_t msg;
        msg.sig = BTC_SIG_API_CB;
        msg.pid = BTC_PID_GAP_BT;
        msg.act = BTC_GAP_BT_SEARCH_SERVICE_RECORD_EVT;
        btc_transfer_context(&msg, &search, sizeof(tBTA_DM_SEARCH_PARAM), search_service_record_copy_cb, NULL);
    } while (0);

}

/*******************************************************************************
**
** Function         btc_gap_bt_search_services
**
** Description      Executes search services event in btc context
**
** Returns          void
**
*******************************************************************************/
static void btc_gap_bt_search_services(char *p_param)
{
    tBTA_DM_SEARCH_PARAM *p_data = (tBTA_DM_SEARCH_PARAM *)p_param;

    switch (p_data->event) {
    case BTA_DM_DISC_RES_EVT: {
        esp_bt_gap_cb_param_t param;
        esp_bt_uuid_t *uuid_list = NULL;
        memcpy(param.rmt_srvcs.bda, p_data->p_data->disc_res.bd_addr, BD_ADDR_LEN);

        param.rmt_srvcs.stat = ESP_BT_STATUS_FAIL;
        if (p_data->p_data->disc_res.result == BTA_SUCCESS) {
            uuid_list = osi_malloc(sizeof(esp_bt_uuid_t) * p_data->p_data->disc_res.num_uuids);
            if (uuid_list) {
                param.rmt_srvcs.stat = ESP_BT_STATUS_SUCCESS;
                param.rmt_srvcs.num_uuids = p_data->p_data->disc_res.num_uuids;
                param.rmt_srvcs.uuid_list = uuid_list;
                // copy UUID list
                uint8_t *i_uu = (uint8_t *)p_data->p_data->disc_res.p_uuid_list;
                esp_bt_uuid_t *o_uu = uuid_list;
                for (int i = 0; i < p_data->p_data->disc_res.num_uuids; i++, i_uu += ESP_UUID_LEN_128, o_uu++) {
                    uuid128_be_to_esp_uuid(o_uu, i_uu);
                }
            }
        }

        if (param.rmt_srvcs.stat == ESP_BT_STATUS_FAIL) {
            param.rmt_srvcs.num_uuids = 0;
            param.rmt_srvcs.uuid_list = NULL;
        }
        btc_gap_bt_cb_to_app(ESP_BT_GAP_RMT_SRVCS_EVT, &param);

        if (uuid_list) {
            osi_free(uuid_list);
        }
    }
    break;

    case BTA_DM_DISC_BLE_RES_EVT:
    case BTA_DM_DISC_CMPL_EVT:
    default:
        break;
    }
}

/*******************************************************************************
**
** Function         bte_dm_search_services_evt
**
** Description      Switches context from BTE to BTIF for DM search services
**                  event
**
** Returns          void
**
*******************************************************************************/
static void bte_dm_search_services_evt(tBTA_DM_SEARCH_EVT event, tBTA_DM_SEARCH *p_data)
{
    tBTA_DM_SEARCH_PARAM search;
    search.event = event;
    search.p_data = p_data;

    UINT16 param_len = 0;
    if (p_data) {
        param_len += sizeof(tBTA_DM_SEARCH);
    }

    switch (event) {
    case BTA_DM_DISC_RES_EVT: {
        if ((p_data->disc_res.result == BTA_SUCCESS) && (p_data->disc_res.num_uuids > 0)) {
            param_len += (p_data->disc_res.num_uuids * MAX_UUID_SIZE);
        }
    } break;
    }
    search.len = param_len;
    do {
        btc_msg_t msg;
        msg.sig = BTC_SIG_API_CB;
        msg.pid = BTC_PID_GAP_BT;
        msg.act = BTC_GAP_BT_SEARCH_SERVICES_EVT;
        btc_transfer_context(&msg, &search, sizeof(tBTA_DM_SEARCH_PARAM), search_services_copy_cb, NULL);
    } while (0);
}

static void search_services_copy_cb(btc_msg_t *msg, void *p_dest, void *p_src)
{
    tBTA_DM_SEARCH_PARAM *p_dest_data =  (tBTA_DM_SEARCH_PARAM *) p_dest;
    tBTA_DM_SEARCH_PARAM *p_src_data =  (tBTA_DM_SEARCH_PARAM *) p_src;

    if (!p_src) {
        return;
    }
    p_dest_data->p_data = osi_malloc(p_dest_data->len);
    memset(p_dest_data->p_data, 0x00, p_dest_data->len);
    memcpy(p_dest_data->p_data, p_src_data->p_data, p_dest_data->len);

    if ( p_dest_data->len > sizeof(tBTA_DM_SEARCH)){
        switch (p_dest_data->event) {
        case BTA_DM_DISC_RES_EVT: {
            if (p_src_data->p_data->disc_res.result == BTA_SUCCESS) {
                if (p_src_data->p_data->disc_res.num_uuids > 0) {
                    p_dest_data->p_data->disc_res.p_uuid_list = (UINT8 *)(p_dest_data->p_data) + sizeof(tBTA_DM_SEARCH);
                    memcpy(p_dest_data->p_data->disc_res.p_uuid_list, p_src_data->p_data->disc_res.p_uuid_list,
                           p_src_data->p_data->disc_res.num_uuids * MAX_UUID_SIZE);
                    osi_free(p_src_data->p_data->disc_res.p_uuid_list);
                    p_src_data->p_data->disc_res.p_uuid_list = NULL;
                }
                if (p_src_data->p_data->disc_res.p_raw_data != NULL) {
                    osi_free(p_src_data->p_data->disc_res.p_raw_data);
                    p_src_data->p_data->disc_res.p_raw_data = NULL;
                }
            }
        } break;
        }
    }
}

static void btc_gap_bt_set_cod(btc_gap_bt_args_t *arg)
{
    tBTA_UTL_COD p_cod;
    esp_bt_cod_t *cod = &(arg->set_cod.cod);
    p_cod.reserved_2 = cod->reserved_2;
    p_cod.minor = cod->minor << 2;
    p_cod.major = cod->major;
    p_cod.service = cod->service << 5;
    bool ret = utl_set_device_class(&p_cod, arg->set_cod.mode);
    if (!ret){
        BTC_TRACE_ERROR("%s set class of device failed!",__func__);
    }
}

esp_err_t btc_gap_bt_get_cod(esp_bt_cod_t *cod)
{
    tBTA_UTL_COD p_cod;
    bool ret = utl_get_device_class(&p_cod);
    if (!ret){
        BTC_TRACE_ERROR("%s get class of device failed!",__func__);
        return ESP_BT_STATUS_FAIL;
    }
    cod->reserved_2 = p_cod.reserved_2;
    cod->minor = p_cod.minor >> 2;
    cod->major = p_cod.major;
    cod->service = p_cod.service >> 5;
    return ESP_BT_STATUS_SUCCESS;
}

static void btc_gap_bt_read_rssi_delta_cmpl_callback(void *p_data)
{
    tBTA_RSSI_RESULTS *result = (tBTA_RSSI_RESULTS *)p_data;
    esp_bt_gap_cb_param_t param;
    bt_status_t ret;
    btc_msg_t msg;
    msg.sig = BTC_SIG_API_CB;
    msg.pid = BTC_PID_GAP_BT;
    msg.act = BTC_GAP_BT_READ_RSSI_DELTA_EVT;
    memcpy(param.read_rssi_delta.bda, result->rem_bda, sizeof(BD_ADDR));
    param.read_rssi_delta.stat = btc_btm_status_to_esp_status(result->status);
    param.read_rssi_delta.rssi_delta = result->rssi;

    ret = btc_transfer_context(&msg, &param,
                               sizeof(esp_bt_gap_cb_param_t), NULL, NULL);

    if (ret != BT_STATUS_SUCCESS) {
        BTC_TRACE_ERROR("%s btc_transfer_context failed\n", __func__);
    }
}

static void btc_gap_bt_read_rssi_delta(btc_gap_bt_args_t *arg)
{
    BTA_DmReadRSSI(arg->read_rssi_delta.bda.address, BTA_TRANSPORT_BR_EDR, btc_gap_bt_read_rssi_delta_cmpl_callback);
}

static esp_err_t btc_gap_bt_remove_bond_device(btc_gap_bt_args_t *arg)
{
    BD_ADDR bd_addr;
    memcpy(bd_addr, arg->rm_bond_device.bda.address, sizeof(BD_ADDR));
    if(BTA_DmRemoveDevice(bd_addr, BT_TRANSPORT_BR_EDR) == BTA_SUCCESS){
        return ESP_BT_STATUS_SUCCESS;
    }
    return ESP_BT_STATUS_FAIL;
}

static void btc_gap_bt_set_pin_type(btc_gap_bt_args_t *arg){
    BTA_DMSetPinType (arg->set_pin_type.pin_type, arg->set_pin_type.pin_code, arg->set_pin_type.pin_code_len);
}

static void btc_gap_bt_pin_reply(btc_gap_bt_args_t *arg){
    BTA_DmPinReply(arg->pin_reply.bda.address, arg->pin_reply.accept, arg->pin_reply.pin_code_len, arg->pin_reply.pin_code);
}

static esp_err_t btc_gap_bt_set_security_param(btc_gap_bt_args_t *arg)
{
    esp_err_t ret;
    switch(arg->set_security_param.param_type) {
    case ESP_BT_SP_IOCAP_MODE:{
        uint8_t iocap = 0;
        uint8_t *p = arg->set_security_param.value;
        STREAM_TO_UINT8(iocap, p);
        ret = bta_dm_co_bt_set_io_cap(iocap);
        break;
    }
    default:
        ret = ESP_BT_STATUS_FAIL;
        break;
    }
    return ret;
}

static void btc_gap_bt_ssp_passkey_reply(btc_gap_bt_args_t *arg)
{
    BTA_DmPasskeyReqReply(arg->passkey_reply.accept, arg->passkey_reply.bda.address, arg->passkey_reply.passkey);
}

static void btc_gap_bt_ssp_confirm(btc_gap_bt_args_t *arg)
{
    BTA_DmConfirm(arg->confirm_reply.bda.address, arg->confirm_reply.accept);
}

static void btc_gap_bt_config_eir(btc_gap_bt_args_t *arg)
{
    tBTA_DM_EIR_CONF eir_config;
    esp_bt_eir_data_t *eir_data = &arg->config_eir.eir_data;

    eir_config.bta_dm_eir_fec_required = eir_data->fec_required;
    eir_config.bta_dm_eir_included_name = eir_data->include_name;
    eir_config.bta_dm_eir_included_tx_power = eir_data->include_txpower;
    eir_config.bta_dm_eir_included_uuid = eir_data->include_uuid;
    eir_config.bta_dm_eir_flags = eir_data->flag;
    eir_config.bta_dm_eir_manufac_spec_len = eir_data->manufacturer_len;
    eir_config.bta_dm_eir_manufac_spec = eir_data->p_manufacturer_data;
    eir_config.bta_dm_eir_url_len = eir_data->url_len;
    eir_config.bta_dm_eir_url = eir_data->p_url;

    BTA_DmConfigEir(&eir_config);
}

static void btc_gap_bt_set_afh_channels_cmpl_callback(void *p_data)
{
    tBTA_SET_AFH_CHANNELS_RESULTS *result = (tBTA_SET_AFH_CHANNELS_RESULTS *)p_data;
    esp_bt_gap_cb_param_t param;
    bt_status_t ret;
    btc_msg_t msg;
    msg.sig = BTC_SIG_API_CB;
    msg.pid = BTC_PID_GAP_BT;
    msg.act = BTC_GAP_BT_SET_AFH_CHANNELS_EVT;

    param.set_afh_channels.stat = btc_btm_status_to_esp_status(result->status);

    ret = btc_transfer_context(&msg, &param,
                               sizeof(esp_bt_gap_cb_param_t), NULL, NULL);

    if (ret != BT_STATUS_SUCCESS) {
        BTC_TRACE_ERROR("%s btc_transfer_context failed\n", __func__);
    }
}

static void btc_gap_bt_set_afh_channels(btc_gap_bt_args_t *arg)
{
    BTA_DmSetAfhChannels(arg->set_afh_channels.channels, btc_gap_bt_set_afh_channels_cmpl_callback);
}

static void btc_gap_bt_set_page_timeout_cmpl_callback(void *p_data)
{
    tBTA_SET_PAGE_TIMEOUT_RESULTS *result = (tBTA_SET_PAGE_TIMEOUT_RESULTS *)p_data;
    esp_bt_gap_cb_param_t param;
    bt_status_t ret;
    btc_msg_t msg;
    msg.sig = BTC_SIG_API_CB;
    msg.pid = BTC_PID_GAP_BT;
    msg.act = BTC_GAP_BT_SET_PAGE_TO_EVT;

    param.set_page_timeout.stat = btc_btm_status_to_esp_status(result->status);

    ret = btc_transfer_context(&msg, &param, sizeof(esp_bt_gap_cb_param_t), NULL, NULL);
    if (ret != BT_STATUS_SUCCESS) {
        BTC_TRACE_ERROR("%s btc_transfer_context failed\n", __func__);
    }
}

static void btc_gap_set_page_timeout(btc_gap_bt_args_t *arg)
{
    BTA_DmSetPageTimeout(arg->set_page_to.page_to, btc_gap_bt_set_page_timeout_cmpl_callback);
}

static void btc_gap_bt_get_page_timeout_cmpl_callback(void *p_data)
{
    tBTA_GET_PAGE_TIMEOUT_RESULTS *result = (tBTA_GET_PAGE_TIMEOUT_RESULTS *)p_data;
    esp_bt_gap_cb_param_t param;
    bt_status_t ret;
    btc_msg_t msg;
    msg.sig = BTC_SIG_API_CB;
    msg.pid = BTC_PID_GAP_BT;
    msg.act = BTC_GAP_BT_GET_PAGE_TO_EVT;

    param.get_page_timeout.stat = btc_btm_status_to_esp_status(result->status);
    param.get_page_timeout.page_to = result->page_to;

    ret = btc_transfer_context(&msg, &param, sizeof(esp_bt_gap_cb_param_t), NULL, NULL);
    if (ret != BT_STATUS_SUCCESS) {
        BTC_TRACE_ERROR("%s btc_transfer_context failed\n", __func__);
    }
}

static void btc_gap_get_page_timeout(void)
{
    BTA_DmGetPageTimeout(btc_gap_bt_get_page_timeout_cmpl_callback);
}

static void btc_gap_bt_set_acl_pkt_types_cmpl_callback(void *p_data)
{
    tBTA_SET_ACL_PKT_TYPES_RESULTS *result = (tBTA_SET_ACL_PKT_TYPES_RESULTS *)p_data;
    esp_bt_gap_cb_param_t param;
    bt_status_t ret;
    btc_msg_t msg;
    msg.sig = BTC_SIG_API_CB;
    msg.pid = BTC_PID_GAP_BT;
    msg.act = BTC_GAP_BT_SET_ACL_PKT_TYPES_EVT;

    param.set_acl_pkt_types.status = btc_btm_status_to_esp_status(result->status);
    memcpy(param.set_acl_pkt_types.bda, result->rem_bda, sizeof(esp_bd_addr_t));
    param.set_acl_pkt_types.pkt_types = result->pkt_types;

    ret = btc_transfer_context(&msg, &param, sizeof(esp_bt_gap_cb_param_t), NULL, NULL);
    if (ret != BT_STATUS_SUCCESS) {
        BTC_TRACE_ERROR("%s btc_transfer_context failed\n", __func__);
    }
}

static void btc_gap_set_acl_pkt_types(btc_gap_bt_args_t *arg)
{
    BTA_DmSetAclPktTypes(arg->set_acl_pkt_types.bda.address,
                         arg->set_acl_pkt_types.pkt_types,
                         btc_gap_bt_set_acl_pkt_types_cmpl_callback);
}

#if (ENC_KEY_SIZE_CTRL_MODE != ENC_KEY_SIZE_CTRL_MODE_NONE)
static void btc_gap_bt_set_min_enc_key_size_cmpl_callback(void *p_data)
{
    tBTA_SET_MIN_ENC_KEY_SIZE_RESULTS *result = (tBTA_SET_MIN_ENC_KEY_SIZE_RESULTS *)p_data;
    esp_bt_gap_cb_param_t param;
    bt_status_t ret;
    btc_msg_t msg;
    msg.sig = BTC_SIG_API_CB;
    msg.pid = BTC_PID_GAP_BT;
    msg.act = BTC_GAP_BT_SET_MIN_ENC_KEY_SIZE_EVT;

    param.set_min_enc_key_size.status = btc_hci_to_esp_status(result->hci_status);

    ret = btc_transfer_context(&msg, &param, sizeof(esp_bt_gap_cb_param_t), NULL, NULL);
    if (ret != BT_STATUS_SUCCESS) {
        BTC_TRACE_ERROR("%s btc_transfer_context failed\n", __func__);
    }
}

static void btc_gap_set_min_enc_key_size(btc_gap_bt_args_t *arg)
{
    BTA_DmSetMinEncKeySize(arg->set_min_enc_key_size.key_size, btc_gap_bt_set_min_enc_key_size_cmpl_callback);
}
#endif

static void btc_gap_bt_read_remote_name_cmpl_callback(void *p_data)
{
    tBTA_REMOTE_DEV_NAME *result = (tBTA_REMOTE_DEV_NAME *)p_data;
    esp_bt_gap_cb_param_t param;
    btc_msg_t msg;
    bt_status_t ret;
    msg.sig = BTC_SIG_API_CB;
    msg.pid = BTC_PID_GAP_BT;
    msg.act = BTC_GAP_BT_READ_REMOTE_NAME_EVT;

    memcpy(param.read_rmt_name.bda,result->bd_addr,BD_ADDR_LEN);
    param.read_rmt_name.stat = btc_btm_status_to_esp_status(result->status);
    memcpy(param.read_rmt_name.rmt_name,result->remote_bd_name,ESP_BT_GAP_MAX_BDNAME_LEN);

    ret = btc_transfer_context(&msg, &param, sizeof(esp_bt_gap_cb_param_t), NULL, NULL);
    if (ret != BT_STATUS_SUCCESS) {
        BTC_TRACE_ERROR("%s btc_transfer_context failed\n", __func__);
    }
}

static void btc_gap_bt_read_remote_name(btc_gap_bt_args_t *arg)
{
    BTA_DmGetRemoteName(arg->rmt_name_bda.address, btc_gap_bt_read_remote_name_cmpl_callback);
}

#if (BTA_DM_QOS_INCLUDED == TRUE)
static void btc_gap_bt_set_qos_cmpl_callback(void *p_data)
{
    tBTM_QOS_SETUP_CMPL *result = (tBTM_QOS_SETUP_CMPL *)p_data;
    esp_bt_gap_cb_param_t param;
    btc_msg_t msg;
    bt_status_t ret;
    msg.sig = BTC_SIG_API_CB;
    msg.pid = BTC_PID_GAP_BT;
    msg.act = BTC_GAP_BT_QOS_EVT;

    param.qos_cmpl.stat = btc_btm_status_to_esp_status(result->status);
    param.qos_cmpl.t_poll = result->flow.latency / 625;
    memcpy(param.qos_cmpl.bda,result->rem_bda,BD_ADDR_LEN);

    ret = btc_transfer_context(&msg, &param, sizeof(esp_bt_gap_cb_param_t), NULL, NULL);
    if (ret != BT_STATUS_SUCCESS) {
        BTC_TRACE_ERROR("%s btc_transfer_context failed\n", __func__);
    }
}
#endif /// (BTA_DM_QOS_INCLUDED == TRUE)

static void btc_gap_bt_set_qos(btc_gap_bt_args_t *arg)
{
#if (BTA_DM_QOS_INCLUDED == TRUE)
    BTA_DmSetQos(arg->set_qos.bda.address, arg->set_qos.t_poll, btc_gap_bt_set_qos_cmpl_callback);
#else
    BTC_TRACE_ERROR("%s: QoS is not supported.\n",__func__);
#endif /// (BTA_DM_QOS_INCLUDED == TRUE)
}

static void btc_gap_bt_get_dev_name_callback(UINT8 status, char *name)
{
    esp_bt_gap_cb_param_t param;
    bt_status_t ret;
    btc_msg_t msg = {0};

    memset(&param, 0, sizeof(esp_bt_gap_cb_param_t));

    msg.sig = BTC_SIG_API_CB;
    msg.pid = BTC_PID_GAP_BT;
    msg.act = BTC_GAP_BT_GET_DEV_NAME_CMPL_EVT;

    param.get_dev_name_cmpl.status = btc_btm_status_to_esp_status(status);
    param.get_dev_name_cmpl.name = (char *)osi_malloc(BTC_MAX_LOC_BD_NAME_LEN + 1);
    if (param.get_dev_name_cmpl.name) {
        BCM_STRNCPY_S(param.get_dev_name_cmpl.name, name, BTC_MAX_LOC_BD_NAME_LEN);
        param.get_dev_name_cmpl.name[BTC_MAX_LOC_BD_NAME_LEN] = '\0';
    } else {
        param.get_dev_name_cmpl.status = ESP_BT_STATUS_NOMEM;
    }

    ret = btc_transfer_context(&msg, &param, sizeof(esp_bt_gap_cb_param_t), NULL, NULL);
    if (ret != BT_STATUS_SUCCESS) {
        BTC_TRACE_ERROR("%s btc_transfer_context failed\n", __func__);
    }
}

void btc_gap_bt_arg_deep_copy(btc_msg_t *msg, void *p_dest, void *p_src)
{
    switch (msg->act) {
    case BTC_GAP_BT_ACT_SET_SCAN_MODE:
    case BTC_GAP_BT_ACT_START_DISCOVERY:
    case BTC_GAP_BT_ACT_CANCEL_DISCOVERY:
    case BTC_GAP_BT_ACT_GET_REMOTE_SERVICES:
    case BTC_GAP_BT_ACT_GET_REMOTE_SERVICE_RECORD:
    case BTC_GAP_BT_ACT_SET_COD:
    case BTC_GAP_BT_ACT_READ_RSSI_DELTA:
    case BTC_GAP_BT_ACT_REMOVE_BOND_DEVICE:
    case BTC_GAP_BT_ACT_PIN_REPLY:
    case BTC_GAP_BT_ACT_SET_PIN_TYPE:
    case BTC_GAP_BT_ACT_SET_AFH_CHANNELS:
    case BTC_GAP_BT_ACT_READ_REMOTE_NAME:
    case BTC_GAP_BT_ACT_SET_QOS:
    case BTC_GAP_BT_ACT_SET_PAGE_TIMEOUT:
    case BTC_GAP_BT_ACT_GET_PAGE_TIMEOUT:
    case BTC_GAP_BT_ACT_SET_ACL_PKT_TYPES:
    case BTC_GAP_BT_ACT_GET_DEV_NAME:
#if (ENC_KEY_SIZE_CTRL_MODE != ENC_KEY_SIZE_CTRL_MODE_NONE)
    case BTC_GAP_BT_ACT_SET_MIN_ENC_KEY_SIZE:
#endif
        break;
    case BTC_GAP_BT_ACT_PASSKEY_REPLY:
    case BTC_GAP_BT_ACT_CONFIRM_REPLY:
        break;
    case BTC_GAP_BT_ACT_SET_SECURITY_PARAM:{
        btc_gap_bt_args_t *src = (btc_gap_bt_args_t *)p_src;
        btc_gap_bt_args_t *dst = (btc_gap_bt_args_t *)p_dest;
        if (src->set_security_param.value) {
            dst->set_security_param.value = osi_malloc(src->set_security_param.len);
            if (dst->set_security_param.value != NULL) {
                memcpy(dst->set_security_param.value, src->set_security_param.value, src->set_security_param.len);
            } else {
                BTC_TRACE_ERROR("%s %d no mem\n",__func__, msg->act);
            }
        }
        break;
    }

    case BTC_GAP_BT_ACT_CONFIG_EIR:{
        btc_gap_bt_args_t *src = (btc_gap_bt_args_t *)p_src;
        btc_gap_bt_args_t *dst = (btc_gap_bt_args_t *)p_dest;
        if (src->config_eir.eir_data.p_manufacturer_data) {
            dst->config_eir.eir_data.p_manufacturer_data = osi_malloc(src->config_eir.eir_data.manufacturer_len);
            if (dst->config_eir.eir_data.p_manufacturer_data != NULL) {
                memcpy(dst->config_eir.eir_data.p_manufacturer_data, src->config_eir.eir_data.p_manufacturer_data, src->config_eir.eir_data.manufacturer_len);
            } else {
                dst->config_eir.eir_data.manufacturer_len = 0;
                BTC_TRACE_ERROR("%s %d no mem\n",__func__, msg->act);
            }
        }
        if (src->config_eir.eir_data.p_url) {
            dst->config_eir.eir_data.p_url = osi_malloc(src->config_eir.eir_data.url_len);
            if (dst->config_eir.eir_data.p_url != NULL) {
                memcpy(dst->config_eir.eir_data.p_url, src->config_eir.eir_data.p_url, src->config_eir.eir_data.url_len);
            } else {
                dst->config_eir.eir_data.url_len = 0;
                BTC_TRACE_ERROR("%s %d no mem\n",__func__, msg->act);
            }
        }
        break;
    }
    case BTC_GAP_BT_ACT_SET_DEV_NAME: {
        btc_gap_bt_args_t *src = (btc_gap_bt_args_t *)p_src;
        btc_gap_bt_args_t *dst = (btc_gap_bt_args_t *)p_dest;
        dst->bt_set_dev_name.device_name = (char *)osi_malloc((BTC_MAX_LOC_BD_NAME_LEN + 1) * sizeof(char));
        if (dst->bt_set_dev_name.device_name) {
            BCM_STRNCPY_S(dst->bt_set_dev_name.device_name, src->bt_set_dev_name.device_name, BTC_MAX_LOC_BD_NAME_LEN);
            dst->bt_set_dev_name.device_name[BTC_MAX_LOC_BD_NAME_LEN] = '\0';
        } else {
            BTC_TRACE_ERROR("%s %d no mem\n", __func__, msg->act);
        }
        break;
    }
    default:
        BTC_TRACE_ERROR("Unhandled deep copy %d\n", msg->act);
        break;
    }
}

void btc_gap_bt_arg_deep_free(btc_msg_t *msg)
{
    btc_gap_bt_args_t *arg = (btc_gap_bt_args_t *)msg->arg;
    switch (msg->act) {
    case BTC_GAP_BT_ACT_SET_SCAN_MODE:
    case BTC_GAP_BT_ACT_START_DISCOVERY:
    case BTC_GAP_BT_ACT_CANCEL_DISCOVERY:
    case BTC_GAP_BT_ACT_GET_REMOTE_SERVICES:
    case BTC_GAP_BT_ACT_GET_REMOTE_SERVICE_RECORD:
    case BTC_GAP_BT_ACT_SET_COD:
    case BTC_GAP_BT_ACT_READ_RSSI_DELTA:
    case BTC_GAP_BT_ACT_REMOVE_BOND_DEVICE:
    case BTC_GAP_BT_ACT_PIN_REPLY:
    case BTC_GAP_BT_ACT_SET_PIN_TYPE:
    case BTC_GAP_BT_ACT_SET_AFH_CHANNELS:
    case BTC_GAP_BT_ACT_READ_REMOTE_NAME:
    case BTC_GAP_BT_ACT_SET_QOS:
    case BTC_GAP_BT_ACT_SET_PAGE_TIMEOUT:
    case BTC_GAP_BT_ACT_GET_PAGE_TIMEOUT:
    case BTC_GAP_BT_ACT_SET_ACL_PKT_TYPES:
    case BTC_GAP_BT_ACT_GET_DEV_NAME:
#if (ENC_KEY_SIZE_CTRL_MODE != ENC_KEY_SIZE_CTRL_MODE_NONE)
    case BTC_GAP_BT_ACT_SET_MIN_ENC_KEY_SIZE:
#endif
        break;
    case BTC_GAP_BT_ACT_PASSKEY_REPLY:
    case BTC_GAP_BT_ACT_CONFIRM_REPLY:
        break;
    case BTC_GAP_BT_ACT_SET_SECURITY_PARAM:
        if (arg->set_security_param.value) {
            osi_free(arg->set_security_param.value);
        }
        break;

    case BTC_GAP_BT_ACT_CONFIG_EIR:
        if (arg->config_eir.eir_data.p_manufacturer_data) {
            osi_free(arg->config_eir.eir_data.p_manufacturer_data);
        }
        if (arg->config_eir.eir_data.p_url) {
            osi_free(arg->config_eir.eir_data.p_url);
        }
        break;
    case BTC_GAP_BT_ACT_SET_DEV_NAME: {
        char *p_name = arg->bt_set_dev_name.device_name;
        if (p_name) {
            osi_free((uint8_t *)p_name);
        }
        break;
    }
    default:
        BTC_TRACE_ERROR("Unhandled deep copy %d, arg: %p\n", msg->act, arg);
        break;
    }
}

void btc_gap_bt_call_handler(btc_msg_t *msg)
{
    btc_gap_bt_args_t *arg = (btc_gap_bt_args_t *)msg->arg;
    BTC_TRACE_DEBUG("%s act %d\n", __func__, msg->act);
    switch (msg->act) {
    case BTC_GAP_BT_ACT_SET_SCAN_MODE: {
        btc_bt_set_scan_mode(arg->set_scan_mode.c_mode, arg->set_scan_mode.d_mode);
        break;
    }
    case BTC_GAP_BT_ACT_START_DISCOVERY: {
        btc_gap_bt_start_discovery(arg);
        break;
    }
    case BTC_GAP_BT_ACT_CANCEL_DISCOVERY: {
        btc_gap_bt_cancel_discovery();
        break;
    }
    case BTC_GAP_BT_ACT_GET_REMOTE_SERVICES: {
        btc_gap_bt_get_remote_services((bt_bdaddr_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_ACT_GET_REMOTE_SERVICE_RECORD: {
        btc_gap_bt_get_remote_service_record(arg);
        break;
    }
    case BTC_GAP_BT_ACT_SET_COD: {
        btc_gap_bt_set_cod(arg);
        break;
    }
    case BTC_GAP_BT_ACT_READ_RSSI_DELTA: {
        btc_gap_bt_read_rssi_delta(arg);
        break;
    }
    case BTC_GAP_BT_ACT_REMOVE_BOND_DEVICE:{
        btc_gap_bt_remove_bond_device(arg);
        break;
    }
    case BTC_GAP_BT_ACT_SET_PIN_TYPE:{
        btc_gap_bt_set_pin_type(arg);
        break;
    }
    case BTC_GAP_BT_ACT_PIN_REPLY: {
        btc_gap_bt_pin_reply(arg);
        break;
    }
    case BTC_GAP_BT_ACT_SET_SECURITY_PARAM:{
        btc_gap_bt_set_security_param(arg);
        break;
    }
    case BTC_GAP_BT_ACT_PASSKEY_REPLY:{
        btc_gap_bt_ssp_passkey_reply(arg);
        break;
    }
    case BTC_GAP_BT_ACT_CONFIRM_REPLY:{
        btc_gap_bt_ssp_confirm(arg);
        break;
    }
    case BTC_GAP_BT_ACT_CONFIG_EIR: {
        btc_gap_bt_config_eir(arg);
        break;
    }

    case BTC_GAP_BT_ACT_SET_AFH_CHANNELS: {
        btc_gap_bt_set_afh_channels(arg);
        break;
    }
    case BTC_GAP_BT_ACT_READ_REMOTE_NAME: {
        btc_gap_bt_read_remote_name(arg);
        break;
    }
    case BTC_GAP_BT_ACT_SET_QOS: {
        btc_gap_bt_set_qos(arg);
        break;
    }
    case BTC_GAP_BT_ACT_SET_PAGE_TIMEOUT: {
        btc_gap_set_page_timeout(arg);
        break;
    }
    case BTC_GAP_BT_ACT_GET_PAGE_TIMEOUT: {
        btc_gap_get_page_timeout();
        break;
    }
    case BTC_GAP_BT_ACT_SET_ACL_PKT_TYPES: {
        btc_gap_set_acl_pkt_types(arg);
        break;
    }
#if (ENC_KEY_SIZE_CTRL_MODE != ENC_KEY_SIZE_CTRL_MODE_NONE)
    case BTC_GAP_BT_ACT_SET_MIN_ENC_KEY_SIZE: {
        btc_gap_set_min_enc_key_size(arg);
        break;
    }
#endif
    case BTC_GAP_BT_ACT_SET_DEV_NAME: {
        BTA_DmSetDeviceName(arg->bt_set_dev_name.device_name, BT_DEVICE_TYPE_BREDR);
        break;
    }
    case BTC_GAP_BT_ACT_GET_DEV_NAME: {
        BTA_DmGetDeviceName(btc_gap_bt_get_dev_name_callback, BT_DEVICE_TYPE_BREDR);
        break;
    }
    default:
        break;
    }
    btc_gap_bt_arg_deep_free(msg);
    return;
}

void btc_gap_bt_busy_level_updated(uint8_t bl_flags)
{
    esp_bt_gap_cb_param_t param;

    if (bl_flags == BTM_BL_INQUIRY_STARTED) {
        param.disc_st_chg.state = ESP_BT_GAP_DISCOVERY_STARTED;
        btc_gap_bt_cb_to_app(ESP_BT_GAP_DISC_STATE_CHANGED_EVT, &param);
        btc_gap_bt_inquiry_in_progress = true;
    } else if (bl_flags == BTM_BL_INQUIRY_CANCELLED) {
        param.disc_st_chg.state = ESP_BT_GAP_DISCOVERY_STOPPED;
        btc_gap_bt_cb_to_app(ESP_BT_GAP_DISC_STATE_CHANGED_EVT, &param);
        btc_gap_bt_inquiry_in_progress = false;
    } else if (bl_flags == BTM_BL_INQUIRY_COMPLETE) {
        /* The Inquiry Complete event is not transported to app layer,
        since the app only cares about the Name Discovery Complete event */
        btc_gap_bt_inquiry_in_progress = false;
    }
}

void btc_gap_bt_cb_deep_free(btc_msg_t *msg)
{
    switch (msg->act) {
    case BTC_GAP_BT_SEARCH_DEVICES_EVT:
    case BTC_GAP_BT_SEARCH_SERVICES_EVT:
    case BTC_GAP_BT_SEARCH_SERVICE_RECORD_EVT:
        osi_free(((tBTA_DM_SEARCH_PARAM *) (msg->arg)) ->p_data);
        break;
    case BTC_GAP_BT_READ_RSSI_DELTA_EVT:
    case BTC_GAP_BT_CONFIG_EIR_DATA_EVT:
    case BTC_GAP_BT_AUTH_CMPL_EVT:
    case BTC_GAP_BT_ENC_CHG_EVT:
    case BTC_GAP_BT_PIN_REQ_EVT:
    case BTC_GAP_BT_SET_AFH_CHANNELS_EVT:
    case BTC_GAP_BT_READ_REMOTE_NAME_EVT:
    case BTC_GAP_BT_REMOVE_BOND_DEV_COMPLETE_EVT:
    case BTC_GAP_BT_QOS_EVT:
    case BTC_GAP_BT_SET_PAGE_TO_EVT:
    case BTC_GAP_BT_GET_PAGE_TO_EVT:
    case BTC_GAP_BT_SET_ACL_PKT_TYPES_EVT:
    case BTC_GAP_BT_CFM_REQ_EVT:
    case BTC_GAP_BT_KEY_NOTIF_EVT:
    case BTC_GAP_BT_KEY_REQ_EVT:
#if (BTC_DM_PM_INCLUDED == TRUE)
    case BTC_GAP_BT_MODE_CHG_EVT:
#endif /// BTC_DM_PM_INCLUDED == TRUE
#if (ENC_KEY_SIZE_CTRL_MODE != ENC_KEY_SIZE_CTRL_MODE_NONE)
    case BTC_GAP_BT_SET_MIN_ENC_KEY_SIZE_EVT:
#endif /// ENC_KEY_SIZE_CTRL_MODE != ENC_KEY_SIZE_CTRL_MODE_NONE
    case BTC_GAP_BT_GET_DEV_NAME_CMPL_EVT:
        break;
    default:
        BTC_TRACE_ERROR("%s: Unhandled event (%d)!\n", __FUNCTION__, msg->act);
        break;
    }
}

void btc_gap_bt_cb_handler(btc_msg_t *msg)
{
    switch (msg->act) {
    case BTC_GAP_BT_SEARCH_DEVICES_EVT: {
        btc_gap_bt_search_devices_evt((tBTA_DM_SEARCH_PARAM *)msg->arg);
        break;
    }
    case BTC_GAP_BT_SEARCH_SERVICES_EVT: {
        btc_gap_bt_search_services((char *)msg->arg);
        break;
    }
    case BTC_GAP_BT_SEARCH_SERVICE_RECORD_EVT: {
        btc_gap_bt_search_service_record((char *)msg->arg);
        break;
    }
    case BTC_GAP_BT_READ_RSSI_DELTA_EVT:{
        btc_gap_bt_cb_to_app(ESP_BT_GAP_READ_RSSI_DELTA_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_CONFIG_EIR_DATA_EVT: {
        btc_gap_bt_cb_to_app(ESP_BT_GAP_CONFIG_EIR_DATA_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_AUTH_CMPL_EVT:{
        btc_gap_bt_cb_to_app(ESP_BT_GAP_AUTH_CMPL_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_ENC_CHG_EVT:{
        btc_gap_bt_cb_to_app(ESP_BT_GAP_ENC_CHG_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_PIN_REQ_EVT:{
        btc_gap_bt_cb_to_app(ESP_BT_GAP_PIN_REQ_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_CFM_REQ_EVT:{
        btc_gap_bt_cb_to_app(ESP_BT_GAP_CFM_REQ_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_KEY_NOTIF_EVT:{
        btc_gap_bt_cb_to_app(ESP_BT_GAP_KEY_NOTIF_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_KEY_REQ_EVT:{
        btc_gap_bt_cb_to_app(ESP_BT_GAP_KEY_REQ_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_SET_AFH_CHANNELS_EVT:{
        btc_gap_bt_cb_to_app(ESP_BT_GAP_SET_AFH_CHANNELS_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
#if (SDP_INCLUDED == TRUE)
    case BTC_GAP_BT_READ_REMOTE_NAME_EVT:{
        btc_gap_bt_cb_to_app(ESP_BT_GAP_READ_REMOTE_NAME_EVT,(esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
#endif

#if (BTC_DM_PM_INCLUDED == TRUE)
    case BTC_GAP_BT_MODE_CHG_EVT:
        btc_gap_bt_cb_to_app(ESP_BT_GAP_MODE_CHG_EVT,(esp_bt_gap_cb_param_t *)msg->arg);
        break;
#endif /// BTC_DM_PM_INCLUDED == TRUE
    case BTC_GAP_BT_REMOVE_BOND_DEV_COMPLETE_EVT:{
        btc_gap_bt_cb_to_app(ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT,(esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }

    case BTC_GAP_BT_QOS_EVT:{
        btc_gap_bt_cb_to_app(ESP_BT_GAP_QOS_CMPL_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_SET_PAGE_TO_EVT: {
        btc_gap_bt_cb_to_app(ESP_BT_GAP_SET_PAGE_TO_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_GET_PAGE_TO_EVT: {
        btc_gap_bt_cb_to_app(ESP_BT_GAP_GET_PAGE_TO_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    case BTC_GAP_BT_SET_ACL_PKT_TYPES_EVT: {
        btc_gap_bt_cb_to_app(ESP_BT_GAP_ACL_PKT_TYPE_CHANGED_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
#if (ENC_KEY_SIZE_CTRL_MODE != ENC_KEY_SIZE_CTRL_MODE_NONE)
    case BTC_GAP_BT_SET_MIN_ENC_KEY_SIZE_EVT: {
        btc_gap_bt_cb_to_app(ESP_BT_GAP_SET_MIN_ENC_KEY_SIZE_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
#endif
    case BTC_GAP_BT_GET_DEV_NAME_CMPL_EVT: {
        btc_gap_bt_cb_to_app(ESP_BT_GAP_GET_DEV_NAME_CMPL_EVT, (esp_bt_gap_cb_param_t *)msg->arg);
        break;
    }
    default:
        BTC_TRACE_ERROR("%s: Unhandled event (%d)!\n", __FUNCTION__, msg->act);
        break;
    }
    btc_gap_bt_cb_deep_free(msg);
}
#endif /* (BTC_GAP_BT_INCLUDED == TRUE) */
