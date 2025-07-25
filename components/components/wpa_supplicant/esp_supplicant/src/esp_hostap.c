/*
 * SPDX-FileCopyrightText: 2019-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/includes.h"

#include "utils/common.h"
#include "utils/eloop.h"
#include "crypto/sha1.h"
#include "common/ieee802_11_defs.h"
#include "common/eapol_common.h"
#include "ap/wpa_auth.h"
#include "ap/ap_config.h"
#include "utils/wpa_debug.h"
#include "ap/hostapd.h"
#include "ap/wpa_auth_i.h"
#include "esp_wifi_driver.h"
#include "esp_wifi_types.h"
#include "esp_wpa3_i.h"
#include "esp_wps.h"
#include "esp_wps_i.h"

#include "ap/sta_info.h"
#include "common/sae.h"
#include "ap/ieee802_11.h"
#define WIFI_PASSWORD_LEN_MAX 65

struct hostapd_data *global_hapd;

#ifdef CONFIG_SAE
extern SemaphoreHandle_t g_wpa3_hostap_auth_api_lock;
#endif /* CONFIG_SAE */

struct hostapd_data *hostapd_get_hapd_data(void)
{
    return global_hapd;
}

void *hostap_init(void)
{
    struct wifi_ssid *ssid = esp_wifi_ap_get_prof_ap_ssid_internal();
    struct hostapd_data *hapd = NULL;
    struct wpa_auth_config *auth_conf;
    u16 spp_attrubute = 0;
    u8 pairwise_cipher;
    wifi_pmf_config_t pmf_cfg = {0};
    uint8_t authmode;

    hapd = (struct hostapd_data *)os_zalloc(sizeof(struct hostapd_data));

    if (hapd == NULL) {
        return NULL;
    }

    hapd->conf = (struct hostapd_bss_config *)os_zalloc(sizeof(struct hostapd_bss_config));

    if (hapd->conf == NULL) {
        os_free(hapd);
        return NULL;
    }

    auth_conf = (struct wpa_auth_config *)os_zalloc(sizeof(struct  wpa_auth_config));

    if (auth_conf == NULL) {
        goto fail;
    }

    hapd->conf->sae_pwe = esp_wifi_get_config_sae_pwe_h2e_internal(WIFI_IF_AP);
    auth_conf->sae_pwe = hapd->conf->sae_pwe;

    authmode = esp_wifi_ap_get_prof_authmode_internal();
    if (authmode == WIFI_AUTH_WPA_PSK) {
        auth_conf->wpa = WPA_PROTO_WPA;
    }
    if (authmode == WIFI_AUTH_WPA2_PSK) {
        auth_conf->wpa = WPA_PROTO_RSN;
    }
    if (authmode == WIFI_AUTH_WPA_WPA2_PSK) {
        auth_conf->wpa = WPA_PROTO_RSN | WPA_PROTO_WPA;
    }
    if (authmode == WIFI_AUTH_WPA3_PSK || authmode == WIFI_AUTH_WPA2_WPA3_PSK) {
        auth_conf->wpa = WPA_PROTO_RSN;
    }

    pairwise_cipher = esp_wifi_ap_get_prof_pairwise_cipher_internal();

#ifdef CONFIG_IEEE80211W
    if ((auth_conf->wpa & WPA_PROTO_RSN) == WPA_PROTO_RSN) {
        esp_wifi_get_pmf_config_internal(&pmf_cfg, WIFI_IF_AP);
        if (pmf_cfg.required) {
            pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
        }
    }
#endif /* CONFIG_IEEE80211W */

    /* TKIP is compulsory in WPA Mode */
    if (auth_conf->wpa == WPA_PROTO_WPA && pairwise_cipher == WIFI_CIPHER_TYPE_CCMP) {
        pairwise_cipher = WIFI_CIPHER_TYPE_TKIP_CCMP;
    }
    if (pairwise_cipher == WIFI_CIPHER_TYPE_TKIP) {
        auth_conf->wpa_group = WPA_CIPHER_TKIP;
        auth_conf->wpa_pairwise = WPA_CIPHER_TKIP;
        auth_conf->rsn_pairwise = WPA_CIPHER_TKIP;
    } else if (pairwise_cipher == WIFI_CIPHER_TYPE_CCMP) {
        auth_conf->wpa_group = WPA_CIPHER_CCMP;
        auth_conf->wpa_pairwise = WPA_CIPHER_CCMP;
        auth_conf->rsn_pairwise = WPA_CIPHER_CCMP;
    } else {
        auth_conf->wpa_group = WPA_CIPHER_TKIP;
        auth_conf->wpa_pairwise = WPA_CIPHER_CCMP | WPA_CIPHER_TKIP;
        auth_conf->rsn_pairwise = WPA_CIPHER_CCMP | WPA_CIPHER_TKIP;
    }

    auth_conf->wpa_key_mgmt = WPA_KEY_MGMT_PSK;
    auth_conf->eapol_version = EAPOL_VERSION;

    hapd->conf->sae_anti_clogging_threshold = SAE_ANTI_CLOGGING_THRESHOLD;
#ifdef CONFIG_IEEE80211W
    if (pmf_cfg.required && pmf_cfg.capable) {
        auth_conf->ieee80211w = MGMT_FRAME_PROTECTION_REQUIRED;
        auth_conf->wpa_key_mgmt = WPA_KEY_MGMT_PSK_SHA256;
        wpa_printf(MSG_DEBUG, "%s :pmf required", __func__);
    } else if (pmf_cfg.capable && !pmf_cfg.required) {
        auth_conf->ieee80211w = MGMT_FRAME_PROTECTION_OPTIONAL;
        auth_conf->wpa_key_mgmt = WPA_KEY_MGMT_PSK;
        wpa_printf(MSG_DEBUG, "%s : pmf optional", __func__);
    }

    if (authmode == WIFI_AUTH_WPA2_WPA3_PSK) {
        auth_conf->wpa_key_mgmt |= WPA_KEY_MGMT_SAE;
    }

    if (authmode == WIFI_AUTH_WPA3_PSK) {
        auth_conf->wpa_key_mgmt = WPA_KEY_MGMT_SAE;
    }
#endif /* CONFIG_IEEE80211W */

    spp_attrubute = esp_wifi_get_spp_attrubute_internal(WIFI_IF_AP);
    auth_conf->spp_sup.capable = ((spp_attrubute & WPA_CAPABILITY_SPP_CAPABLE) ? SPP_AMSDU_CAP_ENABLE : SPP_AMSDU_CAP_DISABLE);
    auth_conf->spp_sup.require = ((spp_attrubute & WPA_CAPABILITY_SPP_REQUIRED) ? SPP_AMSDU_REQ_ENABLE : SPP_AMSDU_REQ_DISABLE);

    memcpy(hapd->conf->ssid.ssid, ssid->ssid, ssid->len);
    hapd->conf->ssid.ssid_len = ssid->len;
    hapd->conf->wpa_key_mgmt = auth_conf->wpa_key_mgmt;
    hapd->conf->ssid.wpa_passphrase = (char *)os_zalloc(WIFI_PASSWORD_LEN_MAX);
    if (hapd->conf->ssid.wpa_passphrase == NULL) {
        goto fail;
    }

#ifdef CONFIG_SAE
    if (authmode == WIFI_AUTH_WPA3_PSK ||
            authmode == WIFI_AUTH_WPA2_WPA3_PSK) {
        if (wpa3_hostap_auth_init(hapd) != 0) {
            goto fail;
        }
    }
#endif /* CONFIG_SAE */

    os_memcpy(hapd->conf->ssid.wpa_passphrase, esp_wifi_ap_get_prof_password_internal(), strlen((char *)esp_wifi_ap_get_prof_password_internal()));
    hapd->conf->ssid.wpa_passphrase[WIFI_PASSWORD_LEN_MAX - 1] = '\0';
    hapd->conf->max_num_sta = esp_wifi_ap_get_max_sta_conn();
    auth_conf->transition_disable = esp_wifi_ap_get_transition_disable_internal();
    if (authmode != WIFI_AUTH_WPA3_PSK &&
            authmode != WIFI_AUTH_WPA2_WPA3_PSK && auth_conf->transition_disable) {
        auth_conf->transition_disable = 0;
        wpa_printf(MSG_DEBUG, "overriding transition_disable config with 0 as authmode is not WPA3");
    }

#ifdef CONFIG_SAE
    auth_conf->sae_require_mfp = 1;
#endif /* CONFIG_SAE */

    //TODO change it when AP support GCMP-PSK
    auth_conf->group_mgmt_cipher = WPA_CIPHER_AES_128_CMAC;

    hapd->conf->ap_max_inactivity = 5 * 60;
    hostapd_setup_wpa_psk(hapd->conf);

    esp_wifi_get_macaddr_internal(WIFI_IF_AP, hapd->own_addr);

    hapd->wpa_auth = wpa_init(hapd->own_addr, auth_conf, NULL);
    if (hapd->wpa_auth == NULL) {
        goto fail;
    }

    esp_wifi_set_appie_internal(WIFI_APPIE_WPA, hapd->wpa_auth->wpa_ie, (uint16_t)hapd->wpa_auth->wpa_ie_len, 0);
    os_free(auth_conf);
    global_hapd = hapd;

    return (void *)hapd;
fail:
    if (hapd->conf->ssid.wpa_passphrase != NULL) {
        os_free(hapd->conf->ssid.wpa_passphrase);
    }
    if (auth_conf != NULL) {
        os_free(auth_conf);
    }
    os_free(hapd->conf);
    os_free(hapd);
    hapd = NULL;
    return NULL;
}

void hostapd_cleanup(struct hostapd_data *hapd)
{
    if (hapd == NULL) {
        return;
    }
    if (hapd->wpa_auth) {
        wpa_deinit(hapd->wpa_auth);
        hapd->wpa_auth = NULL;
    }

    if (hapd->conf) {
        forced_memzero(hapd->conf->ssid.wpa_passphrase, WIFI_PASSWORD_LEN_MAX);
        os_free(hapd->conf->ssid.wpa_passphrase);
        hostapd_config_free_bss(hapd->conf);
        hapd->conf = NULL;
    }

#ifdef CONFIG_SAE

    struct hostapd_sae_commit_queue *q, *tmp;

    if (dl_list_empty(&hapd->sae_commit_queue)) {
        dl_list_for_each_safe(q, tmp, &hapd->sae_commit_queue,
                              struct hostapd_sae_commit_queue, list) {
            dl_list_del(&q->list);
            os_free(q);
        }
    }

#endif /* CONFIG_SAE */
#ifdef CONFIG_WPS_REGISTRAR
    if (esp_wifi_get_wps_type_internal() != WPS_TYPE_DISABLE ||
            esp_wifi_get_wps_status_internal() != WPS_STATUS_DISABLE) {
        esp_wifi_ap_wps_disable();
    }
#endif /* CONFIG_WPS_REGISTRAR */
    os_free(hapd);
    global_hapd = NULL;

}

bool hostap_deinit(void *data)
{
    struct hostapd_data *hapd = (struct hostapd_data *)data;

    if (hapd == NULL) {
        return true;
    }
    esp_wifi_unset_appie_internal(WIFI_APPIE_WPA);
    esp_wifi_unset_appie_internal(WIFI_APPIE_ASSOC_RESP);

#ifdef CONFIG_WPS_REGISTRAR
    wifi_ap_wps_disable_internal();
#endif
#ifdef CONFIG_SAE
    wpa3_hostap_auth_deinit();
    /* Wait till lock is released by wpa3 task */
    if (g_wpa3_hostap_auth_api_lock &&
            WPA3_HOSTAP_AUTH_API_LOCK() == pdTRUE) {
        WPA3_HOSTAP_AUTH_API_UNLOCK();
    }
#endif /* CONFIG_SAE */

    hostapd_cleanup(hapd);

    return true;
}

int esp_wifi_build_rsnxe(struct hostapd_data *hapd, u8 *eid, size_t len)
{
    u8 *pos = eid;
    u16 capab = 0;
    size_t flen;

    if (!(hapd->wpa_auth->conf.wpa & WPA_PROTO_RSN)) {
        return 0;
    }

    if (wpa_key_mgmt_sae(hapd->wpa_auth->conf.wpa_key_mgmt) &&
            (hapd->conf->sae_pwe == SAE_PWE_HASH_TO_ELEMENT
             || hapd->conf->sae_pwe == SAE_PWE_BOTH)) {
        capab |= BIT(WLAN_RSNX_CAPAB_SAE_H2E);
    }

    flen = 1;
    if (len < 2 + flen || !capab) {
        return 0; /* no supported extended RSN capabilities */
    }
    capab |= flen - 1; /* bit 0-3 = Field length (n - 1) */

    *pos++ = WLAN_EID_RSNX;
    *pos++ = flen;
    *pos++ = capab & 0x00ff;

    return pos - eid;
}

u16 esp_send_assoc_resp(struct hostapd_data *hapd, const u8 *addr,
                        u16 status_code, bool omit_rsnxe, int subtype)
{
#define ASSOC_RESP_LENGTH 20
    u8 buf[ASSOC_RESP_LENGTH];
    wifi_mgmt_frm_req_t *reply = NULL;
    int send_len = 0;

    int res = WLAN_STATUS_SUCCESS;

    if (!omit_rsnxe) {
        send_len = esp_wifi_build_rsnxe(hapd, buf, ASSOC_RESP_LENGTH);
    }

    esp_wifi_set_appie_internal(WIFI_APPIE_ASSOC_RESP, buf, send_len, 0);

    reply = os_zalloc(sizeof(wifi_mgmt_frm_req_t) + sizeof(uint16_t));
    if (!reply) {
        wpa_printf(MSG_ERROR, "failed to allocate memory for assoc response");
        return WLAN_STATUS_UNSPECIFIED_FAILURE;
    }
    reply->ifx = WIFI_IF_AP;
    reply->subtype = subtype;
    os_memcpy(reply->da, addr, ETH_ALEN);
    reply->data_len = sizeof(uint16_t);

    ((uint16_t *)reply->data)[0] = status_code;
    if (esp_wifi_send_mgmt_frm_internal(reply) != 0) {
        res = WLAN_STATUS_UNSPECIFIED_FAILURE;
        wpa_printf(MSG_INFO, "esp_send_assoc_resp_failed: send failed");
    }
#undef ASSOC_RESP_LENGTH
    os_free(reply);
    return res;
}

uint8_t wpa_status_to_reason_code(int status)
{
    switch (status) {
    case WLAN_STATUS_INVALID_IE:
        return WLAN_REASON_INVALID_IE;
    case WLAN_STATUS_GROUP_CIPHER_NOT_VALID:
        return WLAN_REASON_GROUP_CIPHER_NOT_VALID;
    case WLAN_STATUS_PAIRWISE_CIPHER_NOT_VALID:
        return WLAN_REASON_PAIRWISE_CIPHER_NOT_VALID;
    case WLAN_STATUS_AKMP_NOT_VALID:
        return WLAN_REASON_AKMP_NOT_VALID;
    case WLAN_STATUS_CIPHER_REJECTED_PER_POLICY:
        return WLAN_REASON_CIPHER_SUITE_REJECTED;
    case WLAN_STATUS_INVALID_PMKID:
        return WLAN_REASON_INVALID_PMKID;
    case WLAN_STATUS_INVALID_MDIE:
        return WLAN_REASON_INVALID_MDE;
    default:
        return WLAN_REASON_UNSPECIFIED;
    }
}

bool hostap_new_assoc_sta(struct sta_info *sta, uint8_t *bssid, uint8_t *wpa_ie,
                          uint8_t wpa_ie_len, uint8_t *rsnxe, uint16_t rsnxe_len,
                          bool *pmf_enable, int subtype, uint8_t *pairwise_cipher, uint8_t *reason)
{
    struct hostapd_data *hapd = (struct hostapd_data*)esp_wifi_get_hostap_private_internal();
    enum wpa_validate_result res = WPA_IE_OK;
    int status = WLAN_STATUS_SUCCESS;
    bool omit_rsnxe = false;

    if (!sta || !bssid || !wpa_ie) {
        return false;
    }

    if (hapd) {
        if (hapd->wpa_auth->conf.wpa) {
            if (sta->wpa_sm) {
                wpa_auth_sta_deinit(sta->wpa_sm);
            }

            sta->wpa_sm = wpa_auth_sta_init(hapd->wpa_auth, bssid);
            wpa_printf(MSG_DEBUG, "init wpa sm=%p", sta->wpa_sm);

            if (sta->wpa_sm == NULL) {
                status = WLAN_STATUS_AP_UNABLE_TO_HANDLE_NEW_STA;
                goto send_resp;
            }

            res = wpa_validate_wpa_ie(hapd->wpa_auth, sta->wpa_sm, wpa_ie, wpa_ie_len, rsnxe, rsnxe_len);

#ifdef CONFIG_SAE
            if (wpa_auth_uses_sae(sta->wpa_sm) && sta->sae &&
                    sta->sae->state == SAE_ACCEPTED) {
                wpa_auth_add_sae_pmkid(sta->wpa_sm, sta->sae->pmkid);
            }
#endif /* CONFIG_SAE */

            status = wpa_res_to_status_code(res);

send_resp:
            if (!rsnxe) {
                omit_rsnxe = true;
            }

            if (esp_send_assoc_resp(hapd, bssid, status, omit_rsnxe, subtype) != WLAN_STATUS_SUCCESS) {
                status = WLAN_STATUS_AP_UNABLE_TO_HANDLE_NEW_STA;
            }

            if (status != WLAN_STATUS_SUCCESS) {
                *reason = wpa_status_to_reason_code(status);
                return false;
            }

            //Check whether AP uses Management Frame Protection for this connection
            *pmf_enable = wpa_auth_uses_mfp(sta->wpa_sm);
            *pairwise_cipher = GET_BIT_POSITION(sta->wpa_sm->pairwise);
        }

        wpa_auth_sta_associated(hapd->wpa_auth, sta->wpa_sm);
    }

    return true;
}

#ifdef CONFIG_WPS_REGISTRAR
static void ap_free_sta_timeout(void *ctx, void *data)
{
    struct hostapd_data *hapd = (struct hostapd_data *) ctx;
    u8 *addr = (u8 *) data;
    struct sta_info *sta = ap_get_sta(hapd, addr);

    if (sta) {
        ap_free_sta(hapd, sta);
    }

    os_free(addr);
}
#endif

bool wpa_ap_remove(u8* bssid)
{
    struct hostapd_data *hapd = hostapd_get_hapd_data();

    if (!hapd) {
        return false;
    }
    struct sta_info *sta = ap_get_sta(hapd, bssid);
    if (!sta) {
        return false;
    }

#ifdef CONFIG_SAE
    if (sta->lock) {
        if (os_semphr_take(sta->lock, 0)) {
            ap_free_sta(hapd, sta);
        } else {
            sta->remove_pending = true;
        }
        return true;
    }
#endif /* CONFIG_SAE */

#ifdef CONFIG_WPS_REGISTRAR
    wpa_printf(MSG_DEBUG, "wps_status=%d", wps_get_status());
    if (wps_get_status() == WPS_STATUS_PENDING) {
        u8 *addr = os_malloc(ETH_ALEN);

        if (!addr) {
            return false;
        }
        os_memcpy(addr, sta->addr, ETH_ALEN);
        eloop_register_timeout(0, 10000, ap_free_sta_timeout, hapd, addr);
    } else
#endif
        ap_free_sta(hapd, sta);

    return true;
}
