#include "xbox_input_internal.h"

#include <stdlib.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "xbox_input";

static esp_ble_scan_params_t kBleScanParams = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
};

static void start_scan(void);
static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gattc_cb(esp_gattc_cb_event_t event,
                     esp_gatt_if_t gattc_if,
                     esp_ble_gattc_cb_param_t *param);

static void reset_notify_handles(void) {
    g_xbox_input_ctx.ble.hid_start_handle = 0;
    g_xbox_input_ctx.ble.hid_end_handle = 0;
    g_xbox_input_ctx.ble.notify_char_handle = 0;
    g_xbox_input_ctx.ble.write_char_handle = 0;
    memset(g_xbox_input_ctx.ble.notify_char_handles, 0,
           sizeof(g_xbox_input_ctx.ble.notify_char_handles));
    g_xbox_input_ctx.ble.notify_char_count = 0;
}

esp_err_t xbox_input_ble_write_hid_report(const uint8_t *data, size_t length) {
    if (data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_xbox_input_ctx.initialized || !g_xbox_input_ctx.ble.bt_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_xbox_input_ctx.ble.gattc_if == ESP_GATT_IF_NONE ||
        !g_xbox_input_ctx.ble.connected ||
        g_xbox_input_ctx.ble.write_char_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_gatt_write_type_t write_type = ESP_GATT_WRITE_TYPE_NO_RSP;
    esp_err_t err = esp_ble_gattc_write_char(g_xbox_input_ctx.ble.gattc_if,
                                             g_xbox_input_ctx.ble.conn_id,
                                             g_xbox_input_ctx.ble.write_char_handle,
                                             (uint16_t)length,
                                             (uint8_t *)data,
                                             write_type,
                                             ESP_GATT_AUTH_REQ_NONE);
    if (err == ESP_OK) {
        return ESP_OK;
    }

    write_type = ESP_GATT_WRITE_TYPE_RSP;
    return esp_ble_gattc_write_char(g_xbox_input_ctx.ble.gattc_if,
                                    g_xbox_input_ctx.ble.conn_id,
                                    g_xbox_input_ctx.ble.write_char_handle,
                                    (uint16_t)length,
                                    (uint8_t *)data,
                                    write_type,
                                    ESP_GATT_AUTH_REQ_NONE);
}

static bool is_xbox_hid_advertisement(const uint8_t *adv_data,
                                      uint8_t adv_data_len) {
    (void)adv_data_len;

    uint8_t len = 0;
    uint8_t *service16 = esp_ble_resolve_adv_data((uint8_t *)adv_data,
                                                  ESP_BLE_AD_TYPE_16SRV_CMPL,
                                                  &len);
    for (uint8_t i = 0; service16 != NULL && (i + 1U) < len; i += 2) {
        const uint16_t uuid = (uint16_t)service16[i] |
                              ((uint16_t)service16[i + 1] << 8);
        if (uuid == XBOX_BLE_UUID_HID_SERVICE) {
            return true;
        }
    }

    uint8_t name_len = 0;
    uint8_t *name = esp_ble_resolve_adv_data((uint8_t *)adv_data,
                                             ESP_BLE_AD_TYPE_NAME_CMPL,
                                             &name_len);
    if (name == NULL) {
        name = esp_ble_resolve_adv_data((uint8_t *)adv_data,
                                        ESP_BLE_AD_TYPE_NAME_SHORT,
                                        &name_len);
    }
    if (name != NULL && name_len > 0) {
        const char token[] = "Xbox";
        for (uint8_t i = 0; i + sizeof(token) - 1U <= name_len; ++i) {
            bool match = true;
            for (size_t j = 0; j < sizeof(token) - 1U; ++j) {
                if (name[i + j] != (uint8_t)token[j]) {
                    match = false;
                    break;
                }
            }
            if (match) {
                return true;
            }
        }
    }

    return false;
}

static void start_scan(void) {
    if (!g_xbox_input_ctx.ble.bt_ready || g_xbox_input_ctx.ble.scan_started) {
        return;
    }

    const esp_err_t err = esp_ble_gap_start_scanning(30);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "start scan failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "starting BLE scan (continuous)");
    }
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "scan params set status=%d", param->scan_param_cmpl.status);
        start_scan();
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            g_xbox_input_ctx.ble.scan_started = true;
            ESP_LOGI(TAG, "BLE scan started");
        } else {
            ESP_LOGW(TAG, "BLE scan start failed status=%d", param->scan_start_cmpl.status);
        }
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE scan stopped status=%d pending_connect=%d",
                 param->scan_stop_cmpl.status,
                 (int)g_xbox_input_ctx.ble.pending_connect);
        g_xbox_input_ctx.ble.scan_started = false;
        if (g_xbox_input_ctx.ble.pending_connect &&
            g_xbox_input_ctx.ble.gattc_if != ESP_GATT_IF_NONE) {
            g_xbox_input_ctx.ble.pending_connect = false;
            g_xbox_input_ctx.ble.connecting = true;
            ESP_LOGI(TAG,
                     "opening GATTC to %02x:%02x:%02x:%02x:%02x:%02x addrType=%u",
                     g_xbox_input_ctx.ble.remote_bda[0], g_xbox_input_ctx.ble.remote_bda[1],
                     g_xbox_input_ctx.ble.remote_bda[2], g_xbox_input_ctx.ble.remote_bda[3],
                     g_xbox_input_ctx.ble.remote_bda[4], g_xbox_input_ctx.ble.remote_bda[5],
                     (unsigned)g_xbox_input_ctx.ble.remote_addr_type);
            const esp_err_t open_err = esp_ble_gattc_open(g_xbox_input_ctx.ble.gattc_if,
                                                          g_xbox_input_ctx.ble.remote_bda,
                                                          g_xbox_input_ctx.ble.remote_addr_type,
                                                          true);
            if (open_err != ESP_OK) {
                g_xbox_input_ctx.ble.connecting = false;
                ESP_LOGW(TAG, "gattc open failed: %s", esp_err_to_name(open_err));
                start_scan();
            }
        } else if (!g_xbox_input_ctx.ble.connecting) {
            start_scan();
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            if (!g_xbox_input_ctx.ble.connecting &&
                !g_xbox_input_ctx.ble.pending_connect &&
                is_xbox_hid_advertisement(param->scan_rst.ble_adv,
                                          param->scan_rst.adv_data_len)) {
                uint8_t name_len = 0;
                const uint8_t *name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                                               ESP_BLE_AD_TYPE_NAME_CMPL,
                                                               &name_len);
                if (name == NULL) {
                    name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                                    ESP_BLE_AD_TYPE_NAME_SHORT,
                                                    &name_len);
                }
                memcpy(g_xbox_input_ctx.ble.remote_bda,
                       param->scan_rst.bda,
                       sizeof(esp_bd_addr_t));
                g_xbox_input_ctx.ble.remote_addr_type = param->scan_rst.ble_addr_type;
                g_xbox_input_ctx.ble.pending_connect = true;
                ESP_LOGI(TAG,
                         "candidate found %02x:%02x:%02x:%02x:%02x:%02x rssi=%d name=%.*s",
                         param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
                         param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5],
                         param->scan_rst.rssi,
                         name_len,
                         (name != NULL) ? (const char *)name : "");
                esp_ble_gap_stop_scanning();
            }
        }
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(TAG, "security request from %02x:%02x:%02x:%02x:%02x:%02x",
                 param->ble_security.ble_req.bd_addr[0],
                 param->ble_security.ble_req.bd_addr[1],
                 param->ble_security.ble_req.bd_addr[2],
                 param->ble_security.ble_req.bd_addr[3],
                 param->ble_security.ble_req.bd_addr[4],
                 param->ble_security.ble_req.bd_addr[5]);
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        ESP_LOGI(TAG,
                 "auth complete success=%d addr=%02x:%02x:%02x:%02x:%02x:%02x fail_reason=0x%x",
                 param->ble_security.auth_cmpl.success,
                 param->ble_security.auth_cmpl.bd_addr[0],
                 param->ble_security.auth_cmpl.bd_addr[1],
                 param->ble_security.auth_cmpl.bd_addr[2],
                 param->ble_security.auth_cmpl.bd_addr[3],
                 param->ble_security.auth_cmpl.bd_addr[4],
                 param->ble_security.auth_cmpl.bd_addr[5],
                 param->ble_security.auth_cmpl.fail_reason);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        ESP_LOGI(TAG, "passkey notif: %u", (unsigned)param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        ESP_LOGI(TAG, "numeric comparison request: %u (auto accept)",
                 (unsigned)param->ble_security.key_notif.passkey);
        esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        ESP_LOGI(TAG, "passkey request -> replying 000000");
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 0);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        ESP_LOGI(TAG, "security key event type=%d", param->ble_security.ble_key.key_type);
        break;
    default:
        break;
    }
}

static void gattc_cb(esp_gattc_cb_event_t event,
                     esp_gatt_if_t gattc_if,
                     esp_ble_gattc_cb_param_t *param) {
    switch (event) {
    case ESP_GATTC_REG_EVT:
        g_xbox_input_ctx.ble.gattc_if = gattc_if;
        ESP_LOGI(TAG, "GATTC registered status=%d if=%d", param->reg.status, gattc_if);
        esp_ble_gap_set_scan_params(&kBleScanParams);
        break;
    case ESP_GATTC_OPEN_EVT:
        g_xbox_input_ctx.ble.connecting = false;
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "open failed: %d", param->open.status);
            g_xbox_input_ctx.ble.connected = false;
            start_scan();
            break;
        }

        g_xbox_input_ctx.ble.connected = true;
        g_xbox_input_ctx.ble.conn_id = param->open.conn_id;
        memcpy(g_xbox_input_ctx.ble.remote_bda,
               param->open.remote_bda,
               sizeof(esp_bd_addr_t));
        reset_notify_handles();

        ESP_LOGI(TAG, "connected (conn_id=%u), starting security + service discovery",
                 (unsigned)g_xbox_input_ctx.ble.conn_id);
        esp_ble_set_encryption(g_xbox_input_ctx.ble.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);

        esp_bt_uuid_t hid_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = XBOX_BLE_UUID_HID_SERVICE},
        };
        esp_ble_gattc_search_service(gattc_if, g_xbox_input_ctx.ble.conn_id, &hid_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
            param->search_res.srvc_id.uuid.uuid.uuid16 == XBOX_BLE_UUID_HID_SERVICE) {
            g_xbox_input_ctx.ble.hid_start_handle = param->search_res.start_handle;
            g_xbox_input_ctx.ble.hid_end_handle = param->search_res.end_handle;
        }
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (g_xbox_input_ctx.ble.hid_start_handle == 0 ||
            g_xbox_input_ctx.ble.hid_end_handle == 0) {
            ESP_LOGW(TAG, "HID service not found");
            esp_ble_gattc_close(gattc_if, g_xbox_input_ctx.ble.conn_id);
            break;
        }
        ESP_LOGI(TAG, "HID service handles start=%u end=%u",
                 (unsigned)g_xbox_input_ctx.ble.hid_start_handle,
                 (unsigned)g_xbox_input_ctx.ble.hid_end_handle);
        {
            uint16_t count = 0;
            esp_ble_gattc_get_attr_count(gattc_if,
                                         g_xbox_input_ctx.ble.conn_id,
                                         ESP_GATT_DB_CHARACTERISTIC,
                                         g_xbox_input_ctx.ble.hid_start_handle,
                                         g_xbox_input_ctx.ble.hid_end_handle,
                                         0,
                                         &count);
            if (count == 0) {
                ESP_LOGW(TAG, "no characteristics in HID service");
                break;
            }

            esp_gattc_char_elem_t *chars =
                (esp_gattc_char_elem_t *)calloc(count, sizeof(esp_gattc_char_elem_t));
            if (chars == NULL) {
                break;
            }

            if (esp_ble_gattc_get_all_char(gattc_if,
                                           g_xbox_input_ctx.ble.conn_id,
                                           g_xbox_input_ctx.ble.hid_start_handle,
                                           g_xbox_input_ctx.ble.hid_end_handle,
                                           chars,
                                           &count,
                                           0) == ESP_GATT_OK) {
                for (uint16_t i = 0; i < count; ++i) {
                    uint16_t char_uuid16 = 0;
                    if (chars[i].uuid.len == ESP_UUID_LEN_16) {
                        char_uuid16 = chars[i].uuid.uuid.uuid16;
                    }
                    ESP_LOGI(TAG,
                             "char handle=%u props=0x%02x uuid16=0x%04x",
                             (unsigned)chars[i].char_handle,
                             (unsigned)chars[i].properties,
                             (unsigned)char_uuid16);

                    const bool can_write =
                        ((chars[i].properties & ESP_GATT_CHAR_PROP_BIT_WRITE) != 0) ||
                        ((chars[i].properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) != 0);
                    const bool is_report_char =
                        (chars[i].uuid.len == ESP_UUID_LEN_16) &&
                        (chars[i].uuid.uuid.uuid16 == XBOX_BLE_UUID_REPORT_CHAR);

                    if (can_write) {
                        if (g_xbox_input_ctx.ble.write_char_handle == 0 || is_report_char) {
                            g_xbox_input_ctx.ble.write_char_handle = chars[i].char_handle;
                        }
                    }

                    if ((chars[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) == 0) {
                        continue;
                    }
                    if (g_xbox_input_ctx.ble.notify_char_count >= XBOX_MAX_NOTIFY_CHARS) {
                        continue;
                    }
                    if (is_report_char) {
                        for (uint8_t k = g_xbox_input_ctx.ble.notify_char_count; k > 0; --k) {
                            g_xbox_input_ctx.ble.notify_char_handles[k] =
                                g_xbox_input_ctx.ble.notify_char_handles[k - 1];
                        }
                        g_xbox_input_ctx.ble.notify_char_handles[0] = chars[i].char_handle;
                        g_xbox_input_ctx.ble.notify_char_count++;
                    } else {
                        g_xbox_input_ctx.ble.notify_char_handles[g_xbox_input_ctx.ble.notify_char_count++] =
                            chars[i].char_handle;
                    }
                }
            }
            free(chars);

            if (g_xbox_input_ctx.ble.notify_char_count > 0) {
                g_xbox_input_ctx.ble.notify_char_handle =
                    g_xbox_input_ctx.ble.notify_char_handles[0];
                ESP_LOGI(TAG,
                         "registering notify for %u characteristic(s), first handle=%u write handle=%u",
                         (unsigned)g_xbox_input_ctx.ble.notify_char_count,
                         (unsigned)g_xbox_input_ctx.ble.notify_char_handle,
                         (unsigned)g_xbox_input_ctx.ble.write_char_handle);
                for (uint8_t i = 0; i < g_xbox_input_ctx.ble.notify_char_count; ++i) {
                    esp_ble_gattc_register_for_notify(gattc_if,
                                                      g_xbox_input_ctx.ble.remote_bda,
                                                      g_xbox_input_ctx.ble.notify_char_handles[i]);
                }
            } else {
                ESP_LOGW(TAG, "no notify characteristic found in HID service");
                esp_ble_gattc_close(gattc_if, g_xbox_input_ctx.ble.conn_id);
            }
        }
        {
            esp_ble_conn_update_params_t params = {0};
            memcpy(params.bda,
                   g_xbox_input_ctx.ble.remote_bda,
                   sizeof(esp_bd_addr_t));
            params.latency = 0;
            params.timeout = 400;
            params.min_int = 0x10;
            params.max_int = 0x20;
            esp_ble_gap_update_conn_params(&params);
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        if (param->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "register notify failed: %d", param->reg_for_notify.status);
            break;
        }
        {
            uint16_t count = 0;
            esp_ble_gattc_get_attr_count(gattc_if,
                                         g_xbox_input_ctx.ble.conn_id,
                                         ESP_GATT_DB_DESCRIPTOR,
                                         g_xbox_input_ctx.ble.hid_start_handle,
                                         g_xbox_input_ctx.ble.hid_end_handle,
                                         param->reg_for_notify.handle,
                                         &count);
            if (count == 0) {
                ESP_LOGW(TAG, "no descriptors for notify handle=%u",
                         (unsigned)param->reg_for_notify.handle);
                break;
            }

            esp_gattc_descr_elem_t *descs =
                (esp_gattc_descr_elem_t *)calloc(count, sizeof(esp_gattc_descr_elem_t));
            if (descs == NULL) {
                break;
            }
            if (esp_ble_gattc_get_all_descr(gattc_if,
                                            g_xbox_input_ctx.ble.conn_id,
                                            param->reg_for_notify.handle,
                                            descs,
                                            &count,
                                            0) == ESP_GATT_OK) {
                bool cccd_found = false;
                for (uint16_t i = 0; i < count; ++i) {
                    if (descs[i].uuid.len == ESP_UUID_LEN_16 &&
                        descs[i].uuid.uuid.uuid16 == XBOX_BLE_UUID_CCCD) {
                        cccd_found = true;
                        uint16_t notify_en = 1;
                        esp_ble_gattc_write_char_descr(gattc_if,
                                                       g_xbox_input_ctx.ble.conn_id,
                                                       descs[i].handle,
                                                       sizeof(notify_en),
                                                       (uint8_t *)&notify_en,
                                                       ESP_GATT_WRITE_TYPE_RSP,
                                                       ESP_GATT_AUTH_REQ_NONE);
                        ESP_LOGI(TAG,
                                 "writing CCCD handle=%u for notify handle=%u",
                                 (unsigned)descs[i].handle,
                                 (unsigned)param->reg_for_notify.handle);
                    }
                }
                if (!cccd_found) {
                    ESP_LOGW(TAG, "CCCD not found for notify handle=%u",
                             (unsigned)param->reg_for_notify.handle);
                }
            }
            free(descs);
        }
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (param->write.status == ESP_GATT_OK) {
            ESP_LOGI(TAG, "notify subscription enabled");
            xbox_input_set_connected(true);
        } else {
            ESP_LOGW(TAG, "write descriptor failed status=%d", param->write.status);
        }
        break;
    case ESP_GATTC_NOTIFY_EVT:
        if (param->notify.value_len >= 2) {
            ESP_LOGD(TAG,
                     "notify handle=%u len=%u b0=0x%02x b1=0x%02x",
                     (unsigned)param->notify.handle,
                     (unsigned)param->notify.value_len,
                     (unsigned)param->notify.value[0],
                     (unsigned)param->notify.value[1]);
        } else {
            ESP_LOGD(TAG,
                     "notify handle=%u len=%u",
                     (unsigned)param->notify.handle,
                     (unsigned)param->notify.value_len);
        }
        {
            const uint8_t rc = xbox_input_update_report(param->notify.value,
                                                        param->notify.value_len);
            if (rc != XBOX_INPUT_ERR_NONE) {
                ESP_LOGW(TAG, "notify parse error rc=%u len=%u",
                         (unsigned)rc,
                         (unsigned)param->notify.value_len);
            }
        }
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGW(TAG, "controller disconnected");
        g_xbox_input_ctx.ble.connected = false;
        g_xbox_input_ctx.ble.conn_id = 0;
        xbox_input_set_connected(false);
        reset_notify_handles();
        start_scan();
        break;
    default:
        break;
    }
}

esp_err_t xbox_input_start_ble(void) {
    if (!g_xbox_input_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_xbox_input_ctx.ble.bt_ready) {
        return ESP_OK;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_bluedroid_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND | ESP_LE_AUTH_REQ_MITM;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH,
                                   &auth_option,
                                   sizeof(auth_option));

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(XBOX_BLE_APP_ID));

    g_xbox_input_ctx.ble.bt_ready = true;
    ESP_LOGI(TAG, "BLE transport started");
    return ESP_OK;
}
