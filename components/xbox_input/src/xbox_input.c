#include "xbox_input.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define XBOX_IDX_BUTTONS_DIR 12
#define XBOX_IDX_BUTTONS_MAIN 13
#define XBOX_IDX_BUTTONS_CENTER 14
#define XBOX_IDX_BUTTONS_SHARE 15

#define XBOX_JOY_MAX 65535.0f
#define XBOX_JOY_CENTER 32767.5f

#define XBOX_BLE_APP_ID 0
#define XBOX_BLE_UUID_HID_SERVICE 0x1812
#define XBOX_BLE_UUID_CCCD 0x2902
#define XBOX_BLE_UUID_REPORT_CHAR 0x2A4D
#define XBOX_MAX_NOTIFY_CHARS 8

static const char *TAG = "xbox_input";

typedef struct {
    xbox_input_state_t state;
    xbox_input_cmd_t latestCmd;
    xbox_input_config_t cfg;
    SemaphoreHandle_t mutex;
    QueueHandle_t cmdQueue;
    TaskHandle_t task;
    bool initialized;

    bool btReady;
    bool scanStarted;
    bool pendingConnect;
    bool connecting;
    esp_gatt_if_t gattcIf;
    uint16_t connId;
    esp_bd_addr_t remoteBda;
    esp_ble_addr_type_t remoteAddrType;
    uint16_t hidStartHandle;
    uint16_t hidEndHandle;
    uint16_t notifyCharHandle;
    uint16_t notifyCharHandles[XBOX_MAX_NOTIFY_CHARS];
    uint8_t notifyCharCount;
} xbox_input_ctx_t;

static xbox_input_ctx_t g_ctx = {
    .cfg = {
        .deadzone = 0.08f,
        .maxVx = 0.15f,
        .maxVy = 0.10f,
        .maxWz = 0.9f,
        .taskPeriodMs = 20,
    },
    .mutex = NULL,
    .cmdQueue = NULL,
    .task = NULL,
    .initialized = false,
    .btReady = false,
    .scanStarted = false,
    .pendingConnect = false,
    .connecting = false,
    .gattcIf = ESP_GATT_IF_NONE,
    .connId = 0,
    .remoteBda = {0},
    .remoteAddrType = BLE_ADDR_TYPE_PUBLIC,
    .hidStartHandle = 0,
    .hidEndHandle = 0,
    .notifyCharHandle = 0,
    .notifyCharHandles = {0},
    .notifyCharCount = 0,
};

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

static float normalize_axis(uint16_t value) {
    return ((float)value - XBOX_JOY_CENTER) / XBOX_JOY_CENTER;
}

static float apply_deadzone(float value, float dz) {
    const float absValue = fabsf(value);
    if (absValue <= dz) {
        return 0.0f;
    }
    const float scaled = (absValue - dz) / (1.0f - dz);
    return copysignf(scaled, value);
}

static void publish_cmd(float vx, float vy, float wz, bool connected) {
    xbox_input_cmd_t cmd = {
        .vx = vx,
        .vy = vy,
        .wz = wz,
        .connected = connected,
        .updatedAtMs = (uint32_t)(esp_timer_get_time() / 1000ULL),
    };
    g_ctx.latestCmd = cmd;
    if (g_ctx.cmdQueue != NULL) {
        xQueueOverwrite(g_ctx.cmdQueue, &cmd);
    }
}

static void xbox_input_task(void *arg) {
    (void)arg;

    while (true) {
        xbox_input_state_t snapshot;
        if (xSemaphoreTake(g_ctx.mutex, portMAX_DELAY) == pdTRUE) {
            snapshot = g_ctx.state;
            xSemaphoreGive(g_ctx.mutex);
        } else {
            memset(&snapshot, 0, sizeof(snapshot));
        }

        float vx = 0.0f;
        float vy = 0.0f;
        float wz = 0.0f;

        if (snapshot.connected) {
            const float leftX = apply_deadzone(normalize_axis(snapshot.joyLHori), g_ctx.cfg.deadzone);
            const float leftY = apply_deadzone(normalize_axis(snapshot.joyLVert), g_ctx.cfg.deadzone);
            const float rightX = apply_deadzone(normalize_axis(snapshot.joyRHori), g_ctx.cfg.deadzone);

            vx = -leftY * g_ctx.cfg.maxVx;
            vy = leftX * g_ctx.cfg.maxVy;
            wz = rightX * g_ctx.cfg.maxWz;
        }

        publish_cmd(vx, vy, wz, snapshot.connected);

        vTaskDelay(pdMS_TO_TICKS(g_ctx.cfg.taskPeriodMs));
    }
}

esp_err_t xbox_input_init(const xbox_input_config_t *cfg) {
    if (g_ctx.initialized) {
        return ESP_OK;
    }

    g_ctx.mutex = xSemaphoreCreateMutex();
    if (g_ctx.mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    g_ctx.cmdQueue = xQueueCreate(1, sizeof(xbox_input_cmd_t));
    if (g_ctx.cmdQueue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (cfg != NULL) {
        g_ctx.cfg = *cfg;
        if (g_ctx.cfg.deadzone < 0.0f) g_ctx.cfg.deadzone = 0.0f;
        if (g_ctx.cfg.deadzone > 0.6f) g_ctx.cfg.deadzone = 0.6f;
        if (g_ctx.cfg.taskPeriodMs == 0) g_ctx.cfg.taskPeriodMs = 20;
    }

    memset(&g_ctx.state, 0, sizeof(g_ctx.state));
    g_ctx.state.joyLHori = (uint16_t)(XBOX_JOY_MAX / 2.0f);
    g_ctx.state.joyLVert = (uint16_t)(XBOX_JOY_MAX / 2.0f);
    g_ctx.state.joyRHori = (uint16_t)(XBOX_JOY_MAX / 2.0f);
    g_ctx.state.joyRVert = (uint16_t)(XBOX_JOY_MAX / 2.0f);

    publish_cmd(0.0f, 0.0f, 0.0f, false);

    g_ctx.initialized = true;
    ESP_LOGI(TAG, "initialized (deadzone=%.2f maxVx=%.2f maxVy=%.2f maxWz=%.2f)",
             (double)g_ctx.cfg.deadzone,
             (double)g_ctx.cfg.maxVx,
             (double)g_ctx.cfg.maxVy,
             (double)g_ctx.cfg.maxWz);
    return ESP_OK;
}

esp_err_t xbox_input_start(void) {
    if (!g_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_ctx.task != NULL) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        xbox_input_task,
        "xbox_input_task",
        4096,
        NULL,
        5,
        &g_ctx.task,
        tskNO_AFFINITY);

    if (ok != pdPASS) {
        g_ctx.task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "task started");
    return ESP_OK;
}

void xbox_input_set_connected(bool connected) {
    if (!g_ctx.initialized) {
        return;
    }
    if (xSemaphoreTake(g_ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_ctx.state.connected = connected;
        xSemaphoreGive(g_ctx.mutex);
    }
    if (!connected) {
        publish_cmd(0.0f, 0.0f, 0.0f, false);
    }
}

uint8_t xbox_input_update_report(const uint8_t *data, size_t length) {
    if (!g_ctx.initialized) {
        return XBOX_INPUT_ERR_INVALID_LENGTH;
    }
    if (data == NULL || length < XBOX_INPUT_REPORT_LEN) {
        return XBOX_INPUT_ERR_INVALID_LENGTH;
    }

    const uint8_t *payload = data;
    size_t payloadLen = length;
    if (length == (XBOX_INPUT_REPORT_LEN + 1)) {
        payload = data + 1;
        payloadLen = XBOX_INPUT_REPORT_LEN;
    }
    if (payloadLen != XBOX_INPUT_REPORT_LEN) {
        return XBOX_INPUT_ERR_INVALID_LENGTH;
    }

    if (xSemaphoreTake(g_ctx.mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return XBOX_INPUT_ERR_INVALID_LENGTH;
    }

    uint8_t btnBits = payload[XBOX_IDX_BUTTONS_MAIN];
    g_ctx.state.btnA = (btnBits & 0b00000001) != 0;
    g_ctx.state.btnB = (btnBits & 0b00000010) != 0;
    g_ctx.state.btnX = (btnBits & 0b00001000) != 0;
    g_ctx.state.btnY = (btnBits & 0b00010000) != 0;
    g_ctx.state.btnLB = (btnBits & 0b01000000) != 0;
    g_ctx.state.btnRB = (btnBits & 0b10000000) != 0;

    btnBits = payload[XBOX_IDX_BUTTONS_CENTER];
    g_ctx.state.btnSelect = (btnBits & 0b00000100) != 0;
    g_ctx.state.btnStart = (btnBits & 0b00001000) != 0;
    g_ctx.state.btnXbox = (btnBits & 0b00010000) != 0;
    g_ctx.state.btnLS = (btnBits & 0b00100000) != 0;
    g_ctx.state.btnRS = (btnBits & 0b01000000) != 0;

    btnBits = payload[XBOX_IDX_BUTTONS_SHARE];
    g_ctx.state.btnShare = (btnBits & 0b00000001) != 0;

    btnBits = payload[XBOX_IDX_BUTTONS_DIR];
    g_ctx.state.btnDirUp = (btnBits == 1 || btnBits == 2 || btnBits == 8);
    g_ctx.state.btnDirRight = (btnBits >= 2 && btnBits <= 4);
    g_ctx.state.btnDirDown = (btnBits >= 4 && btnBits <= 6);
    g_ctx.state.btnDirLeft = (btnBits >= 6 && btnBits <= 8);

    g_ctx.state.joyLHori = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    g_ctx.state.joyLVert = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
    g_ctx.state.joyRHori = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
    g_ctx.state.joyRVert = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
    g_ctx.state.trigLT = (uint16_t)payload[8] | ((uint16_t)payload[9] << 8);
    g_ctx.state.trigRT = (uint16_t)payload[10] | ((uint16_t)payload[11] << 8);
    g_ctx.state.updatedCount++;
    g_ctx.state.lastUpdateMs = (uint32_t)(esp_timer_get_time() / 1000ULL);
    g_ctx.state.connected = true;

    xSemaphoreGive(g_ctx.mutex);
    return 0;
}

void xbox_input_get_state(xbox_input_state_t *outState) {
    if (outState == NULL) {
        return;
    }

    memset(outState, 0, sizeof(*outState));
    if (!g_ctx.initialized) {
        return;
    }

    if (xSemaphoreTake(g_ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *outState = g_ctx.state;
        xSemaphoreGive(g_ctx.mutex);
    }
}

QueueHandle_t xbox_input_get_cmd_queue(void) { return g_ctx.cmdQueue; }

bool xbox_input_try_get_latest_cmd(xbox_input_cmd_t *outCmd) {
    if (outCmd == NULL || !g_ctx.initialized) {
        return false;
    }
    if (xSemaphoreTake(g_ctx.mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }
    *outCmd = g_ctx.latestCmd;
    xSemaphoreGive(g_ctx.mutex);
    return true;
}

static bool is_xbox_hid_advertisement(const uint8_t *advData,
                                      uint8_t advDataLen) {
    uint8_t len = 0;
    uint8_t *service16 = esp_ble_resolve_adv_data((uint8_t *)advData,
                                                  ESP_BLE_AD_TYPE_16SRV_CMPL,
                                                  &len);
    for (uint8_t i = 0; service16 != NULL && (i + 1) < len; i += 2) {
        const uint16_t uuid = (uint16_t)service16[i] |
                              ((uint16_t)service16[i + 1] << 8);
        if (uuid == XBOX_BLE_UUID_HID_SERVICE) {
            return true;
        }
    }

    uint8_t nameLen = 0;
    uint8_t *name = esp_ble_resolve_adv_data((uint8_t *)advData,
                                             ESP_BLE_AD_TYPE_NAME_CMPL,
                                             &nameLen);
    if (name == NULL) {
        name = esp_ble_resolve_adv_data((uint8_t *)advData,
                                        ESP_BLE_AD_TYPE_NAME_SHORT,
                                        &nameLen);
    }
    if (name != NULL && nameLen > 0) {
        const char token[] = "Xbox";
        for (uint8_t i = 0; i + sizeof(token) - 1 <= nameLen; ++i) {
            bool match = true;
            for (size_t j = 0; j < sizeof(token) - 1; ++j) {
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
    if (!g_ctx.btReady || g_ctx.scanStarted) {
        return;
    }
    const esp_err_t err = esp_ble_gap_start_scanning(30); /* 30-second scan window */
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
            g_ctx.scanStarted = true;
            ESP_LOGI(TAG, "BLE scan started");
        } else {
            ESP_LOGW(TAG, "BLE scan start failed status=%d", param->scan_start_cmpl.status);
        }
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE scan stopped status=%d pendingConnect=%d", 
                 param->scan_stop_cmpl.status,
                 (int)g_ctx.pendingConnect);
        g_ctx.scanStarted = false;
        if (g_ctx.pendingConnect && g_ctx.gattcIf != ESP_GATT_IF_NONE) {
            g_ctx.pendingConnect = false;
            g_ctx.connecting = true;
            ESP_LOGI(TAG,
                     "opening GATTC to %02x:%02x:%02x:%02x:%02x:%02x addrType=%u",
                     g_ctx.remoteBda[0], g_ctx.remoteBda[1], g_ctx.remoteBda[2],
                     g_ctx.remoteBda[3], g_ctx.remoteBda[4], g_ctx.remoteBda[5],
                     (unsigned)g_ctx.remoteAddrType);
            const esp_err_t openErr = esp_ble_gattc_open(g_ctx.gattcIf,
                                                         g_ctx.remoteBda,
                                                         g_ctx.remoteAddrType,
                                                         true);
            if (openErr != ESP_OK) {
                g_ctx.connecting = false;
                ESP_LOGW(TAG, "gattc open failed: %s", esp_err_to_name(openErr));
                start_scan();
            }
        } else if (!g_ctx.connecting) {
            start_scan();
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            if (!g_ctx.connecting && !g_ctx.pendingConnect &&
                is_xbox_hid_advertisement(param->scan_rst.ble_adv,
                                          param->scan_rst.adv_data_len)) {
                uint8_t nameLen = 0;
                const uint8_t *name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                                                ESP_BLE_AD_TYPE_NAME_CMPL,
                                                                &nameLen);
                if (name == NULL) {
                    name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                                    ESP_BLE_AD_TYPE_NAME_SHORT,
                                                    &nameLen);
                }
                memcpy(g_ctx.remoteBda, param->scan_rst.bda, sizeof(esp_bd_addr_t));
                g_ctx.remoteAddrType = param->scan_rst.ble_addr_type;
                g_ctx.pendingConnect = true;
                ESP_LOGI(TAG,
                         "candidate found %02x:%02x:%02x:%02x:%02x:%02x rssi=%d name=%.*s",
                         param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
                         param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5],
                         param->scan_rst.rssi,
                         nameLen,
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
        g_ctx.gattcIf = gattc_if;
        ESP_LOGI(TAG, "GATTC registered status=%d if=%d", param->reg.status, gattc_if);
        esp_ble_gap_set_scan_params(&kBleScanParams);
        break;
    case ESP_GATTC_OPEN_EVT:
        g_ctx.connecting = false;

        if (param->open.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "open failed: %d", param->open.status);
        start_scan();
        break;
    }

    g_ctx.connId = param->open.conn_id;
    memcpy(g_ctx.remoteBda, param->open.remote_bda, sizeof(esp_bd_addr_t));

    g_ctx.hidStartHandle = 0;
    g_ctx.hidEndHandle = 0;
    g_ctx.notifyCharHandle = 0;
    memset(g_ctx.notifyCharHandles, 0, sizeof(g_ctx.notifyCharHandles));
    g_ctx.notifyCharCount = 0;

        ESP_LOGI(TAG, "connected (connId=%u), starting security + service discovery",
                 (unsigned)g_ctx.connId);

        /* 1. Request encryption before service discovery */
        esp_ble_set_encryption(g_ctx.remoteBda, ESP_BLE_SEC_ENCRYPT_MITM);

        /* 2. Start service discovery */
        esp_bt_uuid_t hidUuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = XBOX_BLE_UUID_HID_SERVICE},
        };

        esp_ble_gattc_search_service(gattc_if, g_ctx.connId, &hidUuid);

        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
            param->search_res.srvc_id.uuid.uuid.uuid16 == XBOX_BLE_UUID_HID_SERVICE) {
            g_ctx.hidStartHandle = param->search_res.start_handle;
            g_ctx.hidEndHandle = param->search_res.end_handle;
        }
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (g_ctx.hidStartHandle == 0 || g_ctx.hidEndHandle == 0) {
            ESP_LOGW(TAG, "HID service not found");
            esp_ble_gattc_close(gattc_if, g_ctx.connId);
            break;
        }
        ESP_LOGI(TAG, "HID service handles start=%u end=%u",
                 (unsigned)g_ctx.hidStartHandle,
                 (unsigned)g_ctx.hidEndHandle);
        {
            uint16_t count = 0;
            esp_ble_gattc_get_attr_count(gattc_if,
                                         g_ctx.connId,
                                         ESP_GATT_DB_CHARACTERISTIC,
                                         g_ctx.hidStartHandle,
                                         g_ctx.hidEndHandle,
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
                                           g_ctx.connId,
                                           g_ctx.hidStartHandle,
                                           g_ctx.hidEndHandle,
                                           chars,
                                           &count,
                                           0) == ESP_GATT_OK) {
                for (uint16_t i = 0; i < count; ++i) {
                    uint16_t charUuid16 = 0;
                    if (chars[i].uuid.len == ESP_UUID_LEN_16) {
                        charUuid16 = chars[i].uuid.uuid.uuid16;
                    }
                    ESP_LOGI(TAG,
                             "char handle=%u props=0x%02x uuid16=0x%04x",
                             (unsigned)chars[i].char_handle,
                             (unsigned)chars[i].properties,
                             (unsigned)charUuid16);
                    if ((chars[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) != 0) {
                        if (g_ctx.notifyCharCount < XBOX_MAX_NOTIFY_CHARS) {
                            const bool isReportChar =
                                (chars[i].uuid.len == ESP_UUID_LEN_16) &&
                                (chars[i].uuid.uuid.uuid16 == XBOX_BLE_UUID_REPORT_CHAR);
                            if (isReportChar) {
                                for (uint8_t k = g_ctx.notifyCharCount; k > 0; --k) {
                                    g_ctx.notifyCharHandles[k] = g_ctx.notifyCharHandles[k - 1];
                                }
                                g_ctx.notifyCharHandles[0] = chars[i].char_handle;
                                g_ctx.notifyCharCount++;
                            } else {
                                g_ctx.notifyCharHandles[g_ctx.notifyCharCount++] =
                                    chars[i].char_handle;
                            }
                        }
                    }
                }
            }
            free(chars);
            if (g_ctx.notifyCharCount > 0) {
                g_ctx.notifyCharHandle = g_ctx.notifyCharHandles[0];
                ESP_LOGI(TAG,
                         "registering notify for %u characteristic(s), first handle=%u",
                         (unsigned)g_ctx.notifyCharCount,
                         (unsigned)g_ctx.notifyCharHandle);
                for (uint8_t i = 0; i < g_ctx.notifyCharCount; ++i) {
                    esp_ble_gattc_register_for_notify(gattc_if,
                                                      g_ctx.remoteBda,
                                                      g_ctx.notifyCharHandles[i]);
                }
            } else {
                ESP_LOGW(TAG, "no notify characteristic found in HID service");
                esp_ble_gattc_close(gattc_if, g_ctx.connId);
            }
        }
        esp_ble_conn_update_params_t params = {0};

        memcpy(params.bda, g_ctx.remoteBda, sizeof(esp_bd_addr_t));

        params.latency = 0;
        params.timeout = 400;
        params.min_int = 0x10;   // 20 ms
        params.max_int = 0x20;   // 40 ms

        esp_ble_gap_update_conn_params(&params);
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        if (param->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "register notify failed: %d", param->reg_for_notify.status);
            break;
        }
        {
            uint16_t count = 0;
            esp_ble_gattc_get_attr_count(gattc_if,
                                         g_ctx.connId,
                                         ESP_GATT_DB_DESCRIPTOR,
                                         g_ctx.hidStartHandle,
                                         g_ctx.hidEndHandle,
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
                                            g_ctx.connId,
                                            param->reg_for_notify.handle,
                                            descs,
                                            &count,
                                            0) == ESP_GATT_OK) {
                bool cccdFound = false;
                for (uint16_t i = 0; i < count; ++i) {
                    if (descs[i].uuid.len == ESP_UUID_LEN_16 &&
                        descs[i].uuid.uuid.uuid16 == XBOX_BLE_UUID_CCCD) {
                        cccdFound = true;
                        uint16_t notifyEn = 1;
                        esp_ble_gattc_write_char_descr(gattc_if,
                                                       g_ctx.connId,
                                                       descs[i].handle,
                                                       sizeof(notifyEn),
                                                       (uint8_t *)&notifyEn,
                                                       ESP_GATT_WRITE_TYPE_RSP,
                                                       ESP_GATT_AUTH_REQ_NONE);
                        ESP_LOGI(TAG,
                                 "writing CCCD handle=%u for notify handle=%u",
                                 (unsigned)descs[i].handle,
                                 (unsigned)param->reg_for_notify.handle);
                    }
                }
                if (!cccdFound) {
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
        {
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
            const uint8_t rc = xbox_input_update_report(param->notify.value,
                                                        param->notify.value_len);
            if (rc != 0) {
                ESP_LOGW(TAG, "notify parse error rc=%u len=%u",
                         (unsigned)rc,
                         (unsigned)param->notify.value_len);
            }
        }
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGW(TAG, "controller disconnected");
        xbox_input_set_connected(false);
        g_ctx.hidStartHandle = 0;
        g_ctx.hidEndHandle = 0;
        g_ctx.notifyCharHandle = 0;
        memset(g_ctx.notifyCharHandles, 0, sizeof(g_ctx.notifyCharHandles));
        g_ctx.notifyCharCount = 0;
        start_scan();
        break;
    default:
        break;
    }
}

esp_err_t xbox_input_start_ble(void) {
    if (!g_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_ctx.btReady) {
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

    esp_bt_controller_config_t btCfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&btCfg);
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

    esp_ble_auth_req_t authReq = ESP_LE_AUTH_BOND | ESP_LE_AUTH_REQ_MITM;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t keySize = 16;
    uint8_t initKey = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rspKey = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t authOption = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &authReq, sizeof(authReq));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &keySize, sizeof(keySize));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &initKey, sizeof(initKey));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rspKey, sizeof(rspKey));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH,
                                   &authOption,
                                   sizeof(authOption));

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(XBOX_BLE_APP_ID));

    g_ctx.btReady = true;
    ESP_LOGI(TAG, "BLE transport started");
    return ESP_OK;
}
