#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "xbox_input.h"

#define XBOX_JOY_MAX 65535.0f
#define XBOX_JOY_CENTER 32767.5f

#define XBOX_BLE_APP_ID 0
#define XBOX_BLE_UUID_HID_SERVICE 0x1812
#define XBOX_BLE_UUID_CCCD 0x2902
#define XBOX_BLE_UUID_REPORT_CHAR 0x2A4D
#define XBOX_MAX_NOTIFY_CHARS 8

typedef struct {
    bool bt_ready;
    bool scan_started;
    bool pending_connect;
    bool connecting;
    bool connected;
    esp_gatt_if_t gattc_if;
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
    esp_ble_addr_type_t remote_addr_type;
    uint16_t hid_start_handle;
    uint16_t hid_end_handle;
    uint16_t notify_char_handle;
    uint16_t write_char_handle;
    uint16_t notify_char_handles[XBOX_MAX_NOTIFY_CHARS];
    uint8_t notify_char_count;
} xbox_input_ble_ctx_t;

typedef struct {
    xbox_input_state_t state;
    xbox_input_cmd_t latest_cmd;
    xbox_input_config_t cfg;
    SemaphoreHandle_t mutex;
    QueueHandle_t cmd_queue;
    TaskHandle_t task;
    bool initialized;
    xbox_input_ble_ctx_t ble;
} xbox_input_ctx_t;

extern xbox_input_ctx_t g_xbox_input_ctx;

void xbox_input_publish_cmd(float vx, float vy, float wz, bool connected);
uint8_t xbox_input_parse_report(const uint8_t *data,
                                size_t length,
                                xbox_input_state_t *state);
esp_err_t xbox_input_ble_write_hid_report(const uint8_t *data, size_t length);
