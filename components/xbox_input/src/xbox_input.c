#include "xbox_input.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "xbox_input_internal.h"

static const char *TAG = "xbox_input";

#define XBOX_INPUT_RUMBLE_REPORT_LEN 8

static uint8_t clamp_percent(uint8_t value) {
    return (value > 100U) ? 100U : value;
}

xbox_input_ctx_t g_xbox_input_ctx = {
    .cfg = {
        .deadzone = 0.08f,
        .max_vx = 0.15f,
        .max_vy = 0.10f,
        .max_wz = 0.9f,
        .task_period_ms = 20,
    },
    .mutex = NULL,
    .cmd_queue = NULL,
    .task = NULL,
    .initialized = false,
    .ble = {
        .bt_ready = false,
        .scan_started = false,
        .pending_connect = false,
        .connecting = false,
        .connected = false,
        .gattc_if = ESP_GATT_IF_NONE,
        .conn_id = 0,
        .remote_bda = {0},
        .remote_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .hid_start_handle = 0,
        .hid_end_handle = 0,
        .notify_char_handle = 0,
        .write_char_handle = 0,
        .notify_char_handles = {0},
        .notify_char_count = 0,
    },
};

static float normalize_axis(uint16_t value) {
    return ((float)value - XBOX_JOY_CENTER) / XBOX_JOY_CENTER;
}

static float apply_deadzone(float value, float dz) {
    const float abs_value = fabsf(value);
    if (abs_value <= dz) {
        return 0.0f;
    }
    const float scaled = (abs_value - dz) / (1.0f - dz);
    return copysignf(scaled, value);
}

void xbox_input_publish_cmd(float vx, float vy, float wz, bool connected) {
    xbox_input_cmd_t cmd = {
        .vx = vx,
        .vy = vy,
        .wz = wz,
        .connected = connected,
        .updated_at_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
    };

    if (g_xbox_input_ctx.mutex != NULL &&
        xSemaphoreTake(g_xbox_input_ctx.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_xbox_input_ctx.latest_cmd = cmd;
        xSemaphoreGive(g_xbox_input_ctx.mutex);
    }

    if (g_xbox_input_ctx.cmd_queue != NULL) {
        xQueueOverwrite(g_xbox_input_ctx.cmd_queue, &cmd);
    }
}

static void xbox_input_task(void *arg) {
    (void)arg;

    while (true) {
        xbox_input_state_t snapshot;
        if (xSemaphoreTake(g_xbox_input_ctx.mutex, portMAX_DELAY) == pdTRUE) {
            snapshot = g_xbox_input_ctx.state;
            xSemaphoreGive(g_xbox_input_ctx.mutex);
        } else {
            memset(&snapshot, 0, sizeof(snapshot));
        }

        float vx = 0.0f;
        float vy = 0.0f;
        float wz = 0.0f;

        if (snapshot.connected) {
            const float left_x = apply_deadzone(normalize_axis(snapshot.joy_l_hori), g_xbox_input_ctx.cfg.deadzone);
            const float left_y = apply_deadzone(normalize_axis(snapshot.joy_l_vert), g_xbox_input_ctx.cfg.deadzone);
            const float right_x = apply_deadzone(normalize_axis(snapshot.joy_r_hori), g_xbox_input_ctx.cfg.deadzone);

            vx = -left_y * g_xbox_input_ctx.cfg.max_vx;
            vy = left_x * g_xbox_input_ctx.cfg.max_vy;
            wz = right_x * g_xbox_input_ctx.cfg.max_wz;
        }

        xbox_input_publish_cmd(vx, vy, wz, snapshot.connected);

        vTaskDelay(pdMS_TO_TICKS(g_xbox_input_ctx.cfg.task_period_ms));
    }
}

esp_err_t xbox_input_init(const xbox_input_config_t *cfg) {
    if (g_xbox_input_ctx.initialized) {
        return ESP_OK;
    }

    g_xbox_input_ctx.mutex = xSemaphoreCreateMutex();
    if (g_xbox_input_ctx.mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    g_xbox_input_ctx.cmd_queue = xQueueCreate(1, sizeof(xbox_input_cmd_t));
    if (g_xbox_input_ctx.cmd_queue == NULL) {
        vSemaphoreDelete(g_xbox_input_ctx.mutex);
        g_xbox_input_ctx.mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    if (cfg != NULL) {
        g_xbox_input_ctx.cfg = *cfg;
        if (g_xbox_input_ctx.cfg.deadzone < 0.0f) g_xbox_input_ctx.cfg.deadzone = 0.0f;
        if (g_xbox_input_ctx.cfg.deadzone > 0.6f) g_xbox_input_ctx.cfg.deadzone = 0.6f;
        if (g_xbox_input_ctx.cfg.task_period_ms == 0) g_xbox_input_ctx.cfg.task_period_ms = 20;
    }

    memset(&g_xbox_input_ctx.state, 0, sizeof(g_xbox_input_ctx.state));
    g_xbox_input_ctx.state.joy_l_hori = (uint16_t)(XBOX_JOY_MAX / 2.0f);
    g_xbox_input_ctx.state.joy_l_vert = (uint16_t)(XBOX_JOY_MAX / 2.0f);
    g_xbox_input_ctx.state.joy_r_hori = (uint16_t)(XBOX_JOY_MAX / 2.0f);
    g_xbox_input_ctx.state.joy_r_vert = (uint16_t)(XBOX_JOY_MAX / 2.0f);

    xbox_input_publish_cmd(0.0f, 0.0f, 0.0f, false);

    g_xbox_input_ctx.initialized = true;
    ESP_LOGI(TAG, "initialized (deadzone=%.2f max_vx=%.2f max_vy=%.2f max_wz=%.2f)",
             (double)g_xbox_input_ctx.cfg.deadzone,
             (double)g_xbox_input_ctx.cfg.max_vx,
             (double)g_xbox_input_ctx.cfg.max_vy,
             (double)g_xbox_input_ctx.cfg.max_wz);
    return ESP_OK;
}

esp_err_t xbox_input_start(void) {
    if (!g_xbox_input_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_xbox_input_ctx.task != NULL) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        xbox_input_task,
        "xbox_input_task",
        4096,
        NULL,
        5,
        &g_xbox_input_ctx.task,
        tskNO_AFFINITY);

    if (ok != pdPASS) {
        g_xbox_input_ctx.task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "task started");
    return ESP_OK;
}

void xbox_input_set_connected(bool connected) {
    if (!g_xbox_input_ctx.initialized) {
        return;
    }
    bool was_connected = false;
    if (xSemaphoreTake(g_xbox_input_ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        was_connected = g_xbox_input_ctx.state.connected;
        g_xbox_input_ctx.state.connected = connected;
        xSemaphoreGive(g_xbox_input_ctx.mutex);
    }
    if (!connected) {
        xbox_input_publish_cmd(0.0f, 0.0f, 0.0f, false);
    } else if (!was_connected && g_xbox_input_ctx.cfg.on_connected != NULL) {
        g_xbox_input_ctx.cfg.on_connected();
    }
}

uint8_t xbox_input_update_report(const uint8_t *data, size_t length) {
    if (!g_xbox_input_ctx.initialized) {
        return XBOX_INPUT_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_xbox_input_ctx.mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return XBOX_INPUT_ERR_TIMEOUT;
    }

    const uint8_t rc = xbox_input_parse_report(data, length, &g_xbox_input_ctx.state);
    xSemaphoreGive(g_xbox_input_ctx.mutex);
    return rc;
}

esp_err_t xbox_input_set_rumble(const xbox_input_rumble_t *rumble) {
    if (rumble == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t report[XBOX_INPUT_RUMBLE_REPORT_LEN] = {0};
    report[0] = (rumble->motor_center ? 0x01U : 0x00U) |
                (rumble->motor_shake ? 0x02U : 0x00U) |
                (rumble->motor_right ? 0x04U : 0x00U) |
                (rumble->motor_left ? 0x08U : 0x00U);
    report[1] = clamp_percent(rumble->power_left);
    report[2] = clamp_percent(rumble->power_right);
    report[3] = clamp_percent(rumble->power_shake);
    report[4] = clamp_percent(rumble->power_center);
    report[5] = rumble->time_active_10ms;
    report[6] = rumble->time_silent_10ms;
    report[7] = rumble->repeat_count;

    return xbox_input_ble_write_hid_report(report, sizeof(report));
}

esp_err_t xbox_input_stop_rumble(void) {
    const xbox_input_rumble_t off = {
        .motor_left = false,
        .motor_right = false,
        .motor_shake = false,
        .motor_center = false,
        .power_left = 0,
        .power_right = 0,
        .power_shake = 0,
        .power_center = 0,
        .time_active_10ms = 0,
        .time_silent_10ms = 0,
        .repeat_count = 0,
    };
    return xbox_input_set_rumble(&off);
}

void xbox_input_get_state(xbox_input_state_t *out_state) {
    if (out_state == NULL) {
        return;
    }

    memset(out_state, 0, sizeof(*out_state));
    if (!g_xbox_input_ctx.initialized) {
        return;
    }

    if (xSemaphoreTake(g_xbox_input_ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *out_state = g_xbox_input_ctx.state;
        xSemaphoreGive(g_xbox_input_ctx.mutex);
    }
}

QueueHandle_t xbox_input_get_cmd_queue(void) { return g_xbox_input_ctx.cmd_queue; }

bool xbox_input_try_get_latest_cmd(xbox_input_cmd_t *out_cmd) {
    if (out_cmd == NULL || !g_xbox_input_ctx.initialized) {
        return false;
    }
    if (xSemaphoreTake(g_xbox_input_ctx.mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }
    *out_cmd = g_xbox_input_ctx.latest_cmd;
    xSemaphoreGive(g_xbox_input_ctx.mutex);
    return true;
}
