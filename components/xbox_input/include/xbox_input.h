#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define XBOX_INPUT_REPORT_LEN 16

#define XBOX_INPUT_ERR_NONE 0
#define XBOX_INPUT_ERR_INVALID_LENGTH 1
#define XBOX_INPUT_ERR_INVALID_ARG 2
#define XBOX_INPUT_ERR_TIMEOUT 3
#define XBOX_INPUT_ERR_INVALID_STATE 4

typedef struct {
    bool btn_a;
    bool btn_b;
    bool btn_x;
    bool btn_y;
    bool btn_share;
    bool btn_start;
    bool btn_select;
    bool btn_xbox;
    bool btn_lb;
    bool btn_rb;
    bool btn_ls;
    bool btn_rs;
    bool btn_dir_up;
    bool btn_dir_left;
    bool btn_dir_right;
    bool btn_dir_down;
    uint16_t joy_l_hori;
    uint16_t joy_l_vert;
    uint16_t joy_r_hori;
    uint16_t joy_r_vert;
    uint16_t trig_lt;
    uint16_t trig_rt;
    uint32_t updated_count;
    uint32_t last_update_ms;
    bool connected;
} xbox_input_state_t;

typedef struct {
    float vx;
    float vy;
    float wz;
    bool connected;
    uint32_t updated_at_ms;
} xbox_input_cmd_t;

typedef struct {
    float deadzone;
    float max_vx;
    float max_vy;
    float max_wz;
    uint32_t task_period_ms;
    void (*on_connected)(void); /**< Called once when controller connects. May be NULL. */
} xbox_input_config_t;

typedef struct {
    bool motor_left;
    bool motor_right;
    bool motor_shake;
    bool motor_center;
    uint8_t power_left;
    uint8_t power_right;
    uint8_t power_shake;
    uint8_t power_center;
    uint8_t time_active_10ms;
    uint8_t time_silent_10ms;
    uint8_t repeat_count;
} xbox_input_rumble_t;

esp_err_t xbox_input_init(const xbox_input_config_t *cfg);
esp_err_t xbox_input_start(void);
esp_err_t xbox_input_start_ble(void);

void xbox_input_set_connected(bool connected);
uint8_t xbox_input_update_report(const uint8_t *data, size_t length);
esp_err_t xbox_input_set_rumble(const xbox_input_rumble_t *rumble);
esp_err_t xbox_input_stop_rumble(void);

void xbox_input_get_state(xbox_input_state_t *out_state);
QueueHandle_t xbox_input_get_cmd_queue(void);
bool xbox_input_try_get_latest_cmd(xbox_input_cmd_t *out_cmd);

#ifdef __cplusplus
}
#endif
