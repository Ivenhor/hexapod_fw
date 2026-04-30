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

#define XBOX_INPUT_ERR_INVALID_LENGTH 1

typedef struct {
    bool btnA;
    bool btnB;
    bool btnX;
    bool btnY;
    bool btnShare;
    bool btnStart;
    bool btnSelect;
    bool btnXbox;
    bool btnLB;
    bool btnRB;
    bool btnLS;
    bool btnRS;
    bool btnDirUp;
    bool btnDirLeft;
    bool btnDirRight;
    bool btnDirDown;
    uint16_t joyLHori;
    uint16_t joyLVert;
    uint16_t joyRHori;
    uint16_t joyRVert;
    uint16_t trigLT;
    uint16_t trigRT;
    uint32_t updatedCount;
    uint32_t lastUpdateMs;
    bool connected;
} xbox_input_state_t;

typedef struct {
    float vx;
    float vy;
    float wz;
    bool connected;
    uint32_t updatedAtMs;
} xbox_input_cmd_t;

typedef struct {
    float deadzone;
    float maxVx;
    float maxVy;
    float maxWz;
    uint32_t taskPeriodMs;
} xbox_input_config_t;

esp_err_t xbox_input_init(const xbox_input_config_t *cfg);
esp_err_t xbox_input_start(void);
esp_err_t xbox_input_start_ble(void);

void xbox_input_set_connected(bool connected);
uint8_t xbox_input_update_report(const uint8_t *data, size_t length);

void xbox_input_get_state(xbox_input_state_t *outState);
QueueHandle_t xbox_input_get_cmd_queue(void);
bool xbox_input_try_get_latest_cmd(xbox_input_cmd_t *outCmd);

#ifdef __cplusplus
}
#endif
