#include "xbox_input_internal.h"

#include "esp_timer.h"

#define XBOX_IDX_BUTTONS_DIR 12
#define XBOX_IDX_BUTTONS_MAIN 13
#define XBOX_IDX_BUTTONS_CENTER 14
#define XBOX_IDX_BUTTONS_SHARE 15

static uint16_t read_u16_le(const uint8_t *data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

uint8_t xbox_input_parse_report(const uint8_t *data,
                                size_t length,
                                xbox_input_state_t *state) {
    if (data == NULL || state == NULL) {
        return XBOX_INPUT_ERR_INVALID_ARG;
    }
    if (length < XBOX_INPUT_REPORT_LEN) {
        return XBOX_INPUT_ERR_INVALID_LENGTH;
    }

    const uint8_t *payload = data;
    size_t payload_len = length;
    if (length == (XBOX_INPUT_REPORT_LEN + 1U)) {
        payload = data + 1;
        payload_len = XBOX_INPUT_REPORT_LEN;
    }
    if (payload_len != XBOX_INPUT_REPORT_LEN) {
        return XBOX_INPUT_ERR_INVALID_LENGTH;
    }

    uint8_t btn_bits = payload[XBOX_IDX_BUTTONS_MAIN];
    state->btn_a = (btn_bits & 0b00000001) != 0;
    state->btn_b = (btn_bits & 0b00000010) != 0;
    state->btn_x = (btn_bits & 0b00001000) != 0;
    state->btn_y = (btn_bits & 0b00010000) != 0;
    state->btn_lb = (btn_bits & 0b01000000) != 0;
    state->btn_rb = (btn_bits & 0b10000000) != 0;

    btn_bits = payload[XBOX_IDX_BUTTONS_CENTER];
    state->btn_select = (btn_bits & 0b00000100) != 0;
    state->btn_start = (btn_bits & 0b00001000) != 0;
    state->btn_xbox = (btn_bits & 0b00010000) != 0;
    state->btn_ls = (btn_bits & 0b00100000) != 0;
    state->btn_rs = (btn_bits & 0b01000000) != 0;

    btn_bits = payload[XBOX_IDX_BUTTONS_SHARE];
    state->btn_share = (btn_bits & 0b00000001) != 0;

    btn_bits = payload[XBOX_IDX_BUTTONS_DIR];
    state->btn_dir_up = (btn_bits == 1 || btn_bits == 2 || btn_bits == 8);
    state->btn_dir_right = (btn_bits >= 2 && btn_bits <= 4);
    state->btn_dir_down = (btn_bits >= 4 && btn_bits <= 6);
    state->btn_dir_left = (btn_bits >= 6 && btn_bits <= 8);

    state->joy_l_hori = read_u16_le(&payload[0]);
    state->joy_l_vert = read_u16_le(&payload[2]);
    state->joy_r_hori = read_u16_le(&payload[4]);
    state->joy_r_vert = read_u16_le(&payload[6]);
    state->trig_lt = read_u16_le(&payload[8]);
    state->trig_rt = read_u16_le(&payload[10]);
    state->updated_count++;
    state->last_update_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    state->connected = true;

    return XBOX_INPUT_ERR_NONE;
}
