#include "st_servo.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static unsigned long st_servo_millis(void)
{
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

static void host_to_scs(const st_servo_t *servo, uint8_t *data_l, uint8_t *data_h, uint16_t data)
{
    if (servo->end) {
        *data_l = (uint8_t)(data >> 8);
        *data_h = (uint8_t)(data & 0xff);
    } else {
        *data_h = (uint8_t)(data >> 8);
        *data_l = (uint8_t)(data & 0xff);
    }
}

static uint16_t scs_to_host(const st_servo_t *servo, uint8_t data_l, uint8_t data_h)
{
    uint16_t data;
    if (servo->end) {
        data = data_l;
        data <<= 8;
        data |= data_h;
    } else {
        data = data_h;
        data <<= 8;
        data |= data_l;
    }
    return data;
}

static int write_scs(st_servo_t *servo, const uint8_t *data, int len)
{
    if (!servo || !servo->uart_attached || data == NULL || len <= 0) {
        return 0;
    }
    return uart_write_bytes(servo->uart_port, (const char *)data, len);
}

static int write_scs_byte(st_servo_t *servo, uint8_t data)
{
    return write_scs(servo, &data, 1);
}

static int read_scs(st_servo_t *servo, uint8_t *data, int len)
{
    if (!servo || !servo->uart_attached || len <= 0) {
        return 0;
    }

    int size = 0;
    unsigned long t_begin = st_servo_millis();

    while (1) {
        uint8_t byte = 0;
        int read_len = uart_read_bytes(servo->uart_port, &byte, 1, 1 / portTICK_PERIOD_MS);
        if (read_len == 1) {
            if (data) {
                data[size] = byte;
            }
            size++;
            t_begin = st_servo_millis();
        }

        if (size >= len) {
            break;
        }

        unsigned long t_user = st_servo_millis() - t_begin;
        if (t_user > servo->io_timeout_ms) {
            break;
        }
    }

    return size;
}

static void rflush_scs(st_servo_t *servo)
{
    if (!servo || !servo->uart_attached) {
        return;
    }
    uart_flush_input(servo->uart_port);
}

static void wflush_scs(st_servo_t *servo)
{
    if (!servo || !servo->uart_attached) {
        return;
    }
    uart_wait_tx_done(servo->uart_port, pdMS_TO_TICKS(servo->io_timeout_ms));
}

static int check_head(st_servo_t *servo)
{
    uint8_t b_dat;
    uint8_t b_buf[2] = {0, 0};
    uint8_t cnt = 0;

    while (1) {
        if (!read_scs(servo, &b_dat, 1)) {
            return 0;
        }
        b_buf[1] = b_buf[0];
        b_buf[0] = b_dat;
        if (b_buf[0] == 0xff && b_buf[1] == 0xff) {
            break;
        }
        cnt++;
        if (cnt > 10) {
            return 0;
        }
    }
    return 1;
}

static int ack(st_servo_t *servo, uint8_t id)
{
    servo->error = 0;
    if (id != 0xfe && servo->level) {
        uint8_t b_buf[4];
        if (!check_head(servo)) {
            return 0;
        }
        if (read_scs(servo, b_buf, 4) != 4) {
            return 0;
        }
        if (b_buf[0] != id) {
            return 0;
        }
        if (b_buf[1] != 2) {
            return 0;
        }
        uint8_t cal_sum = (uint8_t) ~(b_buf[0] + b_buf[1] + b_buf[2]);
        if (cal_sum != b_buf[3]) {
            return 0;
        }
        servo->error = b_buf[2];
    }
    return 1;
}

static void write_buf(st_servo_t *servo, uint8_t id, uint8_t mem_addr, const uint8_t *data, uint8_t len, uint8_t inst)
{
    uint8_t msg_len = 2;
    uint8_t b_buf[6];
    uint8_t checksum;

    b_buf[0] = 0xff;
    b_buf[1] = 0xff;
    b_buf[2] = id;
    b_buf[4] = inst;

    if (data) {
        msg_len += len + 1;
        b_buf[3] = msg_len;
        b_buf[5] = mem_addr;
        write_scs(servo, b_buf, 6);
    } else {
        b_buf[3] = msg_len;
        write_scs(servo, b_buf, 5);
    }

    checksum = (uint8_t)(id + msg_len + inst + mem_addr);

    if (data) {
        for (uint8_t i = 0; i < len; i++) {
            checksum += data[i];
        }
        write_scs(servo, data, len);
    }

    write_scs_byte(servo, (uint8_t) ~checksum);
}

static int gen_write(st_servo_t *servo, uint8_t id, uint8_t mem_addr, const uint8_t *data, uint8_t len)
{
    rflush_scs(servo);
    write_buf(servo, id, mem_addr, data, len, ST_SERVO_INST_WRITE);
    wflush_scs(servo);
    return ack(servo, id);
}

static int reg_write(st_servo_t *servo, uint8_t id, uint8_t mem_addr, const uint8_t *data, uint8_t len)
{
    rflush_scs(servo);
    write_buf(servo, id, mem_addr, data, len, ST_SERVO_INST_REG_WRITE);
    wflush_scs(servo);
    return ack(servo, id);
}

static void sync_write(st_servo_t *servo, const uint8_t *ids, uint8_t ids_count, uint8_t mem_addr, const uint8_t *data, uint8_t one_len)
{
    rflush_scs(servo);

    uint8_t mes_len = (uint8_t)((one_len + 1) * ids_count + 4);
    uint8_t sum;
    uint8_t b_buf[7] = {0xff, 0xff, 0xfe, mes_len, ST_SERVO_INST_SYNC_WRITE, mem_addr, one_len};
    write_scs(servo, b_buf, 7);

    sum = (uint8_t)(0xfe + mes_len + ST_SERVO_INST_SYNC_WRITE + mem_addr + one_len);
    for (uint8_t i = 0; i < ids_count; i++) {
        write_scs_byte(servo, ids[i]);
        write_scs(servo, data + i * one_len, one_len);
        sum += ids[i];
        for (uint8_t j = 0; j < one_len; j++) {
            sum += data[i * one_len + j];
        }
    }

    write_scs_byte(servo, (uint8_t) ~sum);
    wflush_scs(servo);
}

static int read_data(st_servo_t *servo, uint8_t id, uint8_t mem_addr, uint8_t *data, uint8_t len)
{
    rflush_scs(servo);
    write_buf(servo, id, mem_addr, &len, 1, ST_SERVO_INST_READ);
    wflush_scs(servo);

    if (!check_head(servo)) {
        return 0;
    }

    uint8_t b_buf[4];
    servo->error = 0;
    if (read_scs(servo, b_buf, 3) != 3) {
        return 0;
    }

    int size = read_scs(servo, data, len);
    if (size != len) {
        return 0;
    }

    if (read_scs(servo, b_buf + 3, 1) != 1) {
        return 0;
    }

    uint8_t cal_sum = (uint8_t)(b_buf[0] + b_buf[1] + b_buf[2]);
    for (int i = 0; i < size; i++) {
        cal_sum += data[i];
    }
    cal_sum = (uint8_t) ~cal_sum;

    if (cal_sum != b_buf[3]) {
        return 0;
    }

    servo->error = b_buf[2];
    return size;
}

static int read_byte(st_servo_t *servo, uint8_t id, uint8_t mem_addr)
{
    uint8_t data = 0;
    int size = read_data(servo, id, mem_addr, &data, 1);
    if (size != 1) {
        return -1;
    }
    return data;
}

static int read_word(st_servo_t *servo, uint8_t id, uint8_t mem_addr)
{
    uint8_t data[2] = {0};
    int size = read_data(servo, id, mem_addr, data, 2);
    if (size != 2) {
        return -1;
    }
    return (int)scs_to_host(servo, data[0], data[1]);
}

void st_servo_init(st_servo_t *servo, uint8_t end, uint8_t level)
{
    if (!servo) {
        return;
    }

    servo->uart_port = UART_NUM_MAX;
    servo->uart_attached = false;
    servo->owns_driver = false;
    servo->io_timeout_ms = 100;
    servo->level = level;
    servo->end = end;
    servo->error = 0;
    servo->err = 0;
    servo->sync_read_rx_packet_index = 0;
    servo->sync_read_rx_packet_len = 0;
    servo->sync_read_rx_packet = NULL;
}

esp_err_t st_servo_attach_uart(st_servo_t *servo,
                               uart_port_t uart_port,
                               int baud_rate,
                               gpio_num_t tx_pin,
                               gpio_num_t rx_pin,
                               int rx_buffer_size,
                               int tx_buffer_size)
{
    if (!servo || uart_port < UART_NUM_0 || uart_port >= UART_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(uart_port, &uart_config);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_driver_install(uart_port, rx_buffer_size, tx_buffer_size, 0, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }

    servo->uart_port = uart_port;
    servo->uart_attached = true;
    servo->owns_driver = true;
    servo->err = 0;
    return ESP_OK;
}

void st_servo_detach_uart(st_servo_t *servo)
{
    if (!servo) {
        return;
    }

    if (servo->uart_attached && servo->owns_driver && servo->uart_port < UART_NUM_MAX) {
        uart_driver_delete(servo->uart_port);
    }

    servo->uart_port = UART_NUM_MAX;
    servo->uart_attached = false;
    servo->owns_driver = false;
}

int st_servo_ping(st_servo_t *servo, uint8_t id)
{
    if (!servo) {
        return -1;
    }

    rflush_scs(servo);
    write_buf(servo, id, 0, NULL, 0, ST_SERVO_INST_PING);
    wflush_scs(servo);
    servo->error = 0;

    if (!check_head(servo)) {
        return -1;
    }

    uint8_t b_buf[4];
    if (read_scs(servo, b_buf, 4) != 4) {
        return -1;
    }

    if (b_buf[0] != id && id != 0xfe) {
        return -1;
    }
    if (b_buf[1] != 2) {
        return -1;
    }

    uint8_t cal_sum = (uint8_t) ~(b_buf[0] + b_buf[1] + b_buf[2]);
    if (cal_sum != b_buf[3]) {
        return -1;
    }

    servo->error = b_buf[2];
    return b_buf[0];
}

int st_servo_reg_write_action(st_servo_t *servo, uint8_t id)
{
    if (!servo) {
        return 0;
    }

    rflush_scs(servo);
    write_buf(servo, id, 0, NULL, 0, ST_SERVO_INST_REG_ACTION);
    wflush_scs(servo);
    return ack(servo, id);
}

int st_servo_write_pos_ex(st_servo_t *servo, uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
{
    if (!servo) {
        return 0;
    }

    uint16_t pos_u16 = (uint16_t)position;
    if (position < 0) {
        pos_u16 = (uint16_t)(-position);
        pos_u16 |= (1u << 15);
    }

    uint8_t buf[7];
    buf[0] = acc;
    host_to_scs(servo, buf + 1, buf + 2, pos_u16);
    host_to_scs(servo, buf + 3, buf + 4, 0);
    host_to_scs(servo, buf + 5, buf + 6, speed);

    return gen_write(servo, id, STS_ACC, buf, 7);
}

int st_servo_reg_write_pos_ex(st_servo_t *servo, uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
{
    if (!servo) {
        return 0;
    }

    uint16_t pos_u16 = (uint16_t)position;
    if (position < 0) {
        pos_u16 = (uint16_t)(-position);
        pos_u16 |= (1u << 15);
    }

    uint8_t buf[7];
    buf[0] = acc;
    host_to_scs(servo, buf + 1, buf + 2, pos_u16);
    host_to_scs(servo, buf + 3, buf + 4, 0);
    host_to_scs(servo, buf + 5, buf + 6, speed);

    return reg_write(servo, id, STS_ACC, buf, 7);
}

int st_servo_sync_write_pos_ex(st_servo_t *servo,
                               const uint8_t *ids,
                               uint8_t ids_count,
                               const int16_t *positions,
                               const uint16_t *speeds,
                               const uint8_t *accelerations)
{
    if (!servo || !ids || !positions || ids_count == 0) {
        return 0;
    }

    uint8_t offbuf[7 * 32];
    if (ids_count > 32) {
        return 0;
    }

    for (uint8_t i = 0; i < ids_count; i++) {
        uint16_t pos_u16 = (uint16_t)positions[i];
        if (positions[i] < 0) {
            pos_u16 = (uint16_t)(-positions[i]);
            pos_u16 |= (1u << 15);
        }

        uint16_t speed = speeds ? speeds[i] : 0;
        uint8_t acc = accelerations ? accelerations[i] : 0;

        offbuf[i * 7] = acc;
        host_to_scs(servo, offbuf + i * 7 + 1, offbuf + i * 7 + 2, pos_u16);
        host_to_scs(servo, offbuf + i * 7 + 3, offbuf + i * 7 + 4, 0);
        host_to_scs(servo, offbuf + i * 7 + 5, offbuf + i * 7 + 6, speed);
    }

    sync_write(servo, ids, ids_count, STS_ACC, offbuf, 7);
    return 1;
}

int st_servo_sync_read_pos_ex(st_servo_t *servo, const uint8_t *ids, uint8_t ids_count, int *positions_out)
{
    if (!servo || !ids || !positions_out || ids_count == 0 || ids_count > 32) {
        return 0;
    }

    /* Packet: 0xFF 0xFF 0xFE PKT_LEN SYNC_READ DATA_LEN MEM_ADDR ID1...IDn CHECKSUM
     * PKT_LEN = ids_count + 4  (inst + data_len + mem_addr + ids_count ids + checksum)
     */
    const uint8_t data_len = 2; /* 2 bytes per position */
    const uint8_t pkt_len  = (uint8_t)(ids_count + 4);

    uint8_t header[7] = {
        0xFF, 0xFF, 0xFE, pkt_len,
        ST_SERVO_INST_SYNC_READ, data_len, STS_PRESENT_POSITION_L
    };

    uint8_t sum = (uint8_t)(0xFE + pkt_len + ST_SERVO_INST_SYNC_READ + data_len + STS_PRESENT_POSITION_L);
    for (uint8_t i = 0; i < ids_count; i++) {
        sum += ids[i];
    }

    rflush_scs(servo);
    write_scs(servo, header, 7);
    write_scs(servo, ids, ids_count);
    write_scs_byte(servo, (uint8_t)~sum);
    wflush_scs(servo);

    /* Collect one response packet per servo (in ID list order) */
    int read_count = 0;
    for (uint8_t i = 0; i < ids_count; i++) {
        positions_out[i] = -1; /* default: error */

        if (!check_head(servo)) {
            continue;
        }

        /* Read: servo_id, resp_len, error (3 bytes) */
        uint8_t b_resp[3];
        if (read_scs(servo, b_resp, 3) != 3) {
            continue;
        }

        /* Read position data (2 bytes) */
        uint8_t pos_bytes[2] = {0, 0};
        if (read_scs(servo, pos_bytes, data_len) != data_len) {
            continue;
        }

        /* Read and verify checksum */
        uint8_t chk_recv;
        if (read_scs(servo, &chk_recv, 1) != 1) {
            continue;
        }

        uint8_t cal_chk = (uint8_t)(b_resp[0] + b_resp[1] + b_resp[2] + pos_bytes[0] + pos_bytes[1]);
        if ((uint8_t)~cal_chk != chk_recv) {
            continue;
        }

        /* Decode signed 15-bit position (same as st_servo_read_pos) */
        int pos = (int)scs_to_host(servo, pos_bytes[0], pos_bytes[1]);
        if (pos & (1 << 15)) {
            pos = -(pos & ~(1 << 15));
        }
        positions_out[i] = pos;
        read_count++;
    }

    return read_count;
}

int st_servo_enable_torque(st_servo_t *servo, uint8_t id, uint8_t enable)
{
    return gen_write(servo, id, STS_TORQUE_ENABLE, &enable, 1);
}

int st_servo_unlock_eprom(st_servo_t *servo, uint8_t id)
{
    uint8_t unlock = 0;
    return gen_write(servo, id, STS_LOCK, &unlock, 1);
}

int st_servo_lock_eprom(st_servo_t *servo, uint8_t id)
{
    uint8_t lock = 1;
    return gen_write(servo, id, STS_LOCK, &lock, 1);
}

int st_servo_calibration_offset(st_servo_t *servo, uint8_t id)
{
    uint8_t calibration_cmd = 128;
    return gen_write(servo, id, STS_TORQUE_ENABLE, &calibration_cmd, 1);
}

int st_servo_change_id(st_servo_t *servo, uint8_t current_id, uint8_t new_id)
{
    if (!servo || new_id == 0 || new_id == 0xfe) {
        return 0;
    }

    if (!st_servo_unlock_eprom(servo, current_id)) {
        return 0;
    }

    if (!gen_write(servo, current_id, STS_ID, &new_id, 1)) {
        return 0;
    }

    return st_servo_lock_eprom(servo, new_id);
}

int st_servo_read_pos(st_servo_t *servo, int id)
{
    if (!servo || id < 0) {
        return -1;
    }

    servo->err = 0;
    int pos = read_word(servo, (uint8_t)id, STS_PRESENT_POSITION_L);
    if (pos == -1) {
        servo->err = 1;
    }
    if (!servo->err && (pos & (1 << 15))) {
        pos = -(pos & ~(1 << 15));
    }
    return pos;
}

int st_servo_read_speed(st_servo_t *servo, int id)
{
    if (!servo || id < 0) {
        return -1;
    }

    servo->err = 0;
    int speed = read_word(servo, (uint8_t)id, STS_PRESENT_SPEED_L);
    if (speed == -1) {
        servo->err = 1;
        return -1;
    }
    if (!servo->err && (speed & (1 << 15))) {
        speed = -(speed & ~(1 << 15));
    }
    return speed;
}

int st_servo_read_load(st_servo_t *servo, int id)
{
    if (!servo || id < 0) {
        return -1;
    }

    servo->err = 0;
    int load = read_word(servo, (uint8_t)id, STS_PRESENT_LOAD_L);
    if (load == -1) {
        servo->err = 1;
        return -1;
    }
    if (!servo->err && (load & (1 << 10))) {
        load = -(load & ~(1 << 10));
    }
    return load;
}

int st_servo_read_voltage(st_servo_t *servo, int id)
{
    if (!servo || id < 0) {
        return -1;
    }

    servo->err = 0;
    int v = read_byte(servo, (uint8_t)id, STS_PRESENT_VOLTAGE);
    if (v == -1) {
        servo->err = 1;
    }
    return v;
}

int st_servo_read_temperature(st_servo_t *servo, int id)
{
    if (!servo || id < 0) {
        return -1;
    }

    servo->err = 0;
    int t = read_byte(servo, (uint8_t)id, STS_PRESENT_TEMPERATURE);
    if (t == -1) {
        servo->err = 1;
    }
    return t;
}

int st_servo_read_move(st_servo_t *servo, int id)
{
    if (!servo || id < 0) {
        return -1;
    }

    servo->err = 0;
    int m = read_byte(servo, (uint8_t)id, STS_MOVING);
    if (m == -1) {
        servo->err = 1;
    }
    return m;
}

int st_servo_read_current(st_servo_t *servo, int id)
{
    if (!servo || id < 0) {
        return -1;
    }

    servo->err = 0;
    int current = read_word(servo, (uint8_t)id, STS_PRESENT_CURRENT_L);
    if (current == -1) {
        servo->err = 1;
        return -1;
    }
    if (!servo->err && (current & (1 << 15))) {
        current = -(current & ~(1 << 15));
    }
    return current;
}

int st_servo_read_mode(st_servo_t *servo, int id)
{
    if (!servo || id < 0) {
        return -1;
    }

    servo->err = 0;
    int mode = read_byte(servo, (uint8_t)id, STS_MODE);
    if (mode == -1) {
        servo->err = 1;
    }
    return mode;
}
