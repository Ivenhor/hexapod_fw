#ifndef ST_SERVO_H
#define ST_SERVO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Driver context for one ST servo UART bus.
 */
typedef struct {
    uart_port_t uart_port;
    bool uart_attached;
    bool owns_driver;
    uint32_t io_timeout_ms;
    uint8_t level;
    uint8_t end;
    uint8_t error;
    int err;
    uint8_t sync_read_rx_packet_index;
    uint8_t sync_read_rx_packet_len;
    uint8_t *sync_read_rx_packet;
} st_servo_t;

enum {
    ST_SERVO_INST_PING = 0x01,
    ST_SERVO_INST_READ = 0x02,
    ST_SERVO_INST_WRITE = 0x03,
    ST_SERVO_INST_REG_WRITE = 0x04,
    ST_SERVO_INST_REG_ACTION = 0x05,
    ST_SERVO_INST_SYNC_READ = 0x82,
    ST_SERVO_INST_SYNC_WRITE = 0x83,
};

enum {
    STS_ID = 5,
    STS_ACC = 41,
    STS_GOAL_POSITION_L = 42,
    STS_GOAL_SPEED_L = 46,
    STS_TORQUE_ENABLE = 40,
    STS_LOCK = 55,
    STS_PRESENT_POSITION_L = 56,
    STS_PRESENT_SPEED_L = 58,
    STS_PRESENT_LOAD_L = 60,
    STS_PRESENT_VOLTAGE = 62,
    STS_PRESENT_TEMPERATURE = 63,
    STS_MOVING = 66,
    STS_PRESENT_CURRENT_L = 69,
    STS_MODE = 33,
};

/**
 * @brief Initialize servo driver context.
 *
 * @param servo Pointer to context.
 * @param end Endianness format used by servo protocol (0 for STS/SMS default).
 * @param level Response level from servo (usually 1).
 */
void st_servo_init(st_servo_t *servo, uint8_t end, uint8_t level);

/**
 * @brief Attach ESP-IDF UART driver to servo context.
 *
 * @param servo Pointer to context.
 * @param uart_port UART peripheral.
 * @param baud_rate Bus baudrate.
 * @param tx_pin UART TX pin.
 * @param rx_pin UART RX pin.
 * @param rx_buffer_size UART RX buffer size in bytes.
 * @param tx_buffer_size UART TX buffer size in bytes.
 *
 * @return ESP_OK on success, otherwise ESP-IDF error code.
 */
esp_err_t st_servo_attach_uart(st_servo_t *servo,
                               uart_port_t uart_port,
                               int baud_rate,
                               gpio_num_t tx_pin,
                               gpio_num_t rx_pin,
                               int rx_buffer_size,
                               int tx_buffer_size);

/**
 * @brief Detach UART driver from servo context.
 *
 * @param servo Pointer to context.
 */
void st_servo_detach_uart(st_servo_t *servo);

/**
 * @brief Ping servo by ID.
 * @return Servo ID on success, -1 on failure.
 */
int st_servo_ping(st_servo_t *servo, uint8_t id);

/**
 * @brief Write target position with speed/acceleration.
 * @return 1 on success, 0 on failure.
 */
int st_servo_write_pos_ex(st_servo_t *servo, uint8_t id, int16_t position, uint16_t speed, uint8_t acc);

/**
 * @brief Queue target position command (requires @ref st_servo_reg_write_action).
 * @return 1 on success, 0 on failure.
 */
int st_servo_reg_write_pos_ex(st_servo_t *servo, uint8_t id, int16_t position, uint16_t speed, uint8_t acc);

/**
 * @brief Synchronously write target positions to multiple servos.
 *
 * @param ids Servo ID array.
 * @param ids_count Number of servo IDs.
 * @param positions Position array.
 * @param speeds Optional speed array, can be NULL.
 * @param accelerations Optional acceleration array, can be NULL.
 *
 * @return 1 on success, 0 on failure.
 */
int st_servo_sync_write_pos_ex(st_servo_t *servo,
                               const uint8_t *ids,
                               uint8_t ids_count,
                               const int16_t *positions,
                               const uint16_t *speeds,
                               const uint8_t *accelerations);

/**
 * @brief Synchronously read current positions from multiple servos.
 *
 * Sends a single SYNC_READ broadcast and collects individual response packets
 * from each servo (servos reply in the order they appear in @p ids).
 *
 * @param ids           Servo ID array.
 * @param ids_count     Number of servo IDs (max 32).
 * @param positions_out Output array (must have at least @p ids_count elements).
 *                      Each element is set to the signed raw position value
 *                      (same encoding as @ref st_servo_read_pos), or -1 if the
 *                      servo did not respond or checksum failed.
 *
 * @return Number of positions successfully read.
 */
int st_servo_sync_read_pos_ex(st_servo_t *servo,
                              const uint8_t *ids,
                              uint8_t ids_count,
                              int *positions_out);

/**
 * @brief Execute queued register-write command.
 * @return 1 on success, 0 on failure.
 */
int st_servo_reg_write_action(st_servo_t *servo, uint8_t id);

/**
 * @brief Enable or disable servo torque.
 * @return 1 on success, 0 on failure.
 */
int st_servo_enable_torque(st_servo_t *servo, uint8_t id, uint8_t enable);

/**
 * @brief Unlock servo EPROM for write operations.
 * @return 1 on success, 0 on failure.
 */
int st_servo_unlock_eprom(st_servo_t *servo, uint8_t id);

/**
 * @brief Lock servo EPROM after write operations.
 * @return 1 on success, 0 on failure.
 */
int st_servo_lock_eprom(st_servo_t *servo, uint8_t id);

/**
 * @brief Calibrate servo zero point (middle-position offset calibration).
 *
 * This maps to the original STS command writing value `128` to torque-enable register.
 *
 * @return 1 on success, 0 on failure.
 */
int st_servo_calibration_offset(st_servo_t *servo, uint8_t id);

/**
 * @brief Change servo ID.
 *
 * Function performs EPROM unlock -> ID write -> EPROM lock.
 *
 * @param current_id Existing servo ID.
 * @param new_id New servo ID (1..253, not 0xFE).
 *
 * @return 1 on success, 0 on failure.
 */
int st_servo_change_id(st_servo_t *servo, uint8_t current_id, uint8_t new_id);

/**
 * @brief Read current position.
 * @return Position value, or -1 on failure.
 */
int st_servo_read_pos(st_servo_t *servo, int id);

/**
 * @brief Read current speed.
 * @return Speed value, or -1 on failure.
 */
int st_servo_read_speed(st_servo_t *servo, int id);

/**
 * @brief Read current load.
 * @return Load value, or -1 on failure.
 */
int st_servo_read_load(st_servo_t *servo, int id);

/**
 * @brief Read bus voltage.
 * @return Voltage value (0.1V units depending on model), or -1 on failure.
 */
int st_servo_read_voltage(st_servo_t *servo, int id);

/**
 * @brief Read temperature.
 * @return Temperature value, or -1 on failure.
 */
int st_servo_read_temperature(st_servo_t *servo, int id);

/**
 * @brief Read movement state.
 * @return Move flag, or -1 on failure.
 */
int st_servo_read_move(st_servo_t *servo, int id);

/**
 * @brief Read current consumption.
 * @return Current value, or -1 on failure.
 */
int st_servo_read_current(st_servo_t *servo, int id);

/**
 * @brief Read current servo mode.
 * @return Mode value, or -1 on failure.
 */
int st_servo_read_mode(st_servo_t *servo, int id);

#ifdef __cplusplus
}
#endif

#endif
