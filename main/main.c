/**
 * main.c — Pure-C hexapod runtime for ESP-IDF.
 *
 * Architecture mirrors the C++ implementation:
 *   - Xbox controller -> motion command (velocity, angular rate)
 *   - Exponential curve applied to stick inputs
 *   - hc_controller_t runs tripod gait with foothold planner + leg controller
 *   - Servo frame written via st_servo_sync_write_pos_ex each tick
 *   - Closed-loop correction: servo positions read back every tick and
 *     injected into the controller as the current smoothed pose
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "st_servo.h"
#include "xbox_input.h"

#include "hexapod_controller_c.h"

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

static const char *kTag = "HexapodC";

static const uart_port_t kServoUart  = UART_NUM_1;
static const int         kServoBaud  = 1000000;
static const gpio_num_t  kServoTxPin = GPIO_NUM_19;
static const gpio_num_t  kServoRxPin = GPIO_NUM_18;
static const int         kRxBufSize  = 2048;
static const int         kTxBufSize  = 1024;

static const double kControlDt          = 0.01;
static const double kDefaultGaitHz      = 1.8;
static const double kNoInputTimeoutMs   = 15000.0;
static const double kStrideScale        = 0.2;
static const double kStepHeightScale    = 1.0;
static const double kMotionFilterTauSec = 0.12;
static const double kExpoVxy            = 0.70;
static const double kExpoWz             = 0.55;
static const uint16_t kServoSpeedMax    = 3700;
/* UART read timeout per servo. At 1 Mbaud a 7-byte packet takes ~0.07 ms;
 * 12 ms leaves ample margin while limiting worst-case hang to ~216 ms. */
static const uint32_t kServoReadTimeoutMs = 12;
/* After this many consecutive read failures, log a warning. */
static const int kMaxConsecReadFails = 5;
/* After gait stops, keep torque on this long so the robot settles into
 * stand pose before servos go limp. */
static const uint64_t kTorqueCutoffDelayMs = 2000;

static const uint8_t kServoIds[HC_SERVO_COUNT] = {
     1,  2,  3,
     4,  5,  6,
     7,  8,  9,
    10, 11, 12,
    13, 14, 15,
    16, 17, 18,
};

static const int8_t kServoDir[HC_SERVO_COUNT] = {
    -1,  1, -1,
    -1,  1, -1,
    -1,  1, -1,
    -1,  1, -1,
    -1,  1, -1,
    -1,  1, -1,
};

/* ------------------------------------------------------------------ */
/* Exponential input curve                                             */
/* ------------------------------------------------------------------ */

static double apply_expo(double value, double expo) {
    if (value >  1.0) value =  1.0;
    if (value < -1.0) value = -1.0;
    const double abs_val = (value < 0.0) ? -value : value;
    const double e = (expo > 1.0) ? 1.0 : ((expo < 0.0) ? 0.0 : expo);
    const double result = (1.0 - e) * abs_val + e * abs_val * abs_val * abs_val;
    return (value < 0.0) ? -result : result;
}

/* ------------------------------------------------------------------ */
/* Servo direction helpers                                             */
/* ------------------------------------------------------------------ */

static int16_t apply_dir(int16_t raw, int8_t dir) {
    if (dir >= 0) return raw;
    int32_t m = 4094 - (int32_t)raw;
    if (m < 0)    m = 0;
    if (m > 4095) m = 4095;
    return (int16_t)m;
}

/* Mirroring is its own inverse. */
static int16_t undo_dir(int16_t raw, int8_t dir) {
    return apply_dir(raw, dir);
}

/* ------------------------------------------------------------------ */
/* Closed-loop servo readback                                          */
/* ------------------------------------------------------------------ */

static bool read_joint_angles(st_servo_t *bus,
                               hc_joint_angles_t out[HC_LEG_COUNT]) {
    int raw[HC_SERVO_COUNT];
    const int count = st_servo_sync_read_pos_ex(bus, kServoIds,
                                                 HC_SERVO_COUNT, raw);
    if (count < HC_SERVO_COUNT) {
        return false;
    }

    for (size_t leg = 0; leg < HC_LEG_COUNT; ++leg) {
        const size_t base = leg * HC_JOINTS_PER_LEG;
        out[leg].coxa  = hc_raw_to_rad(undo_dir((int16_t)raw[base + 0], kServoDir[base + 0]));
        out[leg].femur = hc_raw_to_rad(undo_dir((int16_t)raw[base + 1], kServoDir[base + 1]));
        out[leg].tibia = hc_raw_to_rad(undo_dir((int16_t)raw[base + 2], kServoDir[base + 2]));
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* Write servo frame to hardware                                       */
/* ------------------------------------------------------------------ */

static void write_servo_frame(st_servo_t *bus,
                               const hc_servo_frame_t *frame,
                               uint16_t speed_cmd) {
    int16_t  pos[HC_SERVO_COUNT];
    uint16_t spd[HC_SERVO_COUNT];
    uint8_t  acc[HC_SERVO_COUNT];

    for (size_t i = 0; i < HC_SERVO_COUNT; ++i) {
        pos[i] = apply_dir(frame->position_raw[i], kServoDir[i]);
        spd[i] = speed_cmd;
        acc[i] = frame->acc_raw[i];
    }

    st_servo_sync_write_pos_ex(bus, kServoIds, HC_SERVO_COUNT,
                                pos, spd, acc);
}

/* ------------------------------------------------------------------ */
/* Motion state helpers                                                */
/* ------------------------------------------------------------------ */

typedef struct { double vx, vy, omega_z; } motion_cmd_t;

static bool has_motion(const motion_cmd_t *m, double eps) {
    return (m->vx > eps || m->vx < -eps ||
            m->vy > eps || m->vy < -eps ||
            m->omega_z > eps || m->omega_z < -eps);
}

static double clamp01(double v) {
    if (v < 0.0) return 0.0;
    if (v > 1.0) return 1.0;
    return v;
}

/* Max absolute stick deflection on any axis drives speed to kServoSpeedMax. */
static uint16_t speed_from_stick(const hc_servo_frame_t *frame,
                                  const xbox_input_cmd_t *cmd) {
    double stick = fabs((double)cmd->vx);
    const double ay = fabs((double)cmd->vy);
    const double az = fabs((double)cmd->wz);
    if (ay > stick) stick = ay;
    if (az > stick) stick = az;
    stick = clamp01(stick);

    const uint16_t base = frame->speed_raw[0];
    const double blended = (double)base + ((double)kServoSpeedMax - (double)base) * stick;
    if (blended < 0.0) return 0;
    if (blended > 65535.0) return 65535;
    return (uint16_t)(blended + 0.5);
}

/* ------------------------------------------------------------------ */
/* Torque helpers                                                      */
/* ------------------------------------------------------------------ */

static void set_all_torque(st_servo_t *bus, uint8_t enable) {
    for (size_t i = 0; i < HC_SERVO_COUNT; ++i) {
        st_servo_enable_torque(bus, kServoIds[i], enable);
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

static void on_xbox_connected(void) {
    xbox_input_rumble_t pulse = {
        .motor_center    = true,
        .power_center    = 90,
        .time_active_10ms = 30, /* 120 ms */
    };
    xbox_input_set_rumble(&pulse);
}

void app_main(void) {
    /* Xbox controller */
    xbox_input_config_t input_cfg = {
        .deadzone       = 0.10f,
        .max_vx         = 1.0f,
        .max_vy         = 1.0f,
        .max_wz         = 0.8f,
        .task_period_ms = (uint32_t)(kControlDt * 1000.0),
        .on_connected   = on_xbox_connected,
    };
    if (xbox_input_init(&input_cfg) != ESP_OK ||
        xbox_input_start()          != ESP_OK ||
        xbox_input_start_ble()      != ESP_OK) {
        ESP_LOGE(kTag, "xbox init failed");
        return;
    }

    /* Servo bus */
    st_servo_t servo_bus;
    st_servo_init(&servo_bus, 0, 1);
    if (st_servo_attach_uart(&servo_bus, kServoUart, kServoBaud,
                              kServoTxPin, kServoRxPin,
                              kRxBufSize, kTxBufSize) != ESP_OK) {
        ESP_LOGE(kTag, "servo UART attach failed");
        return;
    }
    /* Tighten per-servo read timeout: at 1 Mbaud a response arrives in
     * ~0.1 ms; 12 ms is ample.  Default 100 ms × 18 servos = 1.8 s stall
     * when servos stop responding, which causes the observed hangs. */
    servo_bus.io_timeout_ms = kServoReadTimeoutMs;

    /* Hexapod controller (static: avoids large stack frame) */
    static hc_controller_t ctrl;
    hc_controller_init(&ctrl);
    hc_controller_set_stride_scale(&ctrl, kStrideScale);
    hc_controller_set_step_height_scale(&ctrl, kStepHeightScale);

    /* Runtime state */
    motion_cmd_t filtered      = {0.0, 0.0, 0.0};
    bool filter_init            = false;
    bool gait_active            = false;
    bool output_ready           = false;
    bool connected              = false;
    bool torque_enabled         = false;
    int  consec_read_fails      = 0;
    uint64_t last_motion_ms     = 0;
    /* When non-zero: timestamp after which torque should be cut. */
    uint64_t torque_cutoff_at_ms = 0;

    ESP_LOGI(kTag, "Pure-C runtime started -- gait + closed-loop feedback");

    for (;;) {
        const uint64_t now_ms =
            (uint64_t)(esp_timer_get_time() / 1000LL);

        /* Read input */
        xbox_input_cmd_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        const bool has_cmd = xbox_input_try_get_latest_cmd(&cmd);

        if (!has_cmd || !cmd.connected) {
            if (connected) {
                ESP_LOGI(kTag, "controller disconnected -- standing");
                connected    = false;
                gait_active  = false;
                filter_init  = false;
                output_ready = false;
                hc_controller_disable_gait(&ctrl);
                if (torque_enabled) {
                    set_all_torque(&servo_bus, 0);
                    torque_enabled     = false;
                    consec_read_fails  = 0;
                    torque_cutoff_at_ms = 0;
                }
            }
            vTaskDelay(pdMS_TO_TICKS((int)(kControlDt * 1000.0)));
            continue;
        }

        if (!connected) {
            ESP_LOGI(kTag, "controller connected");
            connected      = true;
            last_motion_ms = now_ms;
            set_all_torque(&servo_bus, 1);
            torque_enabled    = true;
            consec_read_fails = 0;
        }

        /* Expo curve */
        const double raw_vx = apply_expo((double)cmd.vx, kExpoVxy);
        const double raw_vy = apply_expo((double)cmd.vy, kExpoVxy);
        const double raw_wz = apply_expo((double)cmd.wz, kExpoWz);

        /* Low-pass motion filter */
        if (!filter_init) {
            filtered.vx      = raw_vx;
            filtered.vy      = raw_vy;
            filtered.omega_z = raw_wz;
            filter_init = true;
        } else {
            const double alpha =
                kControlDt / (kMotionFilterTauSec + kControlDt);
            filtered.vx      += alpha * (raw_vx - filtered.vx);
            filtered.vy      += alpha * (raw_vy - filtered.vy);
            filtered.omega_z += alpha * (raw_wz - filtered.omega_z);
        }

        /* Feed velocity to controller */
        hc_controller_set_velocity(&ctrl,
                                    filtered.vx,
                                    filtered.vy,
                                    filtered.omega_z);

        /* Gait / stand state machine */
        if (has_motion(&filtered, 0.01)) {
            last_motion_ms = now_ms;
            if (!gait_active) {
                if (!torque_enabled) {
                    set_all_torque(&servo_bus, 1);
                    torque_enabled    = true;
                    consec_read_fails = 0;
                }
                torque_cutoff_at_ms = 0;
                ESP_LOGI(kTag, "starting gait %.1f Hz", kDefaultGaitHz);
                hc_controller_enable_gait(&ctrl, kDefaultGaitHz);
                gait_active  = true;
                output_ready = false;
            }
        } else {
            const uint64_t idle_ms = now_ms - last_motion_ms;
            if (gait_active &&
                idle_ms >= (uint64_t)kNoInputTimeoutMs) {
                ESP_LOGI(kTag, "idle timeout -- standing, torque off in %llu ms",
                         (unsigned long long)kTorqueCutoffDelayMs);
                hc_controller_disable_gait(&ctrl);
                gait_active          = false;
                torque_cutoff_at_ms  = now_ms + kTorqueCutoffDelayMs;
            }
            /* Settling: gait stopped, waiting before cutting torque. */
            if (!gait_active && torque_enabled && torque_cutoff_at_ms &&
                now_ms >= torque_cutoff_at_ms) {
                ESP_LOGI(kTag, "settled -- torque off");
                set_all_torque(&servo_bus, 0);
                torque_enabled       = false;
                consec_read_fails    = 0;
                torque_cutoff_at_ms  = 0;
            }
        }

        /* Compute servo frame */
        const hc_servo_frame_t frame =
            hc_controller_update(&ctrl, kControlDt);
        const uint16_t speed_cmd = speed_from_stick(&frame, &cmd);

        /* Prime servos on first connected tick, then write every tick */
        if (torque_enabled) {
            if (!output_ready) {
                write_servo_frame(&servo_bus, &frame, speed_cmd);
                output_ready = true;
            }
            write_servo_frame(&servo_bus, &frame, speed_cmd);
        }

        /* Closed-loop feedback: inject measured positions back into controller */
        if (torque_enabled) {
            hc_joint_angles_t measured[HC_LEG_COUNT];
            if (read_joint_angles(&servo_bus, measured)) {
                consec_read_fails = 0;
                hc_controller_set_measured_pose(&ctrl, measured);
            } else {
                if (++consec_read_fails == kMaxConsecReadFails) {
                    ESP_LOGW(kTag, "closed-loop: %d consecutive read failures",
                             consec_read_fails);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS((int)(kControlDt * 1000.0)));
    }
}
