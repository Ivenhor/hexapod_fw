/**
 * hexapod_controller_c.h
 *
 * Top-level hexapod controller.
 * Mirrors C++ hexapod::control::HexapodControllerCore.
 *
 * Owns: leg model, leg controller, gait controller.
 * Produces: servo frame (raw positions, speed, acc) each tick.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hexapod_kinematics_c.h"
#include "hexapod_leg_controller_c.h"
#include "hexapod_gait_c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Servo frame                                                         */
/* ------------------------------------------------------------------ */

typedef struct {
    int16_t  position_raw[HC_SERVO_COUNT];
    uint16_t speed_raw[HC_SERVO_COUNT];
    uint8_t  acc_raw[HC_SERVO_COUNT];
} hc_servo_frame_t;

/* ------------------------------------------------------------------ */
/* Controller instance                                                 */
/* ------------------------------------------------------------------ */

typedef struct {
    hc_leg_model_t      leg_model;    /**< Embedded (owned) leg model */
    hc_leg_controller_t leg_ctrl;
    hc_gait_controller_t gait_ctrl;

    hc_body_state_t     body_state;

    bool   gait_enabled;
    double gait_frequency_hz;

    /* ServoCommandBuilder config (mirrors ServoCommandBuilder::Config) */
    double speed_base;
    double speed_from_freq_scale;
    double max_speed;
    double acc_base;
    double acc_from_freq_scale;
    double max_acc;
} hc_controller_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/**
 * Initialise controller with default leg model constants.
 * Default gait frequency: 2 Hz.
 */
void hc_controller_init(hc_controller_t *ctrl);

/** Set body pose in world frame. */
void hc_controller_set_body_pose(hc_controller_t *ctrl,
                                  double x, double y, double theta);

/** Set velocity command (m/s, m/s, rad/s). */
void hc_controller_set_velocity(hc_controller_t *ctrl,
                                 double vx, double vy, double omega_z);

/** Enable tripod gait at the given frequency. */
void hc_controller_enable_gait(hc_controller_t *ctrl, double frequency_hz);

/** Disable gait (switch to stand). */
void hc_controller_disable_gait(hc_controller_t *ctrl);

/** Set stride scale [0..1+]. */
void hc_controller_set_stride_scale(hc_controller_t *ctrl, double scale);

/** Set step height scale [0..1+]. */
void hc_controller_set_step_height_scale(hc_controller_t *ctrl, double scale);

/**
 * Inject measured servo angles as current smoothed pose.
 * Closes the position feedback loop each tick.
 */
void hc_controller_set_measured_pose(hc_controller_t *ctrl,
                                      const hc_joint_angles_t measured[HC_LEG_COUNT]);

/**
 * Advance controller by dt and return the servo frame to write.
 * @param dt  Control loop timestep in seconds
 */
hc_servo_frame_t hc_controller_update(hc_controller_t *ctrl, double dt);

/** Access smoothed pose (for readback / diagnostics). */
const hc_joint_angles_t *hc_controller_smoothed_pose(const hc_controller_t *ctrl);

#ifdef __cplusplus
}
#endif
