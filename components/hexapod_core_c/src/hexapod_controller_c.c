/**
 * hexapod_controller_c.c
 *
 * Top-level hexapod controller.
 * Port of hexapod::control::HexapodControllerCore from C++.
 */
#include "hexapod_controller_c.h"

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Default leg model constants (from C++ firmware/main/main.cpp)       */
/* ------------------------------------------------------------------ */

static void init_leg_model(hc_leg_model_t *m) {
    m->lengths.coxa  = 0.0475;
    m->lengths.femur = 0.1100;
    m->lengths.tibia = 0.20087;

    m->limits.coxa.min_angle  = -1.220;
    m->limits.coxa.max_angle  =  1.301;
    m->limits.femur.min_angle = -2.293;
    m->limits.femur.max_angle =  1.211;
    m->limits.tibia.min_angle = -0.072;
    m->limits.tibia.max_angle =  2.673;
}

/* ------------------------------------------------------------------ */
/* Build servo frame from smoothed pose                                */
/* ------------------------------------------------------------------ */

static hc_servo_frame_t build_servo_frame(const hc_controller_t *ctrl) {
    hc_servo_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    const double freq = ctrl->gait_frequency_hz;

    double speed_d = ctrl->speed_base + ctrl->speed_from_freq_scale * freq;
    if (speed_d > ctrl->max_speed) speed_d = ctrl->max_speed;
    if (speed_d < ctrl->speed_base) speed_d = ctrl->speed_base;

    double acc_d = ctrl->acc_base + ctrl->acc_from_freq_scale * freq;
    if (acc_d > ctrl->max_acc) acc_d = ctrl->max_acc;
    if (acc_d < ctrl->acc_base) acc_d = ctrl->acc_base;

    const uint16_t speed = (uint16_t)speed_d;
    const uint8_t  acc   = (uint8_t)acc_d;

    const hc_joint_angles_t *pose =
        hc_leg_controller_smoothed_pose(&ctrl->leg_ctrl);

    for (size_t i = 0; i < HC_SERVO_COUNT; ++i) {
        const hc_joint_angles_t *leg_angles = &pose[i / HC_JOINTS_PER_LEG];
        double rad;
        switch (i % HC_JOINTS_PER_LEG) {
            case 0:  rad = leg_angles->coxa;  break;
            case 1:  rad = leg_angles->femur; break;
            default: rad = leg_angles->tibia; break;
        }
        frame.position_raw[i] = hc_rad_to_raw(rad);
        frame.speed_raw[i]    = speed;
        frame.acc_raw[i]      = acc;
    }

    return frame;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void hc_controller_init(hc_controller_t *ctrl) {
    if (!ctrl) return;
    memset(ctrl, 0, sizeof(*ctrl));

    init_leg_model(&ctrl->leg_model);

    /* Leg controller: stand foot at (0.18, 0, -0.033) */
    hc_leg_controller_init(&ctrl->leg_ctrl, &ctrl->leg_model,
                            0.18, 0.0, -0.033);

    hc_gait_controller_init(&ctrl->gait_ctrl, &ctrl->leg_model);

    ctrl->gait_enabled       = false;
    ctrl->gait_frequency_hz  = 2.0;

    /* ServoCommandBuilder defaults */
    ctrl->speed_base            = 300.0;
    ctrl->speed_from_freq_scale = 350.0;
    ctrl->max_speed             = 1200.0;
    ctrl->acc_base              = 50.0;
    ctrl->acc_from_freq_scale   = 30.0;
    ctrl->max_acc               = 180.0;
}

void hc_controller_set_body_pose(hc_controller_t *ctrl,
                                  double x, double y, double theta) {
    if (!ctrl) return;
    ctrl->body_state.x     = x;
    ctrl->body_state.y     = y;
    ctrl->body_state.theta = theta;
}

void hc_controller_set_velocity(hc_controller_t *ctrl,
                                 double vx, double vy, double omega_z) {
    if (!ctrl) return;
    ctrl->body_state.vx      = vx;
    ctrl->body_state.vy      = vy;
    ctrl->body_state.omega_z = omega_z;
}

void hc_controller_enable_gait(hc_controller_t *ctrl, double frequency_hz) {
    if (!ctrl) return;
    if (frequency_hz < 0.2) frequency_hz = 0.2;
    if (frequency_hz > 3.0) frequency_hz = 3.0;
    ctrl->gait_frequency_hz = frequency_hz;
    hc_gait_controller_set_frequency(&ctrl->gait_ctrl, frequency_hz);
    if (!ctrl->gait_enabled) {
        ctrl->gait_enabled = true;
        hc_gait_controller_reset(&ctrl->gait_ctrl);
    }
}

void hc_controller_disable_gait(hc_controller_t *ctrl) {
    if (!ctrl) return;
    ctrl->gait_enabled = false;
}

void hc_controller_set_stride_scale(hc_controller_t *ctrl, double scale) {
    if (!ctrl) return;
    hc_gait_controller_set_stride_scale(&ctrl->gait_ctrl, scale);
}

void hc_controller_set_step_height_scale(hc_controller_t *ctrl, double scale) {
    if (!ctrl) return;
    hc_gait_controller_set_step_height_scale(&ctrl->gait_ctrl, scale);
}

void hc_controller_set_measured_pose(hc_controller_t *ctrl,
                                      const hc_joint_angles_t measured[HC_LEG_COUNT]) {
    if (!ctrl || !measured) return;
    hc_leg_controller_set_measured_pose(&ctrl->leg_ctrl, measured);
}

hc_servo_frame_t hc_controller_update(hc_controller_t *ctrl, double dt) {
    hc_servo_frame_t empty;
    memset(&empty, 0, sizeof(empty));
    if (!ctrl) return empty;

    if (dt <= 0.0 || dt > 1.0) dt = 0.001;

    const hc_leg_target_set_t *targets = NULL;

    if (ctrl->gait_enabled) {
        hc_gait_controller_set_body_state(&ctrl->gait_ctrl, &ctrl->body_state);
        hc_gait_controller_set_frequency(&ctrl->gait_ctrl, ctrl->gait_frequency_hz);
        hc_gait_controller_advance(&ctrl->gait_ctrl, dt);
        targets = hc_gait_controller_current_targets(&ctrl->gait_ctrl);
    }

    hc_joint_angles_t commanded[HC_LEG_COUNT];
    if (targets) {
        hc_leg_controller_resolve_targets(&ctrl->leg_ctrl, targets, commanded);
    } else {
        /* Stand: use stand pose as command */
        for (size_t i = 0; i < HC_LEG_COUNT; ++i) {
            commanded[i] = ctrl->leg_ctrl.stand_pose[i];
        }
    }

    hc_leg_controller_smooth_toward(&ctrl->leg_ctrl, commanded,
                                     dt, ctrl->gait_frequency_hz);

    return build_servo_frame(ctrl);
}

const hc_joint_angles_t *hc_controller_smoothed_pose(const hc_controller_t *ctrl) {
    if (!ctrl) return NULL;
    return hc_leg_controller_smoothed_pose(&ctrl->leg_ctrl);
}
