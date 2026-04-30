/**
 * hexapod_leg_controller_c.c
 *
 * Leg controller: directed IK resolution, smoothing, feedback injection.
 * Port of hexapod::control::LegController from C++.
 */
#include "hexapod_leg_controller_c.h"

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static double hc_lc_clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/** Solve IK for stand foot target and fill all 6 legs identically. */
static void build_stand_pose(hc_leg_controller_t *lc) {
    hc_joint_angles_t solved = {0.0, 0.0, 0.0};
    const hc_ik_result_t status =
        hc_compute_ik(lc->leg_model, &lc->stand_foot_target, &solved);
    if (status == HC_IK_OUT_OF_REACH) {
        /* Fallback: reasonable standing configuration */
        solved.coxa  = 0.0;
        solved.femur = -0.5;
        solved.tibia = 2.0;
    }
    for (size_t i = 0; i < HC_LEG_COUNT; ++i) {
        lc->stand_pose[i] = solved;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void hc_leg_controller_init(hc_leg_controller_t *lc,
                             const hc_leg_model_t *leg_model,
                             double stand_foot_x,
                             double stand_foot_y,
                             double stand_foot_z) {
    if (!lc || !leg_model) return;
    memset(lc, 0, sizeof(*lc));
    lc->leg_model           = leg_model;
    lc->stand_foot_target.x = stand_foot_x;
    lc->stand_foot_target.y = stand_foot_y;
    lc->stand_foot_target.z = stand_foot_z;

    build_stand_pose(lc);

    for (size_t i = 0; i < HC_LEG_COUNT; ++i) {
        lc->smoothed_pose[i]      = lc->stand_pose[i];
        lc->last_foothold_leg[i]  = lc->stand_foot_target;
        lc->ik_failure_latched[i] = false;
    }
}

void hc_leg_controller_resolve_targets(
    hc_leg_controller_t *lc,
    const hc_leg_target_set_t *targets,
    hc_joint_angles_t commanded_pose_out[HC_LEG_COUNT]) {

    if (!lc || !targets || !commanded_pose_out) return;

    for (size_t i = 0; i < HC_LEG_COUNT; ++i) {
        /* Copy stand pose as default */
        commanded_pose_out[i] = lc->stand_pose[i];
    }

    for (size_t i = 0; i < HC_LEG_COUNT; ++i) {
        const hc_vec3_t *target = targets->active[i]
                                      ? &targets->foothold_leg[i]
                                      : &lc->stand_foot_target;
        lc->last_foothold_leg[i] = *target;

        hc_joint_angles_t solved = {0.0, 0.0, 0.0};
        const hc_ik_result_t status =
            hc_compute_ik_directed(lc->leg_model, target,
                                   &lc->smoothed_pose[i], &solved);

        const bool ik_ok = (status == HC_IK_SUCCESS ||
                            status == HC_IK_JOINT_LIMIT_VIOLATION);
        if (ik_ok) {
            commanded_pose_out[i]    = solved;
            lc->ik_failure_latched[i] = false;
        } else {
            commanded_pose_out[i]    = lc->stand_pose[i];
            lc->ik_failure_latched[i] = true;
        }
    }
}

void hc_leg_controller_smooth_toward(
    hc_leg_controller_t *lc,
    const hc_joint_angles_t commanded_pose[HC_LEG_COUNT],
    double dt,
    double gait_frequency_hz) {

    if (!lc || !commanded_pose) return;

    const double freq = (gait_frequency_hz > 1.0) ? gait_frequency_hz : 1.0;
    const double tau_base = 0.025;
    double tau = tau_base / freq;
    if (tau > tau_base) tau = tau_base;
    if (tau < 0.008)   tau = 0.008;

    const double alpha = hc_lc_clamp(dt / tau, 0.0, 1.0);

    for (size_t i = 0; i < HC_LEG_COUNT; ++i) {
        lc->smoothed_pose[i].coxa +=
            alpha * (commanded_pose[i].coxa - lc->smoothed_pose[i].coxa);
        lc->smoothed_pose[i].femur +=
            alpha * (commanded_pose[i].femur - lc->smoothed_pose[i].femur);
        lc->smoothed_pose[i].tibia +=
            alpha * (commanded_pose[i].tibia - lc->smoothed_pose[i].tibia);
    }
}

void hc_leg_controller_set_measured_pose(
    hc_leg_controller_t *lc,
    const hc_joint_angles_t measured[HC_LEG_COUNT]) {

    if (!lc || !measured) return;
    for (size_t i = 0; i < HC_LEG_COUNT; ++i) {
        lc->smoothed_pose[i] = measured[i];
    }
}

const hc_joint_angles_t *hc_leg_controller_smoothed_pose(
    const hc_leg_controller_t *lc) {
    if (!lc) return NULL;
    return lc->smoothed_pose;
}
