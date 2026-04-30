/**
 * hexapod_leg_controller_c.h
 *
 * Leg controller: IK resolution, angular smoothing, measured-pose feedback.
 * Mirrors C++ hexapod::control::LegController.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "hexapod_kinematics_c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HC_JOINTS_PER_LEG 3
#define HC_SERVO_COUNT    (HC_LEG_COUNT * HC_JOINTS_PER_LEG)

/** Target set: one foothold per leg in leg-local frame. */
typedef struct {
    hc_vec3_t foothold_leg[HC_LEG_COUNT]; /**< Target in leg-local frame */
    bool      active[HC_LEG_COUNT];
} hc_leg_target_set_t;

/** Leg controller instance. */
typedef struct {
    const hc_leg_model_t   *leg_model;

    hc_joint_angles_t stand_pose[HC_LEG_COUNT];
    hc_joint_angles_t smoothed_pose[HC_LEG_COUNT];
    hc_vec3_t         last_foothold_leg[HC_LEG_COUNT];
    bool              ik_failure_latched[HC_LEG_COUNT];

    hc_vec3_t stand_foot_target; /**< Default stand foot target in leg frame */
} hc_leg_controller_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/**
 * Initialise leg controller.
 * @param lc               Instance to initialise
 * @param leg_model        Shared leg model (must remain valid)
 * @param stand_foot_x/y/z Default stand foot target in leg-local frame
 */
void hc_leg_controller_init(hc_leg_controller_t *lc,
                             const hc_leg_model_t *leg_model,
                             double stand_foot_x,
                             double stand_foot_y,
                             double stand_foot_z);

/**
 * Resolve leg targets to joint angles via directed IK.
 * Populates commanded_pose_out.
 */
void hc_leg_controller_resolve_targets(
    hc_leg_controller_t *lc,
    const hc_leg_target_set_t *targets,
    hc_joint_angles_t commanded_pose_out[HC_LEG_COUNT]);

/**
 * Apply first-order low-pass smoothing toward commanded pose.
 * @param dt                  Control loop timestep (s)
 * @param gait_frequency_hz   Current gait frequency (influences time constant)
 */
void hc_leg_controller_smooth_toward(
    hc_leg_controller_t *lc,
    const hc_joint_angles_t commanded_pose[HC_LEG_COUNT],
    double dt,
    double gait_frequency_hz);

/**
 * Inject measured hardware joint angles as current smoothed pose.
 * Closes the position feedback loop.
 */
void hc_leg_controller_set_measured_pose(
    hc_leg_controller_t *lc,
    const hc_joint_angles_t measured[HC_LEG_COUNT]);

/** Access the current smoothed pose. */
const hc_joint_angles_t *hc_leg_controller_smoothed_pose(
    const hc_leg_controller_t *lc);

#ifdef __cplusplus
}
#endif
