/**
 * hexapod_kinematics_c.h
 *
 * Pure-C kinematics layer: IK, FK, robot geometry.
 * Mirrors the C++ hexapod::kinematics namespace.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Basic types                                                          */
/* ------------------------------------------------------------------ */

typedef struct { double x, y, z; } hc_vec3_t;

typedef struct { double coxa, femur, tibia; } hc_lengths_t;

typedef struct { double min_angle, max_angle; } hc_joint_limit_t;

typedef struct {
  hc_joint_limit_t coxa;
  hc_joint_limit_t femur;
  hc_joint_limit_t tibia;
} hc_joint_limits_t;

typedef struct {
  hc_lengths_t    lengths;
  hc_joint_limits_t limits;
} hc_leg_model_t;

typedef struct { double coxa, femur, tibia; } hc_joint_angles_t;

typedef enum {
  HC_IK_SUCCESS               = 0,
  HC_IK_OUT_OF_REACH          = 1,
  HC_IK_JOINT_LIMIT_VIOLATION = 2,
} hc_ik_result_t;

/* ------------------------------------------------------------------ */
/* Robot geometry (6 legs)                                             */
/* ------------------------------------------------------------------ */

#define HC_LEG_COUNT 6

/**
 * Returns the base attachment position of a leg in body frame.
 * @param leg_index  0..5
 * @param out        filled with (x, y, z) in metres
 */
void hc_get_leg_base_position(size_t leg_index, hc_vec3_t *out);

/**
 * Returns the yaw angle of the leg-local frame relative to body frame.
 * @param leg_index  0..5
 */
double hc_get_leg_base_yaw(size_t leg_index);

/* ------------------------------------------------------------------ */
/* Forward Kinematics                                                   */
/* ------------------------------------------------------------------ */

/**
 * Compute foot position in coxa-origin frame given joint angles.
 */
void hc_compute_fk(const hc_leg_model_t *model,
                   const hc_joint_angles_t *angles,
                   hc_vec3_t *out_foot);

/* ------------------------------------------------------------------ */
/* Inverse Kinematics                                                   */
/* ------------------------------------------------------------------ */

/**
 * Solve IK for a foot target in coxa-origin (leg) frame.
 */
hc_ik_result_t hc_compute_ik(const hc_leg_model_t *model,
                              const hc_vec3_t *foot_target,
                              hc_joint_angles_t *out_angles);

/**
 * Solve IK with angular-continuity correction relative to previous angles.
 * Prefers solution closest to previousAngles to avoid joint branch flips.
 */
hc_ik_result_t hc_compute_ik_directed(const hc_leg_model_t *model,
                                       const hc_vec3_t *foot_target,
                                       const hc_joint_angles_t *prev_angles,
                                       hc_joint_angles_t *out_angles);

/* ------------------------------------------------------------------ */
/* Servo raw value helpers                                              */
/* ------------------------------------------------------------------ */

/** Convert joint angle in radians to ST servo raw position [0..4095]. */
int16_t hc_rad_to_raw(double radians);

/** Convert ST servo raw position to joint angle in radians. */
double hc_raw_to_rad(int16_t raw);

#ifdef __cplusplus
}
#endif
