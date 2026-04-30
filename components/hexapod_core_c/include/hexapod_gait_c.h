/**
 * hexapod_gait_c.h
 *
 * Tripod gait controller.
 * Mirrors C++ hexapod::control::TripodGaitController.
 *
 * Advances leg phases, calls foothold planner, converts footholds to
 * leg-local frames and fills hc_leg_target_set_t.
 */
#pragma once

#include <stddef.h>

#include "hexapod_kinematics_c.h"
#include "hexapod_foothold_planner_c.h"
#include "hexapod_leg_controller_c.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Body state fed to the gait controller each tick. */
typedef struct {
    double x, y, theta;   /**< World-frame body pose */
    double vx, vy;        /**< Commanded velocity (m/s) */
    double omega_z;       /**< Commanded angular velocity (rad/s) */
} hc_body_state_t;

/** Tripod gait controller instance. */
typedef struct {
    double leg_phases[HC_LEG_COUNT];  /**< Gait phases [0,1) per leg */
    double frequency_hz;              /**< Gait cycle frequency */
    double swing_ratio;               /**< Fraction of cycle in swing */

    hc_body_state_t    body_state;
    hc_foothold_planner_t planner;
    hc_leg_target_set_t   current_targets;
} hc_gait_controller_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/**
 * Initialise tripod gait controller.
 * @param gc        Instance
 * @param leg_model Shared leg model for workspace clamping (may be NULL)
 */
void hc_gait_controller_init(hc_gait_controller_t *gc,
                              const hc_leg_model_t *leg_model);

/** Reset phases and planner state. Call before starting gait. */
void hc_gait_controller_reset(hc_gait_controller_t *gc);

/** Set gait cycle frequency [0.2, 3.0] Hz. */
void hc_gait_controller_set_frequency(hc_gait_controller_t *gc, double hz);

/** Set stride scale multiplier. */
void hc_gait_controller_set_stride_scale(hc_gait_controller_t *gc, double scale);

/** Set swing height scale multiplier. */
void hc_gait_controller_set_step_height_scale(hc_gait_controller_t *gc, double scale);

/** Update body state (called each tick before Advance). */
void hc_gait_controller_set_body_state(hc_gait_controller_t *gc,
                                        const hc_body_state_t *state);

/**
 * Advance phases by dt and compute new leg targets.
 * After this call, hc_gait_controller_current_targets() is up-to-date.
 */
void hc_gait_controller_advance(hc_gait_controller_t *gc, double dt);

/** Read-only access to the latest computed targets. */
const hc_leg_target_set_t *hc_gait_controller_current_targets(
    const hc_gait_controller_t *gc);

/** Read-only access to the current leg phases. */
const double *hc_gait_controller_leg_phases(const hc_gait_controller_t *gc);

#ifdef __cplusplus
}
#endif
