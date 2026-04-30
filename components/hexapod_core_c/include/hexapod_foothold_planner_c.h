/**
 * hexapod_foothold_planner_c.h
 *
 * Stateful foothold planner for tripod gait.
 * Mirrors C++ hexapod::motion::FootholdPlanner.
 *
 * Per-leg Bezier trajectory during swing, fixed anchor during stance.
 * Operates in body frame; outputs foothold in body frame.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "hexapod_kinematics_c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */

/** Per-leg persistent state (anchor, swing start/end). */
typedef struct {
    hc_vec3_t anchor;       /**< World-frame planted position (held during stance) */
    hc_vec3_t swing_start;  /**< World-frame position at swing start */
    hc_vec3_t swing_end;    /**< World-frame landing target for current swing */
    bool was_in_swing;
    bool initialized;
} hc_leg_state_t;

/** Foothold planner instance. */
typedef struct {
    /* Body pose in world frame */
    double body_x;
    double body_y;
    double body_theta;

    /* Velocity command */
    double vx;
    double vy;
    double omega_z;

    double stride_scale;
    double swing_height_scale;

    hc_leg_state_t leg_state[HC_LEG_COUNT];

    /* Optional reference leg model for workspace clamping (may be NULL). */
    const hc_leg_model_t *leg_model;
} hc_foothold_planner_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/** Initialise planner to default values (stride_scale=0.6, swing_height=1.0). */
void hc_foothold_planner_init(hc_foothold_planner_t *p,
                               const hc_leg_model_t *leg_model);

/** Reset all per-leg anchor state. Call when gait is (re-)started. */
void hc_foothold_planner_reset(hc_foothold_planner_t *p);

/** Set current body pose in world frame (x, y, theta). */
void hc_foothold_planner_set_body_pose(hc_foothold_planner_t *p,
                                        double x, double y, double theta);

/** Set velocity command. */
void hc_foothold_planner_set_velocity(hc_foothold_planner_t *p,
                                       double vx, double vy, double omega_z);

/**
 * Compute foothold position in **body frame** for a leg at given gait phase.
 *
 * @param p           Planner instance
 * @param leg_index   0..5
 * @param phase       Gait phase [0, 1)
 * @param swing_ratio Fraction of cycle in swing (default 0.4)
 * @param out         Foothold in body frame
 */
void hc_foothold_planner_compute(hc_foothold_planner_t *p,
                                  size_t leg_index,
                                  double phase,
                                  double swing_ratio,
                                  hc_vec3_t *out);

#ifdef __cplusplus
}
#endif
