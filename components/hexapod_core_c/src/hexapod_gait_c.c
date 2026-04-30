/**
 * hexapod_gait_c.c
 *
 * Tripod gait controller.
 * Port of hexapod::control::TripodGaitController from C++.
 */
#include "hexapod_gait_c.h"

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/**
 * Convert a foothold from body frame to the leg-local frame.
 * Mirrors TripodGaitController::WorldToLegFrame applied after
 * FootholdPlanner (which already returns body frame in this port).
 */
static hc_vec3_t body_to_leg_frame(size_t leg_index,
                                    const hc_vec3_t *foothold_body) {
    hc_vec3_t base;
    hc_get_leg_base_position(leg_index, &base);
    const double yaw = hc_get_leg_base_yaw(leg_index);
    const double lc  = cos(yaw);
    const double ls  = sin(yaw);
    const double rel_x = foothold_body->x - base.x;
    const double rel_y = foothold_body->y - base.y;
    hc_vec3_t result;
    result.x =  lc * rel_x + ls * rel_y;
    result.y = -ls * rel_x + lc * rel_y;
    result.z =  foothold_body->z - base.z;
    return result;
}

static double hc_clamp_freq(double hz) {
    if (hz < 0.2) return 0.2;
    if (hz > 3.0) return 3.0;
    return hz;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void hc_gait_controller_init(hc_gait_controller_t *gc,
                              const hc_leg_model_t *leg_model) {
    if (!gc) return;
    memset(gc, 0, sizeof(*gc));
    gc->frequency_hz = 2.0;
    gc->swing_ratio  = 0.45;

    hc_foothold_planner_init(&gc->planner, leg_model);

    for (size_t i = 0; i < HC_LEG_COUNT; ++i) {
        gc->current_targets.active[i] = true;
    }

    hc_gait_controller_reset(gc);
}

void hc_gait_controller_reset(hc_gait_controller_t *gc) {
    if (!gc) return;
    /* Standard tripod offsets: legs {0,2,4} and {1,3,5} alternate */
    gc->leg_phases[0] = 0.00;
    gc->leg_phases[1] = 0.50;
    gc->leg_phases[2] = 0.00;
    gc->leg_phases[3] = 0.50;
    gc->leg_phases[4] = 0.00;
    gc->leg_phases[5] = 0.50;

    hc_foothold_planner_reset(&gc->planner);
}

void hc_gait_controller_set_frequency(hc_gait_controller_t *gc, double hz) {
    if (!gc) return;
    gc->frequency_hz = hc_clamp_freq(hz);
}

void hc_gait_controller_set_stride_scale(hc_gait_controller_t *gc, double scale) {
    if (!gc) return;
    if (scale < 0.0) scale = 0.0;
    gc->planner.stride_scale = scale;
}

void hc_gait_controller_set_step_height_scale(hc_gait_controller_t *gc, double scale) {
    if (!gc) return;
    if (scale < 0.0) scale = 0.0;
    gc->planner.swing_height_scale = scale;
}

void hc_gait_controller_set_body_state(hc_gait_controller_t *gc,
                                        const hc_body_state_t *state) {
    if (!gc || !state) return;
    gc->body_state = *state;
    hc_foothold_planner_set_body_pose(&gc->planner,
                                       state->x, state->y, state->theta);
    hc_foothold_planner_set_velocity(&gc->planner,
                                      state->vx, state->vy, state->omega_z);
}

void hc_gait_controller_advance(hc_gait_controller_t *gc, double dt) {
    if (!gc) return;
    if (dt <= 0.0 || dt > 1.0) dt = 0.001;

    hc_foothold_planner_set_body_pose(&gc->planner,
                                       gc->body_state.x,
                                       gc->body_state.y,
                                       gc->body_state.theta);
    hc_foothold_planner_set_velocity(&gc->planner,
                                      gc->body_state.vx,
                                      gc->body_state.vy,
                                      gc->body_state.omega_z);

    for (size_t i = 0; i < HC_LEG_COUNT; ++i) {
        gc->leg_phases[i] += dt * gc->frequency_hz;
        gc->leg_phases[i] -= floor(gc->leg_phases[i]);

        hc_vec3_t foothold_body;
        hc_foothold_planner_compute(&gc->planner, i,
                                     gc->leg_phases[i],
                                     gc->swing_ratio,
                                     &foothold_body);

        gc->current_targets.foothold_leg[i] =
            body_to_leg_frame(i, &foothold_body);
        gc->current_targets.active[i] = true;
    }
}

const hc_leg_target_set_t *hc_gait_controller_current_targets(
    const hc_gait_controller_t *gc) {
    if (!gc) return NULL;
    return &gc->current_targets;
}

const double *hc_gait_controller_leg_phases(const hc_gait_controller_t *gc) {
    if (!gc) return NULL;
    return gc->leg_phases;
}
