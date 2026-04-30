/**
 * hexapod_foothold_planner_c.c
 *
 * Stateful foothold planner for tripod gait.
 * Port of hexapod::motion::FootholdPlanner from C++.
 */
#include "hexapod_foothold_planner_c.h"

#include <math.h>
#include <string.h>

static const double kPi = 3.14159265358979323846;

/* ------------------------------------------------------------------ */
/* Cubic Bezier scalar interpolation                                   */
/* ------------------------------------------------------------------ */

static double cubic_bezier(double t, double p0, double p1, double p2, double p3) {
    const double u = 1.0 - t;
    return u*u*u*p0 + 3.0*u*u*t*p1 + 3.0*u*t*t*p2 + t*t*t*p3;
}

/* ------------------------------------------------------------------ */
/* Coordinate transforms                                               */
/* ------------------------------------------------------------------ */

/** Body frame → world frame */
static hc_vec3_t body_to_world(const hc_foothold_planner_t *p,
                                const hc_vec3_t *pb) {
    const double c = cos(p->body_theta);
    const double s = sin(p->body_theta);
    hc_vec3_t pw;
    pw.x = p->body_x + c * pb->x - s * pb->y;
    pw.y = p->body_y + s * pb->x + c * pb->y;
    pw.z = pb->z;
    return pw;
}

/** World frame → body frame */
static hc_vec3_t world_to_body(const hc_foothold_planner_t *p,
                                const hc_vec3_t *pw) {
    const double c  = cos(p->body_theta);
    const double s  = sin(p->body_theta);
    const double dx = pw->x - p->body_x;
    const double dy = pw->y - p->body_y;
    hc_vec3_t pb;
    pb.x =  c * dx + s * dy;
    pb.y = -s * dx + c * dy;
    pb.z = pw->z;
    return pb;
}

/** Body frame -> leg-local frame */
static hc_vec3_t body_to_leg_frame(size_t leg_index,
                                    const hc_vec3_t *body_point) {
    hc_vec3_t leg_base;
    hc_get_leg_base_position(leg_index, &leg_base);
    const double leg_yaw = hc_get_leg_base_yaw(leg_index);
    const double lc = cos(leg_yaw);
    const double ls = sin(leg_yaw);
    const double rel_x = body_point->x - leg_base.x;
    const double rel_y = body_point->y - leg_base.y;

    hc_vec3_t leg_point;
    leg_point.x =  lc * rel_x + ls * rel_y;
    leg_point.y = -ls * rel_x + lc * rel_y;
    leg_point.z = body_point->z - leg_base.z;
    return leg_point;
}

/** Leg-local frame -> body frame */
static hc_vec3_t leg_to_body_frame(size_t leg_index,
                                    const hc_vec3_t *leg_point) {
    hc_vec3_t leg_base;
    hc_get_leg_base_position(leg_index, &leg_base);
    const double leg_yaw = hc_get_leg_base_yaw(leg_index);
    const double lc = cos(leg_yaw);
    const double ls = sin(leg_yaw);

    hc_vec3_t body_point;
    body_point.x = leg_base.x + lc * leg_point->x - ls * leg_point->y;
    body_point.y = leg_base.y + ls * leg_point->x + lc * leg_point->y;
    body_point.z = leg_base.z + leg_point->z;
    return body_point;
}

static bool is_ik_success(const hc_leg_model_t *leg_model,
                          const hc_vec3_t *target_leg) {
    hc_joint_angles_t angles;
    return hc_compute_ik(leg_model, target_leg, &angles) == HC_IK_SUCCESS;
}

static hc_vec3_t clamp_leg_target_to_workspace(const hc_foothold_planner_t *p,
                                                 const hc_vec3_t *target_leg,
                                                 const hc_vec3_t *neutral_leg) {
    if (!p->leg_model) {
        return *target_leg;
    }

    if (is_ik_success(p->leg_model, target_leg)) {
        return *target_leg;
    }

    double lo = 0.0;
    double hi = 1.0;
    for (int i = 0; i < 24; ++i) {
        const double mid = 0.5 * (lo + hi);
        hc_vec3_t candidate;
        candidate.x = neutral_leg->x + (target_leg->x - neutral_leg->x) * mid;
        candidate.y = neutral_leg->y + (target_leg->y - neutral_leg->y) * mid;
        candidate.z = neutral_leg->z + (target_leg->z - neutral_leg->z) * mid;

        if (is_ik_success(p->leg_model, &candidate)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    hc_vec3_t clamped;
    clamped.x = neutral_leg->x + (target_leg->x - neutral_leg->x) * lo;
    clamped.y = neutral_leg->y + (target_leg->y - neutral_leg->y) * lo;
    clamped.z = neutral_leg->z + (target_leg->z - neutral_leg->z) * lo;
    return clamped;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void hc_foothold_planner_init(hc_foothold_planner_t *p,
                               const hc_leg_model_t *leg_model) {
    if (!p) return;
    memset(p, 0, sizeof(*p));
    p->stride_scale       = 0.6;
    p->swing_height_scale = 1.0;
    p->leg_model          = leg_model;
}

void hc_foothold_planner_reset(hc_foothold_planner_t *p) {
    if (!p) return;
    for (size_t i = 0; i < HC_LEG_COUNT; ++i) {
        memset(&p->leg_state[i], 0, sizeof(p->leg_state[i]));
    }
}

void hc_foothold_planner_set_body_pose(hc_foothold_planner_t *p,
                                        double x, double y, double theta) {
    if (!p) return;
    p->body_x     = x;
    p->body_y     = y;
    p->body_theta = theta;
}

void hc_foothold_planner_set_velocity(hc_foothold_planner_t *p,
                                       double vx, double vy, double omega_z) {
    if (!p) return;
    p->vx      = vx;
    p->vy      = vy;
    p->omega_z = omega_z;
}

void hc_foothold_planner_compute(hc_foothold_planner_t *p,
                                  size_t leg_index,
                                  double phase,
                                  double swing_ratio,
                                  hc_vec3_t *out) {
    if (!p || !out || leg_index >= HC_LEG_COUNT) return;

    /* Keep phase in [0, 1) */
    phase = phase - floor(phase);

    /* Leg base in body frame */
    hc_vec3_t leg_base;
    hc_get_leg_base_position(leg_index, &leg_base);
    const double leg_yaw = hc_get_leg_base_yaw(leg_index);

    /* Neutral foot in body frame (forward 0.18m in leg-local X, z=-0.1m) */
    const double neutral_x_leg = 0.18;
    const double neutral_y_leg = 0.0;
    const double neutral_z     = -0.1;

    const double lc = cos(leg_yaw);
    const double ls = sin(leg_yaw);

    hc_vec3_t neutral_body;
    neutral_body.x = leg_base.x + lc * neutral_x_leg - ls * neutral_y_leg;
    neutral_body.y = leg_base.y + ls * neutral_x_leg + lc * neutral_y_leg;
    neutral_body.z = leg_base.z + neutral_z;

    /* Stride vector in body frame with rotation contribution */
    double stride_bx = p->vx * p->stride_scale;
    double stride_by = p->vy * p->stride_scale;
    const double rot_gain = 0.60;
    stride_bx += -p->omega_z * neutral_body.y * rot_gain;
    stride_by +=  p->omega_z * neutral_body.x * rot_gain;

    /* Clamp stride */
    if (stride_bx >  0.25) stride_bx =  0.25;
    if (stride_bx < -0.25) stride_bx = -0.25;
    if (stride_by >  0.12) stride_by =  0.12;
    if (stride_by < -0.12) stride_by = -0.12;

    const double stride_mag = sqrt(stride_bx * stride_bx + stride_by * stride_by);
    double step_height = (0.040 + 0.085 * stride_mag) * p->swing_height_scale;
    if (step_height > 0.120) step_height = 0.120;
    if (step_height < 0.0)   step_height = 0.0;

    /* Neutral in world frame */
    const hc_vec3_t neutral_world = body_to_world(p, &neutral_body);

    /* Stateful anchor */
    hc_leg_state_t *ls_state = &p->leg_state[leg_index];
    const bool in_swing = (phase < swing_ratio);

    if (!ls_state->initialized) {
        ls_state->anchor     = neutral_world;
        ls_state->swing_start = neutral_world;
        ls_state->swing_end   = neutral_world;
        ls_state->was_in_swing = in_swing;
        ls_state->initialized  = true;
    }

    if (in_swing && !ls_state->was_in_swing) {
        /* Stance → swing transition */
        ls_state->swing_start = neutral_world;

        /* Stride vector in world frame for landing target */
        const double bc = cos(p->body_theta);
        const double bs = sin(p->body_theta);
        ls_state->swing_end.x = neutral_world.x + 0.5 * (bc * stride_bx - bs * stride_by);
        ls_state->swing_end.y = neutral_world.y + 0.5 * (bs * stride_bx + bc * stride_by);
        ls_state->swing_end.z = neutral_world.z;
    }
    ls_state->was_in_swing = in_swing;

    /* Trajectory */
    double delta_bx = 0.0;
    double delta_by = 0.0;
    double foothold_bz = neutral_body.z;

    if (in_swing) {
        const double t           = phase / swing_ratio;
        const double xy_progress = cubic_bezier(t, 0.0, 0.14, 0.88, 1.0);
        delta_bx = -0.5 * stride_bx + stride_bx * xy_progress;
        delta_by = -0.5 * stride_by + stride_by * xy_progress;

        const double z_late_peak  = cubic_bezier(t, 0.0, 0.14, 1.0, 0.0);
        const double z_safety_arc = sin(kPi * t);
        const double z_lift       = z_late_peak > z_safety_arc ? z_late_peak : z_safety_arc;
        foothold_bz = neutral_body.z + step_height * (z_lift > 0.0 ? z_lift : 0.0);
    } else {
        const double t = (phase - swing_ratio) / (1.0 - swing_ratio);
        delta_bx = 0.5 * stride_bx - stride_bx * t;
        delta_by = 0.5 * stride_by - stride_by * t;
        foothold_bz = neutral_body.z;
    }

    hc_vec3_t foothold_body_unclamped;
    foothold_body_unclamped.x = neutral_body.x + delta_bx;
    foothold_body_unclamped.y = neutral_body.y + delta_by;
    foothold_body_unclamped.z = foothold_bz;

    const hc_vec3_t foothold_world = body_to_world(p, &foothold_body_unclamped);
    const hc_vec3_t foothold_body = world_to_body(p, &foothold_world);

    hc_vec3_t neutral_leg;
    neutral_leg.x = neutral_x_leg;
    neutral_leg.y = neutral_y_leg;
    neutral_leg.z = neutral_z;

    const hc_vec3_t foothold_leg = body_to_leg_frame(leg_index, &foothold_body);
    const hc_vec3_t clamped_leg =
        clamp_leg_target_to_workspace(p, &foothold_leg, &neutral_leg);
    const hc_vec3_t clamped_body = leg_to_body_frame(leg_index, &clamped_leg);

    *out = clamped_body;
}
