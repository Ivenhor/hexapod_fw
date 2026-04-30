/**
 * hexapod_kinematics_c.c
 *
 * Pure-C implementation of FK, IK, and robot geometry.
 * Ported from the C++ robot/kinematics sources.
 */
#include "hexapod_kinematics_c.h"

#include <math.h>
#include <stddef.h>

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

static const double kPi = 3.14159265358979323846;
static const double kTwoPi = 6.28318530717958647692;

/* ------------------------------------------------------------------ */
/* Robot geometry (leg base positions, from RobotGeometry.h values)    */
/* ------------------------------------------------------------------ */

static const double kLegBaseX[HC_LEG_COUNT] = {
     0.095,   /* 0 Front-Left   */
     0.000,   /* 1 Mid-Left     */
    -0.095,   /* 2 Rear-Left    */
    -0.095,   /* 3 Rear-Right   */
     0.000,   /* 4 Mid-Right    */
     0.095,   /* 5 Front-Right  */
};
static const double kLegBaseY[HC_LEG_COUNT] = {
     0.064,
     0.100,
     0.064,
    -0.064,
    -0.100,
    -0.064,
};
static const double kLegBaseZ[HC_LEG_COUNT] = {
    -0.0050, -0.0050, -0.0050,
    -0.0050, -0.0050, -0.0050,
};

/* Yaw of each leg-local frame in body frame (from RobotGeometry.h) */
static const double kLegBaseYaw[HC_LEG_COUNT] = {
    5.896,  /* 0 Front-Left  */
    4.710,  /* 1 Mid-Left    */
    3.523,  /* 2 Rear-Left   */
    2.759,  /* 3 Rear-Right  */
    1.570,  /* 4 Mid-Right   */
    0.383,  /* 5 Front-Right */
};

void hc_get_leg_base_position(size_t leg_index, hc_vec3_t *out) {
    if (!out || leg_index >= HC_LEG_COUNT) {
        return;
    }
    out->x = kLegBaseX[leg_index];
    out->y = kLegBaseY[leg_index];
    out->z = kLegBaseZ[leg_index];
}

double hc_get_leg_base_yaw(size_t leg_index) {
    if (leg_index >= HC_LEG_COUNT) {
        return 0.0;
    }
    return kLegBaseYaw[leg_index];
}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static double hc_clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static double hc_wrap_to_nearest(double angle, double reference) {
    const double delta = atan2(sin(angle - reference), cos(angle - reference));
    double nearest = reference + delta;

    double best = nearest;
    double best_dist = fabs(best - reference);

    const double cand_plus  = nearest + kTwoPi;
    const double cand_minus = nearest - kTwoPi;
    if (fabs(cand_plus - reference) < best_dist) {
        best = cand_plus;
        best_dist = fabs(cand_plus - reference);
    }
    if (fabs(cand_minus - reference) < best_dist) {
        best = cand_minus;
    }
    return best;
}

/* ------------------------------------------------------------------ */
/* Forward Kinematics                                                   */
/* ------------------------------------------------------------------ */

void hc_compute_fk(const hc_leg_model_t *model,
                   const hc_joint_angles_t *angles,
                   hc_vec3_t *out_foot) {
    if (!model || !angles || !out_foot) {
        return;
    }

    const double Lc = model->lengths.coxa;
    const double Lf = model->lengths.femur;
    const double Lt = model->lengths.tibia;

    /* Coxa rotates around Z; femur and tibia rotate around Y (in-plane). */
    const double sc = sin(angles->coxa);
    const double cc = cos(angles->coxa);

    const double sf = sin(angles->femur);
    const double cf = cos(angles->femur);

    /* tibia angle is relative to femur direction */
    const double sft = sin(angles->femur + angles->tibia);
    const double cft = cos(angles->femur + angles->tibia);

    /* Coxa endpoint */
    double cx = Lc * cc;
    double cy = Lc * sc;
    double cz = 0.0;

    /* Femur endpoint (Ry rotates in XZ plane; femur vector is (Lf,0,0) pre-rotation) */
    double fx = Lf * cc * cf;
    double fy = Lf * sc * cf;
    double fz = -Lf * sf;

    /* Tibia endpoint */
    double tx = Lt * cc * cft;
    double ty = Lt * sc * cft;
    double tz = -Lt * sft;

    out_foot->x = cx + fx + tx;
    out_foot->y = cy + fy + ty;
    out_foot->z = cz + fz + tz;
}

/* ------------------------------------------------------------------ */
/* Inverse Kinematics                                                   */
/* ------------------------------------------------------------------ */

hc_ik_result_t hc_compute_ik(const hc_leg_model_t *model,
                              const hc_vec3_t *foot_target,
                              hc_joint_angles_t *out_angles) {
    if (!model || !foot_target || !out_angles) {
        return HC_IK_OUT_OF_REACH;
    }

    const double Lc = model->lengths.coxa;
    const double Lf = model->lengths.femur;
    const double Lt = model->lengths.tibia;

    /* Coxa: horizontal sweep */
    const double coxa = atan2(foot_target->y, foot_target->x);

    /* Project onto leg plane */
    const double r  = sqrt(foot_target->x * foot_target->x +
                           foot_target->y * foot_target->y);
    const double dx = r - Lc;
    const double dz = foot_target->z;

    /* 2-link planar IK: target in the (dx, -dz) plane */
    const double px2d = dx;
    const double py2d = -dz;

    const double D2 = px2d * px2d + py2d * py2d;
    const double D  = sqrt(D2);

    const double reach_max = Lf + Lt;
    const double reach_min = (Lf > Lt) ? (Lf - Lt) : (Lt - Lf);
    if (D > reach_max || D < reach_min) {
        return HC_IK_OUT_OF_REACH;
    }

    double cos_qt = (D2 - Lf * Lf - Lt * Lt) / (2.0 * Lf * Lt);
    if (cos_qt >  1.0) cos_qt =  1.0;
    if (cos_qt < -1.0) cos_qt = -1.0;
    const double tibia = acos(cos_qt);

    const double alpha = atan2(py2d, px2d);
    const double beta  = atan2(Lt * sin(tibia), Lf + Lt * cos(tibia));
    const double femur = alpha - beta;

    out_angles->coxa  = coxa;
    out_angles->femur = femur;
    out_angles->tibia = tibia;

    /* Joint limit check */
    if (coxa  < model->limits.coxa.min_angle  || coxa  > model->limits.coxa.max_angle  ||
        femur < model->limits.femur.min_angle || femur > model->limits.femur.max_angle ||
        tibia < model->limits.tibia.min_angle || tibia > model->limits.tibia.max_angle) {
        return HC_IK_JOINT_LIMIT_VIOLATION;
    }

    return HC_IK_SUCCESS;
}

hc_ik_result_t hc_compute_ik_directed(const hc_leg_model_t *model,
                                       const hc_vec3_t *foot_target,
                                       const hc_joint_angles_t *prev_angles,
                                       hc_joint_angles_t *out_angles) {
    hc_joint_angles_t candidate = {0.0, 0.0, 0.0};
    const hc_ik_result_t status = hc_compute_ik(model, foot_target, &candidate);
    if (status == HC_IK_OUT_OF_REACH) {
        return status;
    }

    candidate.coxa  = hc_clamp(hc_wrap_to_nearest(candidate.coxa,  prev_angles->coxa),
                                model->limits.coxa.min_angle,  model->limits.coxa.max_angle);
    candidate.femur = hc_clamp(hc_wrap_to_nearest(candidate.femur, prev_angles->femur),
                                model->limits.femur.min_angle, model->limits.femur.max_angle);
    candidate.tibia = hc_clamp(hc_wrap_to_nearest(candidate.tibia, prev_angles->tibia),
                                model->limits.tibia.min_angle, model->limits.tibia.max_angle);

    *out_angles = candidate;

    if (candidate.coxa  < model->limits.coxa.min_angle  || candidate.coxa  > model->limits.coxa.max_angle  ||
        candidate.femur < model->limits.femur.min_angle || candidate.femur > model->limits.femur.max_angle ||
        candidate.tibia < model->limits.tibia.min_angle || candidate.tibia > model->limits.tibia.max_angle) {
        return HC_IK_JOINT_LIMIT_VIOLATION;
    }
    return status;
}

/* ------------------------------------------------------------------ */
/* Servo conversion                                                    */
/* ------------------------------------------------------------------ */

int16_t hc_rad_to_raw(double radians) {
    const double scaled = radians * (2047.0 / kPi) + 2047.0;
    long raw = (long)lround(scaled);
    if (raw < 0)    raw = 0;
    if (raw > 4095) raw = 4095;
    return (int16_t)raw;
}

double hc_raw_to_rad(int16_t raw) {
    return ((double)raw - 2047.0) * (kPi / 2047.0);
}
