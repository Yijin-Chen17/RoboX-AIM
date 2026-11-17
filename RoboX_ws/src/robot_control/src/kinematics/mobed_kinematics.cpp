#include "robot_control/kinematics/mobed_kinematics.hpp"
#include <algorithm>

namespace robot_control {
namespace kinematics {

MobedKinematics::MobedKinematics(const MobedParameters& params) : params_(params) {}

double MobedKinematics::normalize_angle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::Matrix<double, 4, 3> MobedKinematics::getSteerPositions() const {
    Eigen::Matrix<double, 4, 3> steer_pos;
    steer_pos.row(FL) <<  params_.length_x,  params_.width_y, 0.0;
    steer_pos.row(FR) <<  params_.length_x, -params_.width_y, 0.0;
    steer_pos.row(RL) << -params_.length_x,  params_.width_y, 0.0;
    steer_pos.row(RR) << -params_.length_x, -params_.width_y, 0.0;
    return steer_pos;
}

MobedKinematics::SteeringTargets MobedKinematics::computeSteeringTargets(
    const Eigen::Vector3d& body_twist,
    const Eigen::Vector4d& previous_steer_targets,
    const Eigen::Vector4d& eccentric_angles) const {
    SteeringTargets targets{previous_steer_targets, previous_steer_targets};
    const auto steer_positions = getSteerPositions();
    const double yaw_rate = body_twist.z();

    for (int i = 0; i < NUM_LEGS; ++i) {
        const Eigen::Vector2d axis_velocity(
            body_twist.x() - yaw_rate * steer_positions(i, 1),
            body_twist.y() + yaw_rate * steer_positions(i, 0));
        const double axis_speed = axis_velocity.norm();
        if (axis_speed < 1e-3 * params_.r_wheel) {
            continue;
        }

        // Zero lateral velocity requires sin(steer - heading) = yaw_rate *
        // eccentric_reach / axis_speed. If there is no exact rolling solution,
        // the clamped value minimizes lateral velocity before sector limits.
        const double heading = std::atan2(axis_velocity.y(), axis_velocity.x());
        const double eccentric_reach = params_.l_ecc * std::sin(eccentric_angles(i));
        const double angle_offset = std::asin(std::clamp(
            yaw_rate * eccentric_reach / axis_speed, -1.0, 1.0));
        targets.direct(i) = normalize_angle(heading + angle_offset);
        targets.reversed(i) = normalize_angle(heading + M_PI - angle_offset);
    }

    return targets;
}

Eigen::Vector4d MobedKinematics::computePostureIK(
    double target_height, 
    double target_roll, 
    double target_pitch,
    const Eigen::Vector4d& current_steer_angles,
    const Eigen::Matrix3d& R_T,
    PostureConfiguration config,
    bool outward_half_circle) const {
    
    Eigen::Vector4d target_ecc_angles;
    
    double safe_height = std::max(params_.min_height, std::min(target_height, params_.max_height));
    
    // R_B: Rotation from Base to World (Standard Tait-Bryan ZYX with zero yaw)
    Eigen::Matrix3d R_B;
    R_B = Eigen::AngleAxisd(target_pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(target_roll, Eigen::Vector3d::UnitX());
        
    // R_TB: Rotation from Base to Terrain (R_TB = R_T^T * R_B)
    Eigen::Matrix3d R_TB = R_T.transpose() * R_B;
    
    // We need the 3rd row of R_TB for the Z_terrain equation
    double rtb_31 = R_TB(2, 0);
    double rtb_32 = R_TB(2, 1);
    double rtb_33 = R_TB(2, 2);

    for (int i = 0; i < 4; ++i) {
        double px = (i == FL || i == FR) ? params_.length_x : -params_.length_x;
        double py = (i == FL || i == RL) ? params_.width_y : -params_.width_y;
        
        double q_str = current_steer_angles(i);
        
        double A = params_.l_ecc * (rtb_31 * std::cos(q_str) + rtb_32 * std::sin(q_str));
        double B = -params_.l_ecc * rtb_33;
        
        // C = target_height + (R_TB * r_BOC)_z (with ecc angle parts separated into A and B)
        // Corrected physical z_offset from CAD multiplied by rtb_33
        double C = safe_height + rtb_31 * px + rtb_32 * py - rtb_33 * params_.r_wheel - rtb_33 * params_.posture_z_offset;
        
        double R = std::sqrt(A * A + B * B);
        double sin_val = -C / R;
        sin_val = std::clamp(sin_val, -1.0, 1.0);
        
        double alpha = std::atan2(B, A);
        
        double sol1 = normalize_angle(std::asin(sin_val) - alpha);
        double sol2 = normalize_angle(M_PI - std::asin(sin_val) - alpha);

        const bool is_front = i == FL || i == FR;
        if (outward_half_circle && config == PostureConfiguration::OUTWARD) {
            // q = 0 points down. In the forward steering reference, positive
            // angles sweep forward and negative angles sweep rearward.
            const double sign = is_front ? 1.0 : -1.0;
            const auto on_branch = [sign](double angle) {
                return std::abs(angle) == M_PI ? sign * M_PI : angle;
            };
            sol1 = on_branch(sol1);
            sol2 = on_branch(sol2);
            if (sign * sol1 >= 0.0) {
                target_ecc_angles(i) = sol1;
            } else if (sign * sol2 >= 0.0) {
                target_ecc_angles(i) = sol2;
            } else {
                // With no solution on this half circle, use the endpoint
                // whose projected height is closest to the requested one.
                target_ecc_angles(i) = std::abs(C + B) <= std::abs(C - B) ? 0.0 : sign * M_PI;
            }
            continue;
        }
        
        // Strict sign selection & feasible workspace filtering (Section II.B & III.E.3, Eq 15)
        // Normal operating range: eccentric arm points downwards/backwards to support the chassis.
        // Expanded to 120 deg (2.094 rad) to provide full deep-crouch stroke for 10-12 deg slope leveling.
        double max_angle = 2.094; // ~120 deg
        bool sol1_feasible = std::abs(sol1) <= max_angle;
        bool sol2_feasible = std::abs(sol2) <= max_angle;

        bool target_positive = true;
        if (config == PostureConfiguration::OUTWARD) {
            target_positive = is_front;
        } else if (config == PostureConfiguration::INWARD) {
            target_positive = !is_front;
        } else if (config == PostureConfiguration::SPIN) {
            // Tangential alignment for in-place spin (Section II.B & Fig 5a 2)
            target_positive = (i == FL || i == RL);
        } else if (config == PostureConfiguration::STRAFE_OUTWARD) {
            // Lateral strafe wide stance (外八字):
            // When wheels are steered sideways (FL/RR: -90 deg, FR/RL: +90 deg),
            // front ecc < 0 and rear ecc > 0 flares wheels outwards to track width 391mm
            target_positive = !is_front;
        }

        double q_ecc;
        if (target_positive) {
            // Prefer positive solution within feasible range [0, max_angle]
            if (sol1_feasible && sol1 >= 0.0) q_ecc = sol1;
            else if (sol2_feasible && sol2 >= 0.0) q_ecc = sol2;
            else if (sol1_feasible) q_ecc = sol1;
            else if (sol2_feasible) q_ecc = sol2;
            else q_ecc = (sol1 >= 0.0) ? sol1 : sol2;
        } else {
            // Prefer negative solution within feasible range [-max_angle, 0]
            if (sol1_feasible && sol1 <= 0.0) q_ecc = sol1;
            else if (sol2_feasible && sol2 <= 0.0) q_ecc = sol2;
            else if (sol1_feasible) q_ecc = sol1;
            else if (sol2_feasible) q_ecc = sol2;
            else q_ecc = (sol1 <= 0.0) ? sol1 : sol2;
        }

        target_ecc_angles(i) = normalize_angle(q_ecc);
    }
    
    return target_ecc_angles;
}

Eigen::Vector3d MobedKinematics::computeFootPositionInBase(
    int leg_index, double q_str, double q_ecc) const {
    double px = (leg_index == FL || leg_index == FR) ? params_.length_x : -params_.length_x;
    double py = (leg_index == FL || leg_index == RL) ? params_.width_y : -params_.width_y;

    // Contact point vector ^B r_i from base origin to contact point C (Section III.A & Eq 2)
    return Eigen::Vector3d(
        px + params_.l_ecc * std::cos(q_str) * std::sin(q_ecc),
        py + params_.l_ecc * std::sin(q_str) * std::sin(q_ecc),
        -params_.posture_z_offset - params_.l_ecc * std::cos(q_ecc) - params_.r_wheel
    );
}

MobedKinematics::WheelRollingSpeeds MobedKinematics::computeWheelRollingSpeeds(
    int leg_index, const Eigen::Vector3d& body_twist,
    double q_str, double q_ecc, double steer_rate, double eccentric_rate) const {
    const Eigen::Vector2d rolling_direction(std::cos(q_str), std::sin(q_str));
    const Eigen::Vector2d lateral_direction(-rolling_direction.y(), rolling_direction.x());
    const double lateral_offset = (leg_index == FL || leg_index == RL)
        ? params_.wheel_scrub_offset_y : -params_.wheel_scrub_offset_y;
    const Eigen::Vector2d wheel_position =
        computeFootPositionInBase(leg_index, q_str, q_ecc).head<2>() +
        lateral_offset * lateral_direction;
    const Eigen::Vector2d chassis_velocity =
        body_twist.head<2>() +
        body_twist.z() * Eigen::Vector2d(-wheel_position.y(), wheel_position.x());

    // The eccentric arm's longitudinal offset sweeps sideways during steering.
    // Only the lateral offset contributes to rolling speed from steering motion.
    const double joint_rolling_velocity =
        -lateral_offset * steer_rate + params_.l_ecc * std::cos(q_ecc) * eccentric_rate;
    return {rolling_direction.dot(chassis_velocity) / params_.r_wheel,
            joint_rolling_velocity / params_.r_wheel};
}

MobedKinematics::SteerSectorConfig MobedKinematics::getLegSafeSectorConfig(int leg_index) const {
    SteerSectorConfig cfg;
    // Reflects CAD coordinate mirror symmetry between left legs (FL, RL) and right legs (FR, RR):
    // For lateral strafe, wheels must flare outwards away from each other along X:
    // FL: steer to -90 deg (swings forward, X=+0.217m) -> center = -20 deg (-0.349 rad)
    // FR: steer to +90 deg (swings forward, X=+0.217m) -> center = +20 deg (+0.349 rad)
    // RL: steer to +90 deg (swings rearward, X=-0.213m) -> center = +20 deg (+0.349 rad)
    // RR: steer to -90 deg (swings rearward, X=-0.213m) -> center = -20 deg (-0.349 rad)
    // Safe half-width = 72 deg (1.257 rad), giving safe sector span [-92 deg, +52 deg] or [-52 deg, +92 deg].
    // This expands wheelbase to 430mm on both flanks, completely eliminating wheel-wheel collision during strafe.
    if (leg_index == FL || leg_index == RR) {
        cfg.center_angle = -0.349066; // -20 deg (-0.349 rad)
        cfg.half_width   = 1.256637;  // 72 deg (1.257 rad)
    } else {
        cfg.center_angle = 0.349066;  // +20 deg (+0.349 rad)
        cfg.half_width   = 1.256637;  // 72 deg (1.257 rad)
    }
    return cfg;
}

MobedKinematics::SteerForbiddenSector MobedKinematics::getForbiddenSteerSectorInfo(int leg_index) const {
    auto cfg = getLegSafeSectorConfig(leg_index);
    SteerForbiddenSector sector;
    sector.center_angle = normalize_angle(cfg.center_angle + M_PI);
    sector.half_width   = M_PI - cfg.half_width;
    sector.active       = true;
    return sector;
}

bool MobedKinematics::isSteerInForbiddenSector(int leg_index, double steer_angle, double margin) const {
    auto cfg = getLegSafeSectorConfig(leg_index);
    double dist = std::abs(normalize_angle(steer_angle - cfg.center_angle));
    return dist > (cfg.half_width - margin);
}

double MobedKinematics::clampToSafeSteerSector(int leg_index, double steer_angle, double margin) const {
    auto cfg = getLegSafeSectorConfig(leg_index);
    double delta = normalize_angle(steer_angle - cfg.center_angle);
    double limit = cfg.half_width - margin;
    double clamped_delta = std::clamp(delta, -limit, limit);
    return normalize_angle(cfg.center_angle + clamped_delta);
}

double MobedKinematics::getOptimalOutwardStanceAngle(int leg_index) const {
    // Widest track width stance (|Y| = 0.178m ~ 0.229m):
    // FL (0) & RR (3): -35 deg (-0.61 rad)
    // FR (1) & RL (2): +35 deg (+0.61 rad)
    return (leg_index == FL || leg_index == RR) ? -0.61 : 0.61;
}

std::pair<double, double> MobedKinematics::getForbiddenSteerSector(int /*leg_index*/, double /*q_ecc*/) const {
    return {MAX_SAFE_STEER_ANGLE, -MAX_SAFE_STEER_ANGLE};
}

} // namespace kinematics
} // namespace robot_control
