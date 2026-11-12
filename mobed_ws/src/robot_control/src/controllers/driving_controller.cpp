#include "robot_control/controllers/driving_controller.hpp"
#include <algorithm>
#include <cmath>

namespace robot_control {
namespace controllers {

DrivingController::DrivingController(const DrivingControllerParams& params,
                                     std::shared_ptr<kinematics::MobedKinematics> kinematics)
    : params_(params), kinematics_(kinematics) {
    prev_steer_angles_.setZero();
    prev_wheel_speeds_.setZero();
}

double DrivingController::computeBankAngle(const Eigen::Vector3d& cmd_vel) const {
    // ================================================================
    // Bank Angle Calculator (Paper Section III.D & Eq 8)
    // ================================================================

    double vx = cmd_vel(0); // Forward velocity
    double wz = cmd_vel(2); // Yaw rate

    // Lateral centripetal acceleration in body frame: a_y = wz * vx.
    // The resulting centrifugal inertial force on the body points in -Y (for wz*vx > 0).
    // In ISO frame (X fwd, Y left, Z up), leaning inward into a left turn requires
    // negative roll (left side drops, right side rises).
    // Eq (8): theta_b^d = -arctan(alpha_c_lateral / g)
    double alpha_c_lateral = wz * vx;

    // Desired inward bank angle
    double bank = -std::atan2(alpha_c_lateral, 9.81);

    // Clamp to maximum allowable bank
    bank = std::clamp(bank, -params_.max_bank_angle, params_.max_bank_angle);

    return bank;
}

double DrivingController::optimizeSwerveModule(
    int leg_index, double direct_angle, double reversed_angle, double curr_steer) {
    const double s_A = direct_angle;
    const double s_B = reversed_angle;

    // Safe Sector Check: evaluated against each leg's individual safe outward sector
    bool safe_A = !kinematics_->isSteerInForbiddenSector(leg_index, s_A);
    bool safe_B = !kinematics_->isSteerInForbiddenSector(leg_index, s_B);

    auto cfg = kinematics_->getLegSafeSectorConfig(leg_index);
    double u_curr = kinematics_->normalize_angle(curr_steer - cfg.center_angle);
    double u_A    = kinematics_->normalize_angle(s_A - cfg.center_angle);
    double u_B    = kinematics_->normalize_angle(s_B - cfg.center_angle);
    double diff_A = std::abs(u_A - u_curr);
    double diff_B = std::abs(u_B - u_curr);

    if (safe_A && safe_B) {
        const double hysteresis = params_.swerve_invert_hysteresis;
        const bool choose_B = prev_swerve_inverted_[leg_index]
            ? diff_B <= diff_A + hysteresis
            : diff_B + hysteresis < diff_A;
        prev_swerve_inverted_[leg_index] = choose_B;
        return choose_B ? s_B : s_A;
    }
    if (safe_A) {
        prev_swerve_inverted_[leg_index] = false;
        return s_A;
    }
    if (safe_B) {
        prev_swerve_inverted_[leg_index] = true;
        return s_B;
    }

    // Clamp the candidate closest to the safe sector when both are outside it.
    bool choose_B = (std::abs(u_B) < std::abs(u_A));
    prev_swerve_inverted_[leg_index] = choose_B;
    return kinematics_->clampToSafeSteerSector(leg_index, choose_B ? s_B : s_A);
}

double DrivingController::enforceForbiddenSectorTransit(
    int leg_index, double prev_s, double opt_s, double max_delta) const {

    // 1. Both endpoints are strictly confined inside this leg's safe outward sector
    opt_s  = kinematics_->clampToSafeSteerSector(leg_index, opt_s);
    prev_s = kinematics_->clampToSafeSteerSector(leg_index, prev_s);

    // 2. Continuous 1D monotonic transit in unwrapped safe coordinate centered at cfg.center_angle.
    // In this coordinate frame, the safe sector is [-half_width, +half_width], which is strictly convex.
    // Stepping between any two points is guaranteed to stay within the safe sector and never cross into the forbidden zone.
    auto cfg = kinematics_->getLegSafeSectorConfig(leg_index);
    double u_opt  = kinematics_->normalize_angle(opt_s - cfg.center_angle);
    double u_prev = kinematics_->normalize_angle(prev_s - cfg.center_angle);
    double delta  = u_opt - u_prev;
    double step   = std::clamp(delta, -max_delta, max_delta);
    double u_next = u_prev + step;
    double next_s = kinematics_->normalize_angle(cfg.center_angle + u_next);

    // 3. Guaranteed barrier defense: clamp to safe sector
    return kinematics_->clampToSafeSteerSector(leg_index, next_s);
}

std::tuple<Eigen::Vector4d, Eigen::Vector4d> DrivingController::update(
    const Eigen::Vector3d& cmd_vel,
    const Eigen::Vector4d& current_steer_angles,
    const Eigen::Vector4d& current_ecc_angles,
    double dt,
    bool e_stop_active,
    bool is_homing,
    const Eigen::Vector4d& current_steer_vels,
    const Eigen::Vector4d& current_wheel_vels,
    const Eigen::Vector4d& current_ecc_vels,
    const std::array<bool, NUM_LEGS>& contact_valid) {

    // First time initialization to current hardware state
    if (!initialized_) {
        prev_steer_angles_ = current_steer_angles;
        prev_wheel_speeds_.setZero();
        steer_torques_.setZero();
        wheel_torques_.setZero();
        wheel_speed_integrals_.setZero();
        initialized_ = true;
    }

    if (e_stop_active) {
        // E-STOP: Lock steering, stop wheels immediately
        prev_wheel_speeds_.setZero();
        filtered_bank_angle_ = 0.0;
        steer_torques_.setZero();
        wheel_torques_.setZero();
        wheel_speed_integrals_.setZero();
        return {prev_steer_angles_, prev_wheel_speeds_};
    }

    // ================================================================
    // 0. Velocity Saturation Limits
    // ================================================================
    Eigen::Vector3d active_cmd = cmd_vel;
    double v_planar = active_cmd.head<2>().norm();
    if (v_planar > params_.max_linear_vel && v_planar > 1e-4) {
        active_cmd.head<2>() *= (params_.max_linear_vel / v_planar);
    }
    active_cmd.z() = std::clamp(active_cmd.z(), -params_.max_angular_vel, params_.max_angular_vel);

    // ================================================================
    // 1. Compute Bank Angle (centripetal force compensation)
    // ================================================================
    double raw_bank = is_homing ? 0.0 : computeBankAngle(active_cmd);
    // Low-pass filter for smooth transitions
    filtered_bank_angle_ += params_.bank_angle_filter * (raw_bank - filtered_bank_angle_);

    const bool align_to_neutral = is_homing || active_cmd.norm() < 1e-3;
    Eigen::Vector4d opt_steer = Eigen::Vector4d::Zero();
    if (align_to_neutral) {
        prev_swerve_inverted_.fill(false);
    } else {
        const auto targets = kinematics_->computeSteeringTargets(
            active_cmd, prev_steer_angles_, current_ecc_angles);
        for (int i = 0; i < NUM_LEGS; ++i) {
            opt_steer(i) = optimizeSwerveModule(
                i, targets.direct(i), targets.reversed(i), prev_steer_angles_(i));
        }
    }

    // ============================================================
    // 4. Chassis-Level Coherent Drive Scaling (RoboMaster Swerve Principle)
    // Scale driving wheel speeds by cos^2(max_error) so wheels don't fight while rotating into alignment!
    // ============================================================
    double max_steer_error = 0.0;
    for (int i = 0; i < 4; ++i) {
        double err = std::abs(kinematics_->normalize_angle(opt_steer(i) - current_steer_angles(i)));
        if (err > max_steer_error) {
            max_steer_error = err;
        }
    }

    double coherent_drive_scale = 1.0;
    if (params_.enable_coherent_drive_scaling) {
        if (max_steer_error > params_.steer_alignment_deadband) {
            double c = std::max(0.0, std::cos(max_steer_error));
            coherent_drive_scale = c * c;
        }
    }

    // Project traction onto the measured rolling axis and reserve wheel speed
    // for joint motion before scaling all traction requests together.
    Eigen::Vector3d traction_twist = active_cmd;
    if (align_to_neutral) {
        traction_twist.setZero();
    }
    const double compensation_limit =
        std::min(params_.max_rolling_compensation, params_.max_wheel_speed);
    Eigen::Vector4d final_steer;
    Eigen::Vector4d traction;
    Eigen::Vector4d compensation;
    double traction_scale = coherent_drive_scale;
    for (int i = 0; i < NUM_LEGS; ++i) {
        final_steer(i) = enforceForbiddenSectorTransit(
            i, prev_steer_angles_(i), opt_steer(i), params_.max_steer_vel * dt);

        const auto rolling = kinematics_->computeWheelRollingSpeeds(
            i, traction_twist, current_steer_angles(i), current_ecc_angles(i),
            current_steer_vels(i), current_ecc_vels(i));
        traction(i) = rolling.chassis;
        compensation(i) = contact_valid[i] && std::isfinite(rolling.joints)
            ? std::clamp(rolling.joints, -compensation_limit, compensation_limit) : 0.0;

        if (std::abs(traction(i)) > 1e-6) {
            const double headroom = params_.max_wheel_speed -
                std::copysign(1.0, traction(i)) * compensation(i);
            traction_scale = std::min(traction_scale, headroom / std::abs(traction(i)));
        }
    }

    const Eigen::Vector4d target_speed = traction_scale * traction + compensation;
    const double max_speed_delta = params_.max_wheel_accel * dt;
    const Eigen::Vector4d speed_delta = (target_speed - prev_wheel_speeds_)
        .cwiseMax(-max_speed_delta).cwiseMin(max_speed_delta);
    const Eigen::Vector4d final_speed = (prev_wheel_speeds_ + speed_delta)
        .cwiseMax(-params_.max_wheel_speed).cwiseMin(params_.max_wheel_speed);

    // ============================================================
    // 5. Low-Level Actuator Torque Loops (Paper Fig. 8 & Dual-Layer Mode)
    // ============================================================
    for (int i = 0; i < 4; ++i) {
        // Steering angle PD torque
        double e_str = kinematics_->normalize_angle(final_steer(i) - current_steer_angles(i));
        double tau_str = params_.kp_steer * e_str - params_.kd_steer * current_steer_vels(i);
        steer_torques_(i) = std::clamp(tau_str, -params_.max_steer_torque, params_.max_steer_torque);

        // Wheel speed PI torque with anti-windup & traction saturation
        double e_whl = final_speed(i) - current_wheel_vels(i);
        wheel_speed_integrals_(i) += e_whl * dt;
        wheel_speed_integrals_(i) = std::clamp(wheel_speed_integrals_(i), -10.0, 10.0);
        double tau_whl = params_.kp_wheel * e_whl + params_.ki_wheel * wheel_speed_integrals_(i);
        wheel_torques_(i) = std::clamp(tau_whl, -params_.max_wheel_torque, params_.max_wheel_torque);
    }

    // 6. Update state and return
    prev_steer_angles_ = final_steer;
    prev_wheel_speeds_ = final_speed;

    return {final_steer, final_speed};
}

}  // namespace controllers
}  // namespace robot_control
