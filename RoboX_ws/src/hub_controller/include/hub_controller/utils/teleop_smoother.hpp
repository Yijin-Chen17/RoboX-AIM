#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace robot_control {
namespace controllers {

struct TeleopSmootherParams {
    double max_lin_vel     = 1.2;   // m/s (longitudinal velocity ceiling)
    double max_strafe_vel  = 1.0;   // m/s (lateral velocity ceiling)
    double max_ang_vel     = 2.0;   // rad/s (yaw rate ceiling)
    double accel_lin_fwd   = 2.6;   // m/s^2
    double accel_lin_brake = 2.8;   // m/s^2
    double accel_strafe    = 2.2;   // m/s^2
    double accel_angular   = 12.0;  // rad/s^2
    double max_lateral_acc = 3.0;   // m/s^2 (centrifugal envelope |vx * wz| <= max_lateral_acc)
};

/**
 * @brief Asymmetric slew-rate and centrifugal-safety teleop velocity smoother.
 *
 * Ports the continuous velocity smoothing and rollover protection from
 * sim/teleop_commander.py to real-hardware C++ teleoperation.
 */
class TeleopSmoother {
public:
    explicit TeleopSmoother(const TeleopSmootherParams& params = TeleopSmootherParams())
        : params_(params) {}

    void setParams(const TeleopSmootherParams& params) { params_ = params; }

    void reset() {
        cmd_vx_ = 0.0;
        cmd_vy_ = 0.0;
        cmd_wz_ = 0.0;
    }

    Eigen::Vector3d update(double norm_vx, double norm_vy, double norm_wz, double dt, bool brake = false) {
        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.01;
        }

        if (brake) {
            cmd_vx_ = slewRate(cmd_vx_, 0.0, params_.accel_lin_brake, params_.accel_lin_brake, dt);
            cmd_vy_ = slewRate(cmd_vy_, 0.0, params_.accel_strafe * 2.0, params_.accel_strafe * 2.0, dt);
            cmd_wz_ = slewRate(cmd_wz_, 0.0, params_.accel_angular * 2.0, params_.accel_angular * 2.0, dt);
            return Eigen::Vector3d(cmd_vx_, cmd_vy_, cmd_wz_);
        }

        const double raw_vx = std::clamp(norm_vx, -1.0, 1.0) * params_.max_lin_vel;
        const double raw_vy = std::clamp(norm_vy, -1.0, 1.0) * params_.max_strafe_vel;
        double raw_wz       = std::clamp(norm_wz, -1.0, 1.0) * params_.max_ang_vel;

        // Centrifugal protection: |vx * wz| <= max_lateral_acc
        if (std::abs(cmd_vx_) > 0.1 && std::abs(raw_wz) > 0.01) {
            const double max_allowed_wz = params_.max_lateral_acc / std::abs(cmd_vx_);
            raw_wz = std::clamp(raw_wz, -max_allowed_wz, max_allowed_wz);
        }

        cmd_vx_ = slewRate(cmd_vx_, raw_vx, params_.accel_lin_fwd, params_.accel_lin_brake, dt);
        cmd_vy_ = slewRate(cmd_vy_, raw_vy, params_.accel_strafe, params_.accel_strafe * 1.5, dt);
        cmd_wz_ = slewRate(cmd_wz_, raw_wz, params_.accel_angular, params_.accel_angular * 2.0, dt);

        return Eigen::Vector3d(cmd_vx_, cmd_vy_, cmd_wz_);
    }

    Eigen::Vector3d velocity() const { return Eigen::Vector3d(cmd_vx_, cmd_vy_, cmd_wz_); }

private:
    static double slewRate(double current, double target, double accel_lim, double decel_lim, double dt) {
        const double delta = target - current;
        const bool is_braking = (current * delta < 0.0) || (std::abs(target) < std::abs(current));
        const double rate_limit = (is_braking ? decel_lim : accel_lim) * dt;
        return current + std::clamp(delta, -rate_limit, rate_limit);
    }

    TeleopSmootherParams params_;
    double cmd_vx_ = 0.0;
    double cmd_vy_ = 0.0;
    double cmd_wz_ = 0.0;
};

}  // namespace controllers
}  // namespace robot_control
