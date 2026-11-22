#include "robot_control/controllers/balance_controller.hpp"
#include <algorithm>
#include <cmath>

namespace robot_control {
namespace controllers {

namespace {
// Keep the upper vertical boundary on its leg's branch when the measured
// angle has been wrapped to the equivalent angle with the opposite sign.
Eigen::Vector4d alignOutwardEndpoints(Eigen::Vector4d angles) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (std::abs(std::abs(angles(i)) - M_PI) < 1e-12) {
            angles(i) = (i == FL || i == FR) ? M_PI : -M_PI;
        }
    }
    return angles;
}
}  // namespace

BalanceController::BalanceController(const BalanceControllerParams& params,
                                     std::shared_ptr<kinematics::MobedKinematics> kinematics,
                                     const RobotParams& robot_params)
    : params_(params),
      robot_params_(robot_params),
      kinematics_(kinematics),
      srbd_model_(robot_params),
      grf_optimizer_(robot_params)
{
    prev_ecc_angles_.setZero();
}

void BalanceController::setPayloadMass(double mass) {
    robot_params_.total_mass += mass - robot_params_.payload_mass;
    robot_params_.payload_mass = mass;
    srbd_model_.setPayload(mass, robot_params_.payload_com_z_offset);
    grf_optimizer_.setRobotParams(robot_params_);
}

// ============================================================================
// Original position-only interface (backward compatible)
// ============================================================================
Eigen::Vector4d BalanceController::update(
    double target_height,
    double target_roll,
    double target_pitch,
    const Eigen::Vector4d& current_steer_angles,
    const Eigen::Vector4d& current_ecc_angles,
    const Eigen::Matrix3d& R_T,
    double dt,
    bool e_stop_active) {

    if (e_stop_active) {
        // E-STOP: Hold current physical state and reset initialization
        // so it smoothly recovers via the cubic trajectory when released
        initialized_ = false;
        prev_ecc_angles_ = current_ecc_angles;
        return current_ecc_angles;
    }

    // First time initialization to current hardware state
    if (!initialized_) {
        start_ecc_angles_ = current_config_ == PostureConfiguration::OUTWARD
            ? alignOutwardEndpoints(current_ecc_angles) : current_ecc_angles;
        prev_ecc_angles_ = start_ecc_angles_;
        time_since_init_ = 0.0;
        initialized_ = true;
    }

    // 1. Safety Input Clamp (Prevent impossible target commands)
    double safe_roll = std::clamp(target_roll, -params_.max_roll, params_.max_roll);
    double safe_pitch = std::clamp(target_pitch, -params_.max_pitch, params_.max_pitch);

    // 2. Math Kinematics (Exact non-linear projection with posture configuration)
    Eigen::Vector4d raw_ecc_angles = kinematics_->computePostureIK(
        target_height, safe_roll, safe_pitch, current_steer_angles, R_T, current_config_);

    Eigen::Vector4d final_ecc = prev_ecc_angles_;

    for (int i = 0; i < 4; ++i) {
        if (current_config_ != PostureConfiguration::OUTWARD) {
            raw_ecc_angles(i) = std::clamp(raw_ecc_angles(i), params_.min_ecc_angle, params_.max_ecc_angle);
        }
    }

    // 4. Synchronized Smooth Startup Trajectory
    if (time_since_init_ < STARTUP_DURATION) {
        time_since_init_ += dt;
        double progress = std::min(1.0, time_since_init_ / STARTUP_DURATION);

        // Cubic easing: 3p^2 - 2p^3 (Smooth start and stop)
        double smooth_progress = progress * progress * (3.0 - 2.0 * progress);

        for (int i = 0; i < 4; ++i) {
            final_ecc(i) = start_ecc_angles_(i) + (raw_ecc_angles(i) - start_ecc_angles_(i)) * smooth_progress;
        }
    } else {
        // 5. Normal Trajectory Smoothing (Low Pass Filter) for teleop
        for (int i = 0; i < 4; ++i) {
            double smoothed = params_.filter_alpha * raw_ecc_angles(i) +
                              (1.0 - params_.filter_alpha) * prev_ecc_angles_(i);

            // Rate Limiter
            double delta = smoothed - prev_ecc_angles_(i);
            double max_delta = params_.max_ecc_vel * dt;
            delta = std::clamp(delta, -max_delta, max_delta);

            final_ecc(i) = prev_ecc_angles_(i) + delta;
        }
    }

    // 6. Update state and return
    prev_ecc_angles_ = final_ecc;

    return final_ecc;
}

// ============================================================================
// PD Controller for desired body acceleration
// ============================================================================
void BalanceController::computeDesiredAcceleration(
    double target_height, double target_roll, double target_pitch,
    const Eigen::Vector4d& current_ecc_angles,
    const BodyState& body_state,
    Eigen::Vector3d& desired_linear_accel,
    Eigen::Vector3d& desired_angular_accel,
    const Eigen::Vector3d& cmd_vel,
    double dt) const
{
    // ================================================================
    // Reference acceleration generation for SRBD model (Paper Eq 11-12)
    //
    // Linear (Z-axis only for height control):
    //   a_z = kp * (h_target - h_current) + kd * (0 - v_z)
    //
    // Physical ground clearance is computed directly from forward kinematics
    // of the 4 legs: H_i = posture_z_offset + l_ecc * cos(q_ecc,i) + r_wheel.
    // ================================================================

    double h_current = 0.0;
    for (int i = 0; i < 4; ++i) {
        h_current += robot_params_.posture_z_offset + robot_params_.l_ecc * std::cos(current_ecc_angles(i)) + robot_params_.r_wheel;
    }
    h_current /= 4.0;

    double h_error = target_height - h_current;
    double vz = body_state.linear_velocity.z();

    desired_linear_accel = Eigen::Vector3d::Zero();
    desired_linear_accel.z() = std::clamp(
        params_.kp_height * h_error - params_.kd_height * vz, -2.5, 2.5);

    // Active longitudinal and centripetal acceleration feedforward (Paper Eq 11):
    // Preemptively cancels pitch load transfer from d(vx)/dt and roll load transfer from wz * vx
    double ax_cmd = 0.0;
    if (dt > 1e-4) {
        ax_cmd = (cmd_vel.x() - prev_cmd_vel_.x()) / dt;
        ax_cmd = std::clamp(ax_cmd, -3.0, 3.0);
    }
    prev_cmd_vel_ = cmd_vel;

    double ay_cmd = cmd_vel.z() * cmd_vel.x();
    ay_cmd = std::clamp(ay_cmd, -3.0, 3.0);

    // Transform planar acceleration from vehicle heading frame to world frame
    double yaw_curr = body_state.yaw();
    Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(yaw_curr, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d a_planar_heading(ax_cmd, ay_cmd, 0.0);
    desired_linear_accel.head<2>() = (R_yaw * a_planar_heading).head<2>();

    // ================================================================
    // SO(3) Lie Algebra Logarithmic Map (Paper Eq 12):
    //   alpha^d = K_{P,T} * log(R^d * R^T) + K_{D,T} * (omega^d - omega)
    //
    // Eliminates gimbal lock and non-commutative Euler angle coupling
    // across the full 19 deg roll x 23 deg pitch workspace.
    // ================================================================
    // Desired orientation R^d (ZYX Tait-Bryan: keeps current yaw heading, commands target roll & pitch)
    Eigen::Matrix3d R_curr = body_state.orientation.toRotationMatrix();
    Eigen::Matrix3d R_d = (Eigen::AngleAxisd(yaw_curr, Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(target_pitch, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(target_roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

    // Relative rotation error in world frame: R_err = R^d * R^T
    Eigen::Matrix3d R_err = R_d * R_curr.transpose();

    // Matrix logarithm map: e_R = log(R_err) \in so(3)
    double trace = R_err.trace();
    double cos_theta = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
    double theta = std::acos(cos_theta);
    Eigen::Vector3d vee(
        R_err(2, 1) - R_err(1, 2),
        R_err(0, 2) - R_err(2, 0),
        R_err(1, 0) - R_err(0, 1)
    );
    Eigen::Vector3d e_R;
    if (theta < 1e-5) {
        e_R = 0.5 * (1.0 + theta * theta / 6.0) * vee;
    } else {
        e_R = (0.5 * theta / std::sin(theta)) * vee;
    }

    // Express rotation error and angular velocity in robot heading frame (R_z(yaw))
    // so gains kp_roll, kp_pitch apply along vehicle longitudinal and lateral axes
    Eigen::Vector3d e_R_heading = R_yaw.transpose() * e_R;
    Eigen::Vector3d omega_heading = R_yaw.transpose() * body_state.angular_velocity;

    Eigen::Vector3d alpha_heading = Eigen::Vector3d::Zero();
    alpha_heading.x() = params_.kp_roll  * e_R_heading.x() - params_.kd_roll  * omega_heading.x();
    alpha_heading.y() = params_.kp_pitch * e_R_heading.y() - params_.kd_pitch * omega_heading.y();

    // Transform desired angular acceleration back to world frame for SRBD
    desired_angular_accel = R_yaw * alpha_heading.cwiseMax(-10.0).cwiseMin(10.0);
}

double BalanceController::computeAdaptiveNominalHeight(
    double target_height, double pitch_slope, double roll_slope, double commanded_roll, double dt) const
{
    // Required vertical difference to maintain level chassis and accommodate active banking:
    // Take the maximum of terrain roll slope and commanded active bank roll
    double effective_roll = std::max(std::abs(std::sin(roll_slope)), std::abs(std::sin(commanded_roll)));
    double delta_z_req = robot_params_.length_x * std::abs(std::sin(pitch_slope))
                       + robot_params_.width_y  * effective_roll;

    // Slope & Bank Headroom Management (Phase 2.7):
    // Rather than subtracting delta_z_req directly (which caused a double-subtraction
    // when computePostureIK also subtracts delta_z_req via R_T), we define the admissible
    // envelope for the chassis reference height:
    //   safe_ceiling = max_safe_leg_length (0.205m) - delta_z_req (to avoid singularity at q->0)
    //   safe_floor   = min_safe_leg_length (0.115m) + delta_z_req (to keep crouch |q| <= 115 deg)
    double safe_ceiling = 0.205 - delta_z_req;
    double safe_floor   = 0.115 + delta_z_req;

    const double h_adapt_raw = safe_floor <= safe_ceiling
        ? std::clamp(target_height, safe_floor, safe_ceiling)
        : 0.5 * (safe_floor + safe_ceiling);

    // A slow common height reference lets the individual wheels absorb short bumps.
    const double alpha_h = dt / (params_.height_filter_time + dt);
    const double h_adapt = prev_h_adapt_ + alpha_h * (h_adapt_raw - prev_h_adapt_);
    prev_h_adapt_ = h_adapt;
    return h_adapt;
}

// ============================================================================
// Hybrid force-position control (Plan 3 upgrade)
// ============================================================================
ControlOutput BalanceController::updateHybrid(
    double target_height,
    double target_roll,
    double target_pitch,
    const Eigen::Vector4d& current_steer_angles,
    const Eigen::Vector4d& current_ecc_angles,
    const BodyState& body_state,
    const std::array<Eigen::Vector3d, NUM_LEGS>& contact_points,
    const std::array<bool, NUM_LEGS>& contact_valid,
    const TerrainState& terrain_state,
    double dt,
    bool e_stop_active,
    const Eigen::Vector3d& cmd_vel,
    bool outward_half_circle,
    const estimation::WheelSupport* wheel_support)
{
    ControlOutput output;

    if (e_stop_active) {
        reset();
        prev_ecc_angles_ = current_ecc_angles;
        output.ecc_angles = current_ecc_angles;
        output.ecc_torques = Eigen::Vector4d::Zero();
        output.ecc_mode = ControlOutput::EccMode::POSITION;
        return output;
    }

    // ================================================================
    // 1. Slope Headroom Management & Fast Residual Attitude PD (Phase 2)
    // ================================================================
    double safe_roll  = std::clamp(target_roll,  -params_.max_roll,  params_.max_roll);
    double safe_pitch = std::clamp(target_pitch, -params_.max_pitch, params_.max_pitch);

    // Compute dynamic adaptive height to preserve downhill leg stroke headroom and bank angle stroke
    double h_adapt = computeAdaptiveNominalHeight(
        target_height, terrain_state.pitch_slope, terrain_state.roll_slope, safe_roll, dt);

    bool is_in_startup = !initialized_ || (time_since_init_ < STARTUP_DURATION);

    // Fast IMU Attitude Residual Feedback:
    // Terrain slope feedforward is already handled geometrically via terrain_state.rotation in computePostureIK.
    // The attitude PD loop only eliminates dynamic disturbances and residual unmodeled error.
    // Suppress attitude corrections during initial startup trajectory so uncalibrated/transient IMU states don't tilt the standing posture.
    double roll_corr  = 0.0;
    double pitch_corr = 0.0;
    if (!is_in_startup) {
        double roll_current  = body_state.roll();
        double pitch_current = body_state.pitch();
        // The estimator reports world-frame angular velocity. Apply roll and
        // pitch damping in the same heading frame used by the balance PD.
        const Eigen::Vector3d omega_heading =
            Eigen::AngleAxisd(-body_state.yaw(), Eigen::Vector3d::UnitZ())
            * body_state.angular_velocity;
        double omega_x = omega_heading.x();
        double omega_y = omega_heading.y();

        double roll_err  = roll_current - safe_roll;
        double pitch_err = pitch_current - safe_pitch;

        // Apply deadband on attitude errors to eliminate gyro noise amplification on flat ground
        if (std::abs(roll_err) < params_.attitude_deadband) {
            roll_err = 0.0;
        } else {
            roll_err -= std::copysign(params_.attitude_deadband, roll_err);
        }
        if (std::abs(pitch_err) < params_.attitude_deadband) {
            pitch_err = 0.0;
        } else {
            pitch_err -= std::copysign(params_.attitude_deadband, pitch_err);
        }

        omega_x = std::clamp(omega_x, -1.5, 1.5);
        omega_y = std::clamp(omega_y, -1.5, 1.5);

        // Command residual corrections (expanded to ±0.18 rad ≈ ±10.3 deg) to ensure adequate leveling authority
        roll_corr  = -(params_.kp_attitude_roll  * roll_err  + params_.kd_attitude_roll  * omega_x);
        pitch_corr = -(params_.kp_attitude_pitch * pitch_err + params_.kd_attitude_pitch * omega_y);
        roll_corr  = std::clamp(roll_corr,  -0.18, 0.18);
        pitch_corr = std::clamp(pitch_corr, -0.18, 0.18);
    }

    double active_ik_roll  = std::clamp(safe_roll  + roll_corr,  -params_.max_roll,  params_.max_roll);
    double active_ik_pitch = std::clamp(safe_pitch + pitch_corr, -params_.max_pitch, params_.max_pitch);
    const bool use_outward_sector = outward_half_circle && current_config_ == PostureConfiguration::OUTWARD;

    Eigen::Vector4d ik_target = kinematics_->computePostureIK(
        h_adapt, active_ik_roll, active_ik_pitch, current_steer_angles,
        terrain_state.rotation, current_config_, use_outward_sector);

    for (int i = 0; i < 4; ++i) {
        if (use_outward_sector) continue;
        ik_target(i) = std::clamp(ik_target(i), params_.min_ecc_angle, params_.max_ecc_angle);
        // Singularity avoidance: enforce minimum bend of 15 deg (0.26 rad) along the configuration sign branch
        bool is_front = (i == FL || i == FR);
        bool target_positive = true;
        if (current_config_ == PostureConfiguration::OUTWARD) {
            target_positive = is_front;
        } else if (current_config_ == PostureConfiguration::INWARD) {
            target_positive = !is_front;
        } else if (current_config_ == PostureConfiguration::SPIN) {
            target_positive = (i == FL || i == RL);
        } else if (current_config_ == PostureConfiguration::STRAFE_OUTWARD) {
            target_positive = !is_front;
        }

        if (target_positive) {
            ik_target(i) = std::max(0.26, ik_target(i));
        } else {
            ik_target(i) = std::min(-0.26, ik_target(i));
        }
    }

    // Smooth the nominal inverse-kinematics position.
    const Eigen::Vector4d tracking_angles = use_outward_sector
        ? alignOutwardEndpoints(current_ecc_angles) : current_ecc_angles;
    if (!initialized_) {
        start_ecc_angles_ = tracking_angles;
        prev_ecc_angles_ = start_ecc_angles_;
        time_since_init_ = 0.0;
        initialized_ = true;
    }

    Eigen::Vector4d smoothed_ik = prev_ecc_angles_;

    if (time_since_init_ < STARTUP_DURATION) {
        time_since_init_ += dt;
        double progress = std::min(1.0, time_since_init_ / STARTUP_DURATION);
        double smooth_progress = progress * progress * (3.0 - 2.0 * progress);

        for (int i = 0; i < 4; ++i) {
            smoothed_ik(i) = start_ecc_angles_(i) +
                (ik_target(i) - start_ecc_angles_(i)) * smooth_progress;
        }
    } else {
        for (int i = 0; i < 4; ++i) {
            double smoothed = params_.filter_alpha * ik_target(i) +
                              (1.0 - params_.filter_alpha) * prev_ecc_angles_(i);
            double delta = smoothed - prev_ecc_angles_(i);
            double max_delta = params_.max_ecc_vel * dt;
            delta = std::clamp(delta, -max_delta, max_delta);
            smoothed_ik(i) = prev_ecc_angles_(i) + delta;
        }
    }

    prev_ecc_angles_ = smoothed_ik;

    output.ecc_angles = smoothed_ik;
    if (!wheel_support || is_in_startup) {
        prev_compliance_shift_.setZero();
        prev_support_torque_.setZero();
        grf_optimizer_.reset();
        prev_cmd_vel_ = cmd_vel;
        return output;
    }

    Eigen::Vector3d desired_lin_accel, desired_ang_accel;
    computeDesiredAcceleration(h_adapt, safe_roll, safe_pitch, current_ecc_angles,
                               body_state, desired_lin_accel, desired_ang_accel, cmd_vel, dt);

    Eigen::Vector4d force_limits;
    Eigen::Vector4d authority;
    for (int i = 0; i < NUM_LEGS; ++i) {
        const double lever = std::abs(grf_optimizer_.computeVerticalJacobian(
            i, current_ecc_angles(i), current_steer_angles(i), safe_roll, safe_pitch));
        authority(i) = std::clamp((lever - 0.012) / 0.024, 0.0, 1.0);
        const double load = wheel_support->normal_load(i);
        const double load_weight = std::clamp(load / 35.0, 0.0, 1.0);
        force_limits(i) = params_.support_torque_limit * authority(i) * load_weight
                          / std::max(lever, 1e-6);
    }
    srbd_model_.setState(body_state, contact_points, contact_valid);
    const Eigen::Vector4d optimal_grf = grf_optimizer_.computeOptimalGRF(
        srbd_model_, desired_lin_accel, desired_ang_accel, force_limits);
    const Eigen::Vector4d support_torque = grf_optimizer_.grfToEccTorque(
        optimal_grf, current_ecc_angles, current_steer_angles, safe_roll, safe_pitch);

    const double nominal_load = robot_params_.total_mass * robot_params_.gravity / NUM_LEGS;
    for (int i = 0; i < NUM_LEGS; ++i) {
        // Compare each wheel with its nominal share of weight, independent of
        // the body controller's instantaneous demand for force redistribution.
        const double load = wheel_support->normal_load(i);
        const double load_error = load >= 0.0 ? load - nominal_load : 0.0;
        const double excess_load = std::copysign(
            std::max(0.0, std::abs(load_error) - params_.load_deadband), load_error);
        const double desired_shift = std::clamp(params_.wheel_compliance * excess_load,
            -params_.max_wheel_adjustment, params_.max_wheel_adjustment);
        const double shift_step = params_.max_adjustment_speed * dt;
        prev_compliance_shift_(i) += std::clamp(
            desired_shift - prev_compliance_shift_(i), -shift_step, shift_step);

        const double target_lever = robot_params_.l_ecc * std::sin(smoothed_ik(i));
        const double angle_shift = prev_compliance_shift_(i) * target_lever /
                                   (target_lever * target_lever + 0.018 * 0.018);
        const bool front = i == FL || i == FR;
        output.ecc_angles(i) = use_outward_sector
            ? std::clamp(smoothed_ik(i) + angle_shift, front ? 0.0 : -M_PI, front ? M_PI : 0.0)
            : std::clamp(smoothed_ik(i) + angle_shift, params_.min_ecc_angle, params_.max_ecc_angle);

        if (!contact_valid[i] || load < 0.0 || authority(i) == 0.0) {
            prev_support_torque_(i) = 0.0;
        } else {
            const double step = params_.support_torque_rate * dt;
            prev_support_torque_(i) += std::clamp(
                support_torque(i) - prev_support_torque_(i), -step, step);
        }
    }

    output.ecc_torques = prev_support_torque_;
    output.ecc_mode = ControlOutput::EccMode::HYBRID;
    return output;
}

}  // namespace controllers
}  // namespace robot_control
