#include "robot_control/estimation/contact_detector.hpp"
#include <cmath>
#include <algorithm>

namespace robot_control {
namespace estimation {

ContactDetector::ContactDetector(const RobotParams& params)
    : params_(params)
{
    reset();
}

void ContactDetector::reset() {
    impact_flags_.fill(false);
    leg_impact_cycles_.fill(0);
    wheel_ext_torques_.setZero();
    prev_wheel_velocities_.setZero();
    shock_window_timer_.fill(0.0);
    sustain_timer_.fill(0.0);
    startup_timer_ = 0.0;
    front_evidence_score_ = 0.0;
    front_evidence_sustain_timer_ = 0.0;
    prev_cmd_vel_ = 0.0;
    prev_imu_gyro_y_ = 0.0;
    prev_imu_accel_x_ = 0.0;
    prev_chassis_vel_ = 0.0;
    pitch_accel_ = 0.0;
    accel_jerk_x_ = 0.0;
    ecc_torque_rates_ = Eigen::Vector4d::Zero();
    wheel_torque_rates_ = Eigen::Vector4d::Zero();
    prev_ecc_efforts_ = Eigen::Vector4d::Zero();
    prev_wheel_efforts_ = Eigen::Vector4d::Zero();
    ecc_effort_baseline_ = Eigen::Vector4d::Zero();
    delta_tau_ecc_ = Eigen::Vector4d::Zero();
    momentum_residual_ = 0.0;
    momentum_observer_z_ = 0.0;
    rear_impact_confirm_cycles_ = 0;
    impact_confirm_cycles_ = 0;
    initialized_ = false;

    for (int i = 0; i < NUM_LEGS; ++i) {
        ecc_effort_history_[i].clear();
        wheel_effort_history_[i].clear();
    }
}

double ContactDetector::computeSmoothedRate(
    std::deque<double>& history, double new_val, double dt) const
{
    history.push_back(new_val);

    // Maintain window size
    while (static_cast<int>(history.size()) > WINDOW_SIZE) {
        history.pop_front();
    }

    if (history.size() < 2 || dt <= 0.0) {
        return 0.0;
    }

    double oldest = history.front();
    double newest = history.back();
    double window_dt = dt * static_cast<double>(history.size() - 1);

    if (window_dt <= 0.0) return 0.0;

    return (newest - oldest) / window_dt;
}

void ContactDetector::update(
    const Eigen::Vector4d& ecc_efforts,
    const Eigen::Vector4d& wheel_efforts,
    const Eigen::Vector4d& wheel_velocities,
    double chassis_forward_vel,
    double cmd_forward_vel,
    double imu_accel_x,
    const ClimbGeometry& geometry,
    double dt,
    double cmd_lateral_vel,
    double cmd_yaw_rate,
    double imu_gyro_y)
{
    if (dt <= 0.0 || dt > 0.5) return;

    // First call: just store the initial values
    if (!initialized_) {
        prev_ecc_efforts_ = ecc_efforts;
        prev_wheel_efforts_ = wheel_efforts;
        prev_wheel_velocities_ = wheel_velocities;
        wheel_ext_torques_.setZero();
        leg_impact_cycles_.fill(0);
        prev_cmd_vel_ = cmd_forward_vel;
        prev_imu_gyro_y_ = imu_gyro_y;
        prev_imu_accel_x_ = imu_accel_x;
        prev_chassis_vel_ = chassis_forward_vel;
        pitch_accel_ = 0.0;
        accel_jerk_x_ = 0.0;
        ecc_effort_baseline_ = ecc_efforts;
        momentum_observer_z_ = params_.total_mass * chassis_forward_vel;
        momentum_residual_ = 0.0;
        initialized_ = true;
        startup_timer_ = 0.0;
        front_evidence_score_ = 0.0;
        front_evidence_sustain_timer_ = 0.0;
        return;
    }

    // Clear impact flags from previous cycle
    impact_flags_.fill(false);

    // Compute teleop command acceleration rate (for tracking)
    double cmd_accel = (cmd_forward_vel - prev_cmd_vel_) / dt;
    prev_cmd_vel_ = cmd_forward_vel;

    // Forward command gate: require clear straight forward driving intent (>= 0.10 m/s)
    // Suppress front collision detection when steering/rotating or strafing sideways
    bool pure_forward_intent = (cmd_forward_vel >= 0.10) &&
                               (std::abs(cmd_lateral_vel) < 0.08) &&
                               (std::abs(cmd_yaw_rate) < 0.25);
    bool forward_cmd_active = pure_forward_intent;

    // 1. Generalized Momentum Observer (GMO) & Per-Wheel External Torque Decoupling
    // Decouples wheel motor acceleration inertia: tau_ext = |tau_meas| - J_w * |d_omega/dt|
    Eigen::Vector4d wheel_accels = Eigen::Vector4d::Zero();
    for (int i = 0; i < NUM_LEGS; ++i) {
        double dw = (wheel_velocities(i) - prev_wheel_velocities_(i)) / dt;
        wheel_accels(i) = dw;
        double inertia_torque = WHEEL_INERTIA_J * std::abs(dw);
        wheel_ext_torques_(i) = std::max(0.0, std::abs(wheel_efforts(i)) - inertia_torque);
    }
    prev_wheel_velocities_ = wheel_velocities;

    // Startup window tracking to eliminate false alarms during initial start from stand-still
    if (forward_cmd_active) {
        startup_timer_ += dt;
    } else {
        startup_timer_ = 0.0;
    }

    // Fast startup blanking window (only 50ms):
    // Inhibit impact detection only during the first 50ms to allow derivative filters to initialize.
    if (startup_timer_ < 0.05) {
        front_evidence_score_ = 0.0;
        front_evidence_sustain_timer_ = 0.0;
        impact_confirm_cycles_ = 0;
        impact_flags_.fill(false);
        prev_ecc_efforts_ = ecc_efforts;
        prev_wheel_efforts_ = wheel_efforts;
        prev_imu_gyro_y_ = imu_gyro_y;
        prev_imu_accel_x_ = imu_accel_x;
        prev_chassis_vel_ = chassis_forward_vel;
        return;
    }

    // Scheme 1: Kinematic IMU shock & jerk derivatives
    pitch_accel_ = (imu_gyro_y - prev_imu_gyro_y_) / dt;
    prev_imu_gyro_y_ = imu_gyro_y;

    accel_jerk_x_ = (imu_accel_x - prev_imu_accel_x_) / dt;
    prev_imu_accel_x_ = imu_accel_x;

    // Update derivatives, per-wheel torque rates, and eccentric baseline
    for (int i = 0; i < NUM_LEGS; ++i) {
        ecc_torque_rates_(i) = computeSmoothedRate(
            ecc_effort_history_[i], ecc_efforts(i), dt);
        wheel_torque_rates_(i) = computeSmoothedRate(
            wheel_effort_history_[i], wheel_efforts(i), dt);

        delta_tau_ecc_(i) = ecc_efforts(i) - ecc_effort_baseline_(i);
        double alpha_ecc = std::clamp(dt / (dt + 0.40), 0.0, 1.0);
        ecc_effort_baseline_(i) += alpha_ecc * (ecc_efforts(i) - ecc_effort_baseline_(i));
    }

    // Update Generalized Momentum Observer:
    // p = M * vx
    // \dot{z} = F_drive + r
    // r = K_O * (p - z)
    double total_drive_force = 0.0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        total_drive_force += wheel_efforts(i) / params_.r_wheel;
    }
    double p_momentum = params_.total_mass * chassis_forward_vel;
    if (forward_cmd_active) {
        double k_obs = 15.0; // Observer gain (1/s)
        momentum_observer_z_ += (total_drive_force + momentum_residual_) * dt;
        momentum_residual_ = k_obs * (p_momentum - momentum_observer_z_);
    } else {
        momentum_observer_z_ = p_momentum;
        momentum_residual_ = 0.0;
    }

    // =========================================================================
    // Scheme 1 (Refined): Stator-Decoupled & Sign-Locked Fast Collision Detector
    // =========================================================================

    // 1. Forward Acceleration & Speed Growth Veto (Hard Kinematic Gate)
    // A physical front obstacle collision is strictly an ARREST of forward motion (deceleration).
    // If the chassis is actively accelerating forward (a_x > 0) or speed is actively growing,
    // the robot is driving on open ground and CANNOT be a collision.
    double chassis_accel_kinematic = (chassis_forward_vel - prev_chassis_vel_) / dt;
    prev_chassis_vel_ = chassis_forward_vel;

    bool chassis_accelerating = (imu_accel_x > 0.15) || (chassis_accel_kinematic > 0.10);
    bool speed_actively_growing = (chassis_forward_vel > 0.05) && (chassis_accel_kinematic > 0.05);
    bool active_launch_or_accel = chassis_accelerating || speed_actively_growing;

    // 2. Physical Deceleration / Arrest Shock
    // Real collision produces negative deceleration shock or negative momentum residual:
    bool decel_shock = (imu_accel_x < -0.30) || (accel_jerk_x_ < -6.0) || (momentum_residual_ < -3.5);
    bool chassis_halted = (chassis_forward_vel < 0.08);

    // Collision kinematics: Must be decelerating or completely stopped against an obstacle,
    // and CANNOT be actively accelerating forward on open ground.
    bool collision_kinematics_valid = (decel_shock || chassis_halted) && !active_launch_or_accel;

    // 3. Structural Shock: Eccentric Joint Reaction Moment Decoupled from Drive Motor
    // The drive motor stator is mounted on the eccentric knuckle. Drive torque directly induces
    // an internal reaction torque ~ wheel_effort on the eccentric joint.
    // We decouple this internal motor reaction to extract true external ground/obstacle impact:
    double ext_ecc_fl = std::max(0.0, std::abs(delta_tau_ecc_(FL)) - 0.85 * std::abs(wheel_efforts(FL)));
    double ext_ecc_fr = std::max(0.0, std::abs(delta_tau_ecc_(FR)) - 0.85 * std::abs(wheel_efforts(FR)));
    double rate_ecc_fl = std::abs(ecc_torque_rates_(FL));
    double rate_ecc_fr = std::abs(ecc_torque_rates_(FR));

    bool front_ecc_shock = ((ext_ecc_fl > 0.70 || ext_ecc_fr > 0.70) ||
                            (rate_ecc_fl > 12.0 || rate_ecc_fr > 12.0)) &&
                           collision_kinematics_valid;

    // 4. Inertial Shock: Deceleration Shock & Directional Pitch Diving
    // When hitting an obstacle from the front, the chassis decelerates and dives/pitches forward.
    // Normal acceleration squats back (pitch up). We require deceleration shock to validate pitch motion:
    bool imu_impact_shock = decel_shock && (std::abs(pitch_accel_) > 2.0 || imu_accel_x < -0.50);

    // 5. Drive Wheel Resistance: True Obstacle Stall vs Wall-Slip (Excluding Standstill Spin-Up)
    const double wheel_resist_thresh = 1.30;
    bool fl_effort_high = (std::abs(wheel_efforts(FL)) >= wheel_resist_thresh);
    bool fr_effort_high = (std::abs(wheel_efforts(FR)) >= wheel_resist_thresh);

    // Exclude normal startup spin-up: if wheels are accelerating rapidly from rest, they are launching
    bool fl_spin_up = (wheel_accels(FL) > 15.0);
    bool fr_spin_up = (wheel_accels(FR) > 15.0);

    // Case A: Wheel is physically blocked/stalled against obstacle
    bool fl_stalled = fl_effort_high && (std::abs(wheel_velocities(FL)) < 1.0) && !fl_spin_up && collision_kinematics_valid;
    bool fr_stalled = fr_effort_high && (std::abs(wheel_velocities(FR)) < 1.0) && !fr_spin_up && collision_kinematics_valid;

    // Case B: Wheel is slipping against vertical wall face
    bool chassis_blocked = (chassis_forward_vel < 0.15) && !active_launch_or_accel;
    bool fl_wall_slip = fl_effort_high && chassis_blocked && (wheel_velocities(FL) * params_.r_wheel > 0.15);
    bool fr_wall_slip = fr_effort_high && chassis_blocked && (wheel_velocities(FR) * params_.r_wheel > 0.15);

    bool fl_blocked = fl_stalled || fl_wall_slip;
    bool fr_blocked = fr_stalled || fr_wall_slip;
    bool front_any_blocked = fl_blocked || fr_blocked;
    bool front_both_blocked = fl_blocked && fr_blocked;

    // 6. Geometric Wall Climbing (Height Delta & Nose-up Pitch)
    bool geometric_wall_impact = geometry.front_wall_climbing && front_any_blocked;

    // 7. Multi-Modal Collision Trigger Decision
    // Condition 1: Structural strike (decoupled eccentric shock + wheel resistance)
    bool structural_strike = front_ecc_shock && front_any_blocked;
    // Condition 2: Inertial strike (IMU deceleration shock + wheel resistance)
    bool inertial_strike = imu_impact_shock && front_any_blocked;
    // Condition 3: Bilateral wheel block (both front wheels resisted/slipping with chassis halted)
    bool dual_wheel_block = front_both_blocked && collision_kinematics_valid;

    bool front_obstacle_hit = forward_cmd_active &&
                              (structural_strike || inertial_strike || dual_wheel_block || geometric_wall_impact);

    // 8. Fast 20ms Temporal Persistence Confirmation
    // Requires 2 consecutive cycles (20 ms at 100 Hz) to eliminate single-step numerical glitches
    if (front_obstacle_hit) {
        impact_confirm_cycles_++;
        front_evidence_score_ = 1.0;
        if (impact_confirm_cycles_ >= 2) {
            impact_flags_[FL] = true;
            impact_flags_[FR] = true;
        }
    } else {
        impact_confirm_cycles_ = 0;
        front_evidence_score_ = 0.0;
        front_evidence_sustain_timer_ = 0.0;
    }

    // 6. Rear wall contact confirmation (Dual-Track Arrival Observer: Continuous high-speed + Low-speed/Static bump)
    // Strictly requires front wheels to already be elevated on curb top and robot to have advanced across curb
    // Do not gate rear impact on geometry.travel_distance: that odometry was
    // previously accumulated only after front_edge_crested, creating a
    // circular dependency that could make rear detection impossible.
    bool front_on_plateau = geometry.front_platform_contact ||
                            geometry.front_edge_crested ||
                            (geometry.delta_height > 0.030);
    if (forward_cmd_active && front_on_plateau) {
        double tau_rear_max = std::max(std::abs(wheel_efforts(RL)), std::abs(wheel_efforts(RR)));
        bool rear_wheel_resisting = (tau_rear_max > 2.0);

        bool rear_reaction_spike = (std::abs(delta_tau_ecc_(RL)) > 1.8 || std::abs(delta_tau_ecc_(RR)) > 1.8) ||
                                   (std::abs(ecc_torque_rates_(RL)) > 12.0 || std::abs(ecc_torque_rates_(RR)) > 12.0);

        bool continuous_arrival = geometry.rear_platform_contact ||
                                  (geometry.rear_slip_ratio > 0.65 && rear_wheel_resisting) ||
                                  (chassis_halted && rear_wheel_resisting);

        bool momentum_arrested = (momentum_residual_ < -10.0) || (chassis_forward_vel < 0.06);
        bool rear_stalled = (std::abs(wheel_velocities(RL)) < params_.wheel_stall_vel_threshold ||
                             std::abs(wheel_velocities(RR)) < params_.wheel_stall_vel_threshold) &&
                            rear_wheel_resisting;
        bool static_bump_arrival = (rear_reaction_spike && (momentum_arrested || chassis_halted)) ||
                                   (rear_wheel_resisting && (momentum_arrested || chassis_halted)) ||
                                   rear_stalled ||
                                   (tau_rear_max > 3.2);

        bool contact_evidence = continuous_arrival || static_bump_arrival;

        if (contact_evidence) {
            rear_impact_confirm_cycles_++;
            if (rear_impact_confirm_cycles_ >= REQUIRED_REAR_CONFIRM_CYCLES) {
                impact_flags_[RL] = true;
                impact_flags_[RR] = true;
            }
        } else {
            rear_impact_confirm_cycles_ = std::max(0, rear_impact_confirm_cycles_ - 1);
        }
    } else {
        rear_impact_confirm_cycles_ = 0;
    }

    // Store for next cycle
    prev_ecc_efforts_ = ecc_efforts;
    prev_wheel_efforts_ = wheel_efforts;
}

bool ContactDetector::frontImpactDetected() const {
    return impact_flags_[FL] || impact_flags_[FR];
}

bool ContactDetector::rearImpactDetected() const {
    return impact_flags_[RL] || impact_flags_[RR];
}

}  // namespace estimation
}  // namespace robot_control
