#pragma once

#include <Eigen/Dense>
#include <memory>
#include "robot_control/kinematics/mobed_kinematics.hpp"
#include "robot_control/core/robot_params.hpp"
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/dynamics/srbd_model.hpp"
#include "robot_control/dynamics/grf_optimizer.hpp"
#include "robot_control/estimation/wheel_support.hpp"

namespace robot_control {
namespace controllers {

struct BalanceControllerParams {
    double min_ecc_angle = -1.745; // ~ -100 deg
    double max_ecc_angle = 1.745;  // ~ 100 deg

    double max_roll = 0.331;       // ~ 19 deg
    double max_pitch = 0.401;      // ~ 23 deg

    double filter_alpha = 0.25;    // LPF coefficient for normal operation (tau = 80ms)
    double max_ecc_vel = 2.5;      // rad/s

    // ---- Fast Attitude Closed-Loop PD gains (direct IMU compensation) ----
    double kp_attitude_roll  = 1.0;   // dimensionless scale on roll error [rad -> rad]
    double kd_attitude_roll  = 0.08;  // damping scale on gyro wx [rad/s -> rad]
    double kp_attitude_pitch = 1.0;   // dimensionless scale on pitch error [rad -> rad]
    double kd_attitude_pitch = 0.08;  // damping scale on gyro wy [rad/s -> rad]
    double attitude_deadband  = 0.008; // ~0.46 deg deadband on attitude errors to eliminate gyro noise amplification

    // Body acceleration gains (position s^-2, velocity s^-1).
    double kp_height = 28.0;
    double kd_height = 5.0;
    double kp_roll   = 18.0;
    double kd_roll   = 3.0;
    double kp_pitch  = 18.0;
    double kd_pitch  = 3.0;

    // Regular driving uses independent, bounded adjustments at each wheel.
    double height_filter_time = 0.35;       // s
    double load_deadband = 18.0;            // N
    double wheel_compliance = 0.00014;      // m/N
    double max_wheel_adjustment = 0.014;    // m
    double max_adjustment_speed = 0.12;     // m/s
    double support_torque_limit = 7.0;      // Nm, reserves torque for the MIT feedback loop
    double support_torque_rate = 45.0;      // Nm/s
};

class BalanceController {
public:
    BalanceController(const BalanceControllerParams& params,
                      std::shared_ptr<kinematics::MobedKinematics> kinematics,
                      const RobotParams& robot_params = RobotParams());

    /**
     * @brief Update the balance controller step (position-only mode, backward compatible).
     *
     * This is the original interface that outputs pure position commands.
     * Used when the system is still in position-control mode.
     *
     * @param target_height Target base height
     * @param target_roll Target roll angle (includes bank angle from driving)
     * @param target_pitch Target pitch angle
     * @param current_steer_angles Required for exact posture projection
     * @param current_ecc_angles Required for initialization and E-Stop freezing
     * @param dt Time step
     * @param e_stop_active Emergency stop flag
     * @return Eigen::Vector4d Target eccentric angles (position command)
     */
    Eigen::Vector4d update(
        double target_height,
        double target_roll,
        double target_pitch,
        const Eigen::Vector4d& current_steer_angles,
        const Eigen::Vector4d& current_ecc_angles,
        const Eigen::Matrix3d& R_T,
        double dt,
        bool e_stop_active = false);

    ControlOutput updateHybrid(
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
        bool e_stop_active = false,
        const Eigen::Vector3d& cmd_vel = Eigen::Vector3d::Zero(),
        bool outward_half_circle = true,
        const estimation::WheelSupport* wheel_support = nullptr);

    void reset() {
        initialized_ = false;
        prev_cmd_vel_.setZero();
        prev_compliance_shift_.setZero();
        prev_support_torque_.setZero();
        grf_optimizer_.reset();
        prev_h_adapt_ = 0.205;
        time_since_init_ = 0.0;
    }
    void syncCurrentAngles(const Eigen::Vector4d& current_ecc_angles) { prev_ecc_angles_ = current_ecc_angles; }

    void setPayloadMass(double mass);
    void setPostureConfiguration(PostureConfiguration config) { current_config_ = config; }
    PostureConfiguration getPostureConfiguration() const { return current_config_; }

    dynamics::GrfOptimizer& getGrfOptimizer() { return grf_optimizer_; }
    const dynamics::GrfOptimizer& getGrfOptimizer() const { return grf_optimizer_; }

    /**
     * @brief Compute slope- and bank-adaptive nominal height (Headroom Management).
     *
     * Automatically lowers nominal chassis height on slopes or during sharp banked turns
     * so that downhill/outside legs do not exceed max_height (0.22m), enabling full
     * bilateral leveling authority and compliance without singularity.
     */
    double computeAdaptiveNominalHeight(
        double target_height, double pitch_slope, double roll_slope,
        double commanded_roll, double dt) const;

private:
    BalanceControllerParams params_;
    RobotParams robot_params_;
    std::shared_ptr<kinematics::MobedKinematics> kinematics_;
    PostureConfiguration current_config_ = PostureConfiguration::OUTWARD;

    // Dynamics modules
    dynamics::SrbdModel srbd_model_;
    dynamics::GrfOptimizer grf_optimizer_;

    Eigen::Vector4d prev_ecc_angles_;
    Eigen::Vector4d start_ecc_angles_;
    Eigen::Vector4d prev_compliance_shift_ = Eigen::Vector4d::Zero();  // m, positive raises the wheel
    Eigen::Vector4d prev_support_torque_ = Eigen::Vector4d::Zero();
    mutable double prev_h_adapt_ = 0.205;    // IIR filter state for adaptive height smoothing (nominal 0.205m)
    mutable Eigen::Vector3d prev_cmd_vel_ = Eigen::Vector3d::Zero();
    bool initialized_ = false;
    double time_since_init_ = 0.0;
    const double STARTUP_DURATION = 3.0; // 3 seconds to smoothly stand up

    /**
     * @brief Compute desired body acceleration from PD control.
     *
     * Generates the reference acceleration for the SRBD model based on
     * the tracking error between target and estimated body state.
     *
     * @param target_height Target height
     * @param target_roll Target roll
     * @param target_pitch Target pitch
     * @param body_state Current estimated body state
     * @param[out] desired_linear_accel Linear acceleration command (world frame)
     * @param[out] desired_angular_accel Angular acceleration command (world frame)
     * @param cmd_vel Commanded chassis velocity for inertial load transfer feedforward
     * @param dt Time step
     */
    void computeDesiredAcceleration(
        double target_height, double target_roll, double target_pitch,
        const Eigen::Vector4d& current_ecc_angles,
        const BodyState& body_state,
        Eigen::Vector3d& desired_linear_accel,
        Eigen::Vector3d& desired_angular_accel,
        const Eigen::Vector3d& cmd_vel = Eigen::Vector3d::Zero(),
        double dt = 0.01) const;
};

}  // namespace controllers
}  // namespace robot_control
