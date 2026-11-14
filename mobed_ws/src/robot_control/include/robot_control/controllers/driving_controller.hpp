#pragma once

#include <Eigen/Dense>
#include <array>
#include <tuple>
#include <memory>
#include "robot_control/kinematics/mobed_kinematics.hpp"
#include "robot_control/core/robot_params.hpp"

namespace robot_control {
namespace controllers {

struct DrivingControllerParams {
    double max_wheel_accel = 20.0; // rad/s^2 (keeps 4*2.2 Nm MF9010 V2 17T well inside traction limit)
    double max_steer_vel = 12.0;   // rad/s (within DM-J4310P-2EC 24V rated 120 rpm = 12.57 rad/s)

    // ---- Swerve Steering Optimization & Coherent Drive Gating ----
    // Enables coherent whole-body velocity scaling to preserve instantaneous ICR during steering transitions
    bool enable_coherent_drive_scaling = true;
    // Maximum steer alignment deadband for full drive power (rad, ~4.5 deg)
    double steer_alignment_deadband = 0.08;
    // Switching hysteresis to prevent 100 Hz chattering near 90 deg inversion boundaries (rad, ~10 deg)
    double swerve_invert_hysteresis = 0.175;

    // ---- Bank Angle Calculator (Eq 8 in paper) ----
    // Maximum allowable bank angle (roll induced by centripetal force)
    // 0.20 rad (~11.5 deg) preserves >= 2cm vertical stroke margin at 0.18m nominal height
    double max_bank_angle = 0.20;  // rad (~11.5 deg)
    // Height of center of gravity above ground (used for bank angle calc)
    double cog_height = 0.18;     // m, approximate CoG height
    // Low-pass filter coefficient for smoothing bank angle transitions
    double bank_angle_filter = 0.05;  // 0=frozen, 1=instant

    // ---- Low-Level Actuator Torque Loops (Paper Fig. 8 & Dual-Layer Mode) ----
    double kp_steer = 360.0;        // Nm/rad (DM-J4310P-2EC MIT Kp <= 500)
    double kd_steer = 5.0;          // Nm·s/rad (DM-J4310P-2EC MIT hardware Kd ceiling = 5.0)
    double kp_wheel = 15.0;         // Nm/(rad/s) (wheel speed PI proportional gain)
    double ki_wheel = 0.5;          // Nm/rad (wheel speed PI integral gain)
    double max_steer_torque = 7.0;  // Nm (DM-J4310P-2EC conservative peak; rated 3.5 Nm)
    double max_wheel_torque = 2.2;  // Nm (LK MF9010 V2 17T max torque)

    // Maximum wheel speed contribution from joint motion (rad/s).
    double max_rolling_compensation = 1.2;

    // ---- Velocity & Speed Saturation Limits ----
    double max_linear_vel  = 2.0;   // m/s (user linear driving limit)
    double max_angular_vel = 8.2;   // rad/s (supports up to 8.0 rad/s spin mode cleanly)
    double max_wheel_speed = 113.0; // rad/s (~1079 rpm, within LK MF9010 V2 17T rated 1080 rpm = 113.10 rad/s)
};

class DrivingController {
public:
    DrivingController(const DrivingControllerParams& params,
                      std::shared_ptr<kinematics::MobedKinematics> kinematics);

    /**
     * @brief Update the driving controller step
     *
     * @param cmd_vel Target body velocity [vx, vy, omega_z]
     * @param current_steer_angles Feedback from hardware [FL, FR, RL, RR]
     * @param current_ecc_angles Feedback from hardware for collision check [FL, FR, RL, RR]
     * @param dt Time step since last call
     * @param e_stop_active Emergency stop flag
     * @param is_homing Whether the robot is in homing mode
     * @param current_steer_vels Feedback steering joint velocities [FL, FR, RL, RR] (rad/s)
     * @param current_wheel_vels Feedback wheel rotational velocities [FL, FR, RL, RR] (rad/s)
     * @param current_ecc_vels Feedback eccentric joint velocities [FL, FR, RL, RR] (rad/s)
     * @param contact_valid Whether each wheel can use rolling compensation
     * @return std::tuple<Eigen::Vector4d, Eigen::Vector4d> (target_steer_angles, target_wheel_speeds)
     */
    std::tuple<Eigen::Vector4d, Eigen::Vector4d> update(
        const Eigen::Vector3d& cmd_vel,
        const Eigen::Vector4d& current_steer_angles,
        const Eigen::Vector4d& current_ecc_angles,
        double dt,
        bool e_stop_active = false,
        bool is_homing = false,
        const Eigen::Vector4d& current_steer_vels = Eigen::Vector4d::Zero(),
        const Eigen::Vector4d& current_wheel_vels = Eigen::Vector4d::Zero(),
        const Eigen::Vector4d& current_ecc_vels = Eigen::Vector4d::Zero(),
        const std::array<bool, NUM_LEGS>& contact_valid = {true, true, true, true},
        double alignment_yaw_rate = 0.0, bool spin_mode = false);

    /**
     * @brief Get synthesized low-level steering torques (Paper Fig. 8)
     */
    Eigen::Vector4d getSteerTorques() const { return steer_torques_; }

    /**
     * @brief Get synthesized low-level driving wheel torques with traction limits (Paper Fig. 8)
     */
    Eigen::Vector4d getWheelTorques() const { return wheel_torques_; }

    double getSteeringError() const { return max_steer_error_; }

    /**
     * @brief Get the bank angle (roll) induced by centripetal force during turning.
     *
     * Computes the optimal chassis roll angle to counteract centrifugal force
     * during curved motion, similar to a motorcycle leaning into a turn.
     * (Paper Section III.D & Eq 8)
     *
     * Sign convention (ISO 8855: X forward, Y left, Z up):
     * - Left turn (wz * vx > 0): returns negative roll (leans left / inward)
     * - Right turn (wz * vx < 0): returns positive roll (leans right / inward)
     *
     * @return double  Desired bank roll angle (rad)
     */
    double getBankAngle() const { return filtered_bank_angle_; }

    /**
     * @brief Compute the raw bank angle from current motion state.
     *
     * Uses the centripetal force formula (Paper Eq 8):
     *   tan(bank_angle) = v^2 / (R * g) = v * omega / g
     *
     * where v = linear speed, omega = yaw rate, g = gravity.
     *
     * @param cmd_vel  Current velocity command [vx, vy, omega_z]
     * @return double  Raw bank angle (rad)
     */
    double computeBankAngle(const Eigen::Vector3d& cmd_vel) const;

    void reset() {
        initialized_ = false;
        filtered_bank_angle_ = 0.0;
        prev_swerve_inverted_.fill(false);
        steer_torques_.setZero();
        wheel_torques_.setZero();
        wheel_speed_integrals_.setZero();
    }

private:
    DrivingControllerParams params_;
    std::shared_ptr<kinematics::MobedKinematics> kinematics_;

    Eigen::Vector4d prev_steer_angles_;
    Eigen::Vector4d prev_wheel_speeds_;

    // Actuator torque loops
    Eigen::Vector4d steer_torques_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d wheel_torques_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d wheel_speed_integrals_ = Eigen::Vector4d::Zero();

    bool initialized_ = false;
    double max_steer_error_ = 0.0;
    double filtered_bank_angle_ = 0.0;  // smoothed bank angle output
    std::array<bool, 4> prev_swerve_inverted_ = {false, false, false, false};

    /**
     * @brief Choose a rolling heading within the safe steering sector.
     *
     * @param leg_index Leg index (FL, FR, RL, RR)
     * @param direct_angle Direct rolling heading (rad)
     * @param reversed_angle Reversed rolling heading (rad)
     * @param curr_steer Previous steering target (rad)
     * @return Optimized steering angle (rad)
     */
    double optimizeSwerveModule(
        int leg_index, double direct_angle, double reversed_angle, double curr_steer);

    /**
     * @brief Dynamic collision-free transit interpolator.
     * Guarantees intermediate angles during slew rate limiting NEVER enter the forbidden sector.
     */
    double enforceForbiddenSectorTransit(
        int leg_index, double prev_s, double opt_s, double max_delta) const;
};

}  // namespace controllers
}  // namespace robot_control
