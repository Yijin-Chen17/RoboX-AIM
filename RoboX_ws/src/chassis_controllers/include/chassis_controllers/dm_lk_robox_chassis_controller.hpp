#ifndef CHASSIS_CONTROLLERS_DM_LK_ROBOX_CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLERS_DM_LK_ROBOX_CHASSIS_CONTROLLER_HPP

#pragma once

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "chassis_controllers/msg/robox_chassis_control.hpp"
#include "chassis_controllers/utils/robox/controllers/mit_servo.hpp"
#include "chassis_controllers/utils/robox/core/mobed_types.hpp"
#include "chassis_controllers/utils/robox/core/robot_params.hpp"
#include "chassis_controllers/utils/robox/robox_solver.hpp"
#include "custom_msgs/msg/read_dm_motor.hpp"
#include "custom_msgs/msg/read_lk_motor.hpp"
#include "custom_msgs/msg/write_dm_motor_mit_control.hpp"
#include "custom_msgs/msg/write_lk_motor_speed_control_with_torque_limit.hpp"

class DmLkRoboxChassisController : public rclcpp::Node
{
public:
    explicit DmLkRoboxChassisController(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void init_parameters();
    void init_communications();

    void command_callback(const chassis_controllers::msg::RoboxChassisControl::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void dm_motor_callback(int leg_idx, bool is_steer,
                           const custom_msgs::msg::ReadDmMotor::SharedPtr msg);
    void wheel_motor_callback(int leg_idx, const custom_msgs::msg::ReadLkMotor::SharedPtr msg);

    bool all_motors_initialized() const;
    bool check_motors_healthy(const rclcpp::Time& now, double& oldest_stamp_sec) const;
    void publish_disable_all_motors();
    void control_loop();

    // ================================================================
    // Encapsulated RoboX Whole-Body Solver
    // ================================================================
    robot_control::RoboxSolver robox_solver_;
    robot_control::RoboxSensorState sensor_state_;
    robot_control::RoboxCommandState command_state_;

    // ================================================================
    // ROS 2 Subscriptions, Publishers & Timer
    // ================================================================
    std::string cmd_topic_;
    std::string imu_topic_;
    std::vector<std::string> steer_read_topics_;
    std::vector<std::string> steer_write_topics_;
    std::vector<std::string> ecc_read_topics_;
    std::vector<std::string> ecc_write_topics_;
    std::vector<std::string> wheel_read_topics_;
    std::vector<std::string> wheel_write_topics_;

    rclcpp::Subscription<chassis_controllers::msg::RoboxChassisControl>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::array<rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr,
               robot_control::NUM_LEGS> steer_subs_;
    std::array<rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr,
               robot_control::NUM_LEGS> ecc_subs_;
    std::array<rclcpp::Subscription<custom_msgs::msg::ReadLkMotor>::SharedPtr,
               robot_control::NUM_LEGS> wheel_subs_;

    std::array<rclcpp::Publisher<custom_msgs::msg::WriteDmMotorMITControl>::SharedPtr,
               robot_control::NUM_LEGS> steer_pubs_;
    std::array<rclcpp::Publisher<custom_msgs::msg::WriteDmMotorMITControl>::SharedPtr,
               robot_control::NUM_LEGS> ecc_pubs_;
    std::array<rclcpp::Publisher<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>::SharedPtr,
               robot_control::NUM_LEGS> wheel_pubs_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    // ================================================================
    // Hardware Parameters & Feedback State
    // ================================================================
    bool enable_motors_           = false;
    bool use_ecc_torque_ff_       = true;
    double motor_watchdog_timeout_ = 0.25;
    double cmd_timeout_           = 0.5;

    std::array<double, robot_control::NUM_LEGS> steer_signs_   = {1.0, 1.0, 1.0, 1.0};
    std::array<double, robot_control::NUM_LEGS> steer_offsets_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, robot_control::NUM_LEGS> ecc_signs_     = {1.0, -1.0, 1.0, -1.0};
    std::array<double, robot_control::NUM_LEGS> ecc_offsets_   = {M_PI, M_PI, M_PI, M_PI};
    std::array<double, robot_control::NUM_LEGS> wheel_signs_   = {-1.0, 1.0, -1.0, 1.0};

    double dm_steer_kp_            = 360.0;
    double dm_steer_kd_            = 5.0;
    double dm_steer_pmax_          = M_PI;
    double dm_ecc_kp_              = 125.0;
    double dm_ecc_kd_              = 4.8;
    double dm_ecc_pmax_            = M_PI;
    double wheel_kt_               = 0.16;
    double wheel_max_torque_nm_    = 2.2;
    double wheel_mf_max_current_a_ = 16.5;

    Eigen::Matrix3d R_body_imu_ = Eigen::Matrix3d::Identity();

    std::array<double, robot_control::NUM_LEGS> raw_steer_pos_{0.0, 0.0, 0.0, 0.0};
    std::array<double, robot_control::NUM_LEGS> raw_ecc_pos_{0.0, 0.0, 0.0, 0.0};

    std::array<bool, robot_control::NUM_LEGS> steer_received_{false, false, false, false};
    std::array<bool, robot_control::NUM_LEGS> ecc_received_{false, false, false, false};
    std::array<bool, robot_control::NUM_LEGS> wheel_received_{false, false, false, false};

    std::array<bool, robot_control::NUM_LEGS> steer_online_{false, false, false, false};
    std::array<bool, robot_control::NUM_LEGS> ecc_online_{false, false, false, false};
    std::array<bool, robot_control::NUM_LEGS> wheel_online_{false, false, false, false};

    bool joints_initialized_ = false;

    std::array<rclcpp::Time, robot_control::NUM_LEGS> last_steer_time_{};
    std::array<rclcpp::Time, robot_control::NUM_LEGS> last_ecc_time_{};
    std::array<rclcpp::Time, robot_control::NUM_LEGS> last_wheel_time_{};

    rclcpp::Time last_time_;
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_imu_time_;
};

#endif  // CHASSIS_CONTROLLERS_DM_LK_ROBOX_CHASSIS_CONTROLLER_HPP
