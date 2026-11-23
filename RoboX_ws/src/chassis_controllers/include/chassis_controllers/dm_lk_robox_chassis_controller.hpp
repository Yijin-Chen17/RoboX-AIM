#ifndef CHASSIS_CONTROLLERS_DM_LK_ROBOX_CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLERS_DM_LK_ROBOX_CHASSIS_CONTROLLER_HPP

#pragma once

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "chassis_controllers/msg/robox_chassis_control.hpp"
#include "chassis_controllers/utils/robox/core/mobed_types.hpp"
#include "chassis_controllers/utils/robox/core/robot_params.hpp"
#include "chassis_controllers/utils/robox/robox_solver.hpp"
#include "custom_msgs/msg/read_dm_motor.hpp"
#include "custom_msgs/msg/read_lk_motor_multi.hpp"
#include "custom_msgs/msg/write_dm_motor_position_control_with_speed_limit.hpp"
#include "custom_msgs/msg/write_lk_motor_broadcast_current_control.hpp"

class DmLkRoboxChassisController : public rclcpp::Node
{
public:
    explicit DmLkRoboxChassisController(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    static constexpr double kWheelStopThresholdRadPerSec = 0.5;

    void init_parameters();
    void init_communications();

    void command_callback(const chassis_controllers::msg::RoboxChassisControl::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void dm_motor_callback(int leg_idx, bool is_steer,
                           const custom_msgs::msg::ReadDmMotor::SharedPtr msg);
    void wheel_motor_callback(const custom_msgs::msg::ReadLkMotorMulti::SharedPtr msg);

    bool all_motors_initialized() const;
    int16_t torque_to_broadcast_cmd(double torque_nm) const;
    void publish_emergency_stop_commands();
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
    std::string wheel_read_topic_;
    std::string wheel_write_topic_;

    rclcpp::Subscription<chassis_controllers::msg::RoboxChassisControl>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::array<rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr,
               robot_control::NUM_LEGS> steer_subs_;
    std::array<rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr,
               robot_control::NUM_LEGS> ecc_subs_;
    rclcpp::Subscription<custom_msgs::msg::ReadLkMotorMulti>::SharedPtr wheel_sub_;

    std::array<rclcpp::Publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>::SharedPtr,
               robot_control::NUM_LEGS> steer_pubs_;
    std::array<rclcpp::Publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>::SharedPtr,
               robot_control::NUM_LEGS> ecc_pubs_;
    rclcpp::Publisher<custom_msgs::msg::WriteLkMotorBroadcastCurrentControl>::SharedPtr wheel_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    // ================================================================
    // Hardware Parameters & Feedback State
    // ================================================================
    std::array<double, robot_control::NUM_LEGS> steer_signs_   = {1.0, 1.0, 1.0, 1.0};
    std::array<double, robot_control::NUM_LEGS> steer_offsets_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, robot_control::NUM_LEGS> ecc_signs_     = {1.0, -1.0, 1.0, -1.0};
    std::array<double, robot_control::NUM_LEGS> ecc_offsets_   = {M_PI, M_PI, M_PI, M_PI};
    std::array<double, robot_control::NUM_LEGS> wheel_signs_   = {-1.0, 1.0, -1.0, 1.0};

    double dm_steer_speed_limit_   = 12.0;
    double dm_ecc_speed_limit_     = 3.5;
    double wheel_speed_kp_         = 15.0;
    double wheel_speed_ki_         = 0.5;
    double wheel_kt_               = 0.16;
    double wheel_max_torque_nm_    = 2.2;
    double wheel_mf_max_current_a_ = 16.5;

    Eigen::Matrix3d R_body_imu_ = Eigen::Matrix3d::Identity();

    std::array<bool, robot_control::NUM_LEGS> steer_received_{false, false, false, false};
    std::array<bool, robot_control::NUM_LEGS> ecc_received_{false, false, false, false};
    std::array<bool, robot_control::NUM_LEGS> wheel_received_{false, false, false, false};

    bool joints_initialized_ = false;

    rclcpp::Time last_time_;
    rclcpp::Time last_imu_time_;
};

#endif  // CHASSIS_CONTROLLERS_DM_LK_ROBOX_CHASSIS_CONTROLLER_HPP
