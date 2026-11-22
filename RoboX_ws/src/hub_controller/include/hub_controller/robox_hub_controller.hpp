#ifndef HUB_CONTROLLER_ROBOX_HUB_CONTROLLER_HPP
#define HUB_CONTROLLER_ROBOX_HUB_CONTROLLER_HPP

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "chassis_controllers/msg/robox_chassis_control.hpp"
#include "custom_msgs/msg/read_sbusrc.hpp"
#include "hub_controller/utils/teleop_smoother.hpp"

class RoboxHubController : public rclcpp::Node
{
public:
    explicit RoboxHubController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void init_parameters();
    void init_communications();

    void sbus_callback(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg);
    void external_cmd_callback(const chassis_controllers::msg::RoboxChassisControl::SharedPtr msg);
    void control_loop();

    double normalize_sbus_channel(uint16_t raw, bool invert, bool apply_deadband) const;

    // ROS 2 communication
    std::string sbus_read_topic_;
    std::string external_cmd_read_topic_;
    std::string chassis_cmd_write_topic_;

    rclcpp::Subscription<custom_msgs::msg::ReadSBUSRC>::SharedPtr sbus_sub_;
    rclcpp::Subscription<chassis_controllers::msg::RoboxChassisControl>::SharedPtr external_cmd_sub_;
    rclcpp::Publisher<chassis_controllers::msg::RoboxChassisControl>::SharedPtr chassis_cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // SBUS parameters
    bool sbus_enabled_        = true;
    bool sbus_failsafe_estop_ = true;
    double sbus_timeout_      = 0.5;
    double external_cmd_timeout_ = 0.5;

    double sbus_ch_min_   = 172.0;
    double sbus_ch_mid_   = 992.0;
    double sbus_ch_max_   = 1811.0;
    double sbus_deadband_ = 0.05;

    int sbus_ch_vx_         = 1;
    int sbus_ch_vy_         = 0;
    int sbus_ch_wz_         = 3;
    int sbus_ch_height_     = 2;
    int sbus_ch_estop_      = 4;
    int sbus_ch_climb_mode_ = 5;
    int sbus_ch_spin_       = 6;
    int sbus_ch_posture_    = 7;

    bool sbus_inv_vx_     = false;
    bool sbus_inv_vy_     = false;
    bool sbus_inv_wz_     = false;
    bool sbus_inv_height_ = false;
    bool sbus_inv_estop_  = false;

    double sbus_height_min_      = 0.084;
    double sbus_height_max_      = 0.22;
    double default_body_height_  = 0.175;
    uint8_t default_posture_mode_ = 0;
    uint8_t default_climb_mode_   = 1;
    float default_curb_height_   = 0.075f;
    float default_payload_mass_  = 0.0f;

    // Runtime state
    robot_control::controllers::TeleopSmoother teleop_smoother_;
    bool sbus_received_         = false;
    bool sbus_active_           = false;
    bool sbus_estop_            = false;
    bool external_cmd_received_ = false;

    double sbus_norm_vx_     = 0.0;
    double sbus_norm_vy_     = 0.0;
    double sbus_norm_wz_     = 0.0;
    double sbus_body_height_ = 0.175;
    int8_t sbus_spin_mode_   = 0;
    uint8_t sbus_posture_mode_ = 0;
    uint8_t sbus_climb_mode_   = 1;

    chassis_controllers::msg::RoboxChassisControl external_cmd_{};

    rclcpp::Time last_time_;
    rclcpp::Time last_sbus_time_;
    rclcpp::Time last_external_cmd_time_;
};

#endif  // HUB_CONTROLLER_ROBOX_HUB_CONTROLLER_HPP
