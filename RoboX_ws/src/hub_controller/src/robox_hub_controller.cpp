#include "hub_controller/robox_hub_controller.hpp"

#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

RoboxHubController::RoboxHubController(const rclcpp::NodeOptions& options)
    : Node("robox_hub_controller", options)
{
    init_parameters();
    init_communications();

    last_time_              = this->now();
    last_sbus_time_         = last_time_;
    last_external_cmd_time_ = last_time_;

    RCLCPP_INFO(this->get_logger(),
        "RoboX Hub Controller started (sbus_enabled=%s, publishing '%s').",
        sbus_enabled_ ? "true" : "false", chassis_cmd_write_topic_.c_str());
}

void RoboxHubController::init_parameters()
{
    this->declare_parameter<bool>("sbus_enabled", true);
    this->declare_parameter<std::string>("sbus_read_topic", "sbus_rc");
    this->declare_parameter<std::string>("external_cmd_read_topic", "external_robox_command");
    this->declare_parameter<std::string>("chassis_cmd_write_topic", "robox_chassis_command");

    this->declare_parameter<bool>("sbus_failsafe_estop", true);
    this->declare_parameter<double>("sbus_timeout", 0.5);
    this->declare_parameter<double>("external_cmd_timeout", 0.5);

    this->declare_parameter<double>("sbus_ch_min", 172.0);
    this->declare_parameter<double>("sbus_ch_mid", 992.0);
    this->declare_parameter<double>("sbus_ch_max", 1811.0);
    this->declare_parameter<double>("sbus_deadband", 0.05);

    this->declare_parameter<int>("sbus_ch_vx", 1);
    this->declare_parameter<int>("sbus_ch_vy", 0);
    this->declare_parameter<int>("sbus_ch_wz", 3);
    this->declare_parameter<int>("sbus_ch_height", 2);
    this->declare_parameter<int>("sbus_ch_estop", 4);
    this->declare_parameter<int>("sbus_ch_climb_mode", 5);
    this->declare_parameter<int>("sbus_ch_spin", 6);
    this->declare_parameter<int>("sbus_ch_posture", 7);

    this->declare_parameter<bool>("sbus_inv_vx", false);
    this->declare_parameter<bool>("sbus_inv_vy", false);
    this->declare_parameter<bool>("sbus_inv_wz", false);
    this->declare_parameter<bool>("sbus_inv_height", false);
    this->declare_parameter<bool>("sbus_inv_estop", false);

    this->declare_parameter<double>("sbus_height_min", 0.084);
    this->declare_parameter<double>("sbus_height_max", 0.22);
    this->declare_parameter<double>("default_body_height", 0.175);
    this->declare_parameter<int>("default_posture_mode", 0);
    this->declare_parameter<int>("default_climb_mode", 1);
    this->declare_parameter<double>("default_curb_height", 0.075);
    this->declare_parameter<double>("default_payload_mass", 0.0);

    robot_control::controllers::TeleopSmootherParams teleop_params;
    this->declare_parameter<double>("teleop_max_vx", teleop_params.max_lin_vel);
    this->declare_parameter<double>("teleop_max_vy", teleop_params.max_strafe_vel);
    this->declare_parameter<double>("teleop_max_wz", teleop_params.max_ang_vel);
    this->declare_parameter<double>("teleop_accel_lin_fwd", teleop_params.accel_lin_fwd);
    this->declare_parameter<double>("teleop_accel_lin_brake", teleop_params.accel_lin_brake);
    this->declare_parameter<double>("teleop_accel_strafe", teleop_params.accel_strafe);
    this->declare_parameter<double>("teleop_accel_angular", teleop_params.accel_angular);
    this->declare_parameter<double>("teleop_max_lateral_acc", teleop_params.max_lateral_acc);

    sbus_enabled_            = this->get_parameter("sbus_enabled").as_bool();
    sbus_read_topic_         = this->get_parameter("sbus_read_topic").as_string();
    external_cmd_read_topic_ = this->get_parameter("external_cmd_read_topic").as_string();
    chassis_cmd_write_topic_ = this->get_parameter("chassis_cmd_write_topic").as_string();

    sbus_failsafe_estop_  = this->get_parameter("sbus_failsafe_estop").as_bool();
    sbus_timeout_         = this->get_parameter("sbus_timeout").as_double();
    external_cmd_timeout_ = this->get_parameter("external_cmd_timeout").as_double();

    sbus_ch_min_   = this->get_parameter("sbus_ch_min").as_double();
    sbus_ch_mid_   = this->get_parameter("sbus_ch_mid").as_double();
    sbus_ch_max_   = this->get_parameter("sbus_ch_max").as_double();
    sbus_deadband_ = this->get_parameter("sbus_deadband").as_double();

    sbus_ch_vx_         = this->get_parameter("sbus_ch_vx").as_int();
    sbus_ch_vy_         = this->get_parameter("sbus_ch_vy").as_int();
    sbus_ch_wz_         = this->get_parameter("sbus_ch_wz").as_int();
    sbus_ch_height_     = this->get_parameter("sbus_ch_height").as_int();
    sbus_ch_estop_      = this->get_parameter("sbus_ch_estop").as_int();
    sbus_ch_climb_mode_ = this->get_parameter("sbus_ch_climb_mode").as_int();
    sbus_ch_spin_       = this->get_parameter("sbus_ch_spin").as_int();
    sbus_ch_posture_    = this->get_parameter("sbus_ch_posture").as_int();

    sbus_inv_vx_     = this->get_parameter("sbus_inv_vx").as_bool();
    sbus_inv_vy_     = this->get_parameter("sbus_inv_vy").as_bool();
    sbus_inv_wz_     = this->get_parameter("sbus_inv_wz").as_bool();
    sbus_inv_height_ = this->get_parameter("sbus_inv_height").as_bool();
    sbus_inv_estop_  = this->get_parameter("sbus_inv_estop").as_bool();

    sbus_height_min_     = this->get_parameter("sbus_height_min").as_double();
    sbus_height_max_     = this->get_parameter("sbus_height_max").as_double();
    default_body_height_ = this->get_parameter("default_body_height").as_double();
    default_posture_mode_ = static_cast<uint8_t>(
        std::clamp<int64_t>(this->get_parameter("default_posture_mode").as_int(), 0, 3));
    default_climb_mode_ = static_cast<uint8_t>(
        std::clamp<int64_t>(this->get_parameter("default_climb_mode").as_int(), 0, 2));
    default_curb_height_  = static_cast<float>(this->get_parameter("default_curb_height").as_double());
    default_payload_mass_ = static_cast<float>(this->get_parameter("default_payload_mass").as_double());

    sbus_body_height_  = default_body_height_;
    sbus_posture_mode_ = default_posture_mode_;
    sbus_climb_mode_   = default_climb_mode_;

    teleop_params.max_lin_vel     = this->get_parameter("teleop_max_vx").as_double();
    teleop_params.max_strafe_vel  = this->get_parameter("teleop_max_vy").as_double();
    teleop_params.max_ang_vel     = this->get_parameter("teleop_max_wz").as_double();
    teleop_params.accel_lin_fwd   = this->get_parameter("teleop_accel_lin_fwd").as_double();
    teleop_params.accel_lin_brake = this->get_parameter("teleop_accel_lin_brake").as_double();
    teleop_params.accel_strafe    = this->get_parameter("teleop_accel_strafe").as_double();
    teleop_params.accel_angular   = this->get_parameter("teleop_accel_angular").as_double();
    teleop_params.max_lateral_acc = this->get_parameter("teleop_max_lateral_acc").as_double();
    teleop_smoother_.setParams(teleop_params);
}

void RoboxHubController::init_communications()
{
    const auto sensor_qos = rclcpp::SensorDataQoS();

    if (sbus_enabled_) {
        sbus_sub_ = this->create_subscription<custom_msgs::msg::ReadSBUSRC>(
            sbus_read_topic_, sensor_qos,
            [this](const custom_msgs::msg::ReadSBUSRC::SharedPtr msg) {
                sbus_callback(msg);
            });
    }

    external_cmd_sub_ = this->create_subscription<chassis_controllers::msg::RoboxChassisControl>(
        external_cmd_read_topic_, sensor_qos,
        [this](const chassis_controllers::msg::RoboxChassisControl::SharedPtr msg) {
            external_cmd_callback(msg);
        });

    chassis_cmd_pub_ = this->create_publisher<chassis_controllers::msg::RoboxChassisControl>(
        chassis_cmd_write_topic_, sensor_qos);

    control_timer_ = this->create_wall_timer(10ms, [this] { control_loop(); });
}

double RoboxHubController::normalize_sbus_channel(
    const uint16_t raw, const bool invert, const bool apply_deadband) const
{
    const double span = (raw >= sbus_ch_mid_)
                      ? std::max(1.0, sbus_ch_max_ - sbus_ch_mid_)
                      : std::max(1.0, sbus_ch_mid_ - sbus_ch_min_);
    double norm = std::clamp((static_cast<double>(raw) - sbus_ch_mid_) / span, -1.0, 1.0);
    if (invert) {
        norm = -norm;
    }
    if (apply_deadband) {
        if (std::abs(norm) <= sbus_deadband_) {
            return 0.0;
        }
        const double sign = (norm > 0.0) ? 1.0 : -1.0;
        norm = sign * (std::abs(norm) - sbus_deadband_) / std::max(1e-3, 1.0 - sbus_deadband_);
    }
    return norm;
}

void RoboxHubController::sbus_callback(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg)
{
    last_sbus_time_ = this->now();
    sbus_received_  = true;

    if (!msg->online || msg->fail_safe || msg->channels.size() < 16) {
        sbus_active_    = false;
        sbus_norm_vx_   = 0.0;
        sbus_norm_vy_   = 0.0;
        sbus_norm_wz_   = 0.0;
        sbus_spin_mode_ = 0;
        if (sbus_failsafe_estop_) {
            sbus_estop_ = true;
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[SBUS RC] offline=%u fail_safe=%u frame_lost=%u ch_size=%zu",
            msg->online, msg->fail_safe, msg->frame_lost, msg->channels.size());
        return;
    }

    sbus_active_ = true;

    auto read_switch = [&](const int ch_idx, const bool invert = false) -> double {
        if (ch_idx < 0 || ch_idx >= static_cast<int>(msg->channels.size())) {
            return 0.0;
        }
        return normalize_sbus_channel(msg->channels[ch_idx], invert, false);
    };
    auto read_stick = [&](const int ch_idx, const bool invert) -> double {
        if (ch_idx < 0 || ch_idx >= static_cast<int>(msg->channels.size())) {
            return 0.0;
        }
        return normalize_sbus_channel(msg->channels[ch_idx], invert, true);
    };

    if (sbus_ch_estop_ >= 0) {
        sbus_estop_ = (read_switch(sbus_ch_estop_, sbus_inv_estop_) > 0.5);
    } else {
        sbus_estop_ = false;
    }

    sbus_norm_vx_ = read_stick(sbus_ch_vx_, sbus_inv_vx_);
    sbus_norm_vy_ = read_stick(sbus_ch_vy_, sbus_inv_vy_);
    sbus_norm_wz_ = read_stick(sbus_ch_wz_, sbus_inv_wz_);

    if (sbus_ch_height_ >= 0) {
        const double h_norm  = read_switch(sbus_ch_height_, sbus_inv_height_);
        const double h_ratio = 0.5 * (h_norm + 1.0);
        sbus_body_height_ = std::clamp(
            sbus_height_min_ + h_ratio * (sbus_height_max_ - sbus_height_min_),
            0.08, 0.22);
    }

    // 3-position Climb Mode switch: low=0 (DISABLED), mid=1 (PAPER_NORMAL), high=2 (STAIR_200MM)
    if (sbus_ch_climb_mode_ >= 0) {
        const double c_val = read_switch(sbus_ch_climb_mode_);
        sbus_climb_mode_ = (c_val < -0.4) ? 0 : ((c_val > 0.4) ? 2 : 1);
    }

    // 3-position Spin Mode switch: low=-1 (CW), mid=0 (off), high=+1 (CCW)
    if (sbus_ch_spin_ >= 0) {
        const double s_val = read_switch(sbus_ch_spin_);
        sbus_spin_mode_ = (s_val > 0.4) ? 1 : ((s_val < -0.4) ? -1 : 0);
    }

    // 3-position Posture switch: low=0 (OUTWARD), mid=1 (INWARD), high=3 (STRAFE_OUTWARD)
    if (sbus_ch_posture_ >= 0) {
        const double p_val = read_switch(sbus_ch_posture_);
        sbus_posture_mode_ = (p_val < -0.4) ? 0 : ((p_val > 0.4) ? 3 : 1);
    }
}

void RoboxHubController::external_cmd_callback(
    const chassis_controllers::msg::RoboxChassisControl::SharedPtr msg)
{
    external_cmd_          = *msg;
    external_cmd_received_ = true;
    last_external_cmd_time_ = this->now();
}

void RoboxHubController::control_loop()
{
    const auto now = this->now();
    const double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0 || dt > 0.5) {
        return;
    }

    if (sbus_enabled_ && sbus_received_) {
        Eigen::Vector3d cmd_vel = Eigen::Vector3d::Zero();
        if ((now - last_sbus_time_).seconds() > sbus_timeout_) {
            sbus_active_    = false;
            sbus_norm_vx_   = 0.0;
            sbus_norm_vy_   = 0.0;
            sbus_norm_wz_   = 0.0;
            sbus_spin_mode_ = 0;
            cmd_vel = teleop_smoother_.update(0.0, 0.0, 0.0, dt, true);
            if (sbus_failsafe_estop_) {
                sbus_estop_ = true;
            }
        } else if (sbus_active_) {
            if (sbus_estop_) {
                teleop_smoother_.reset();
                cmd_vel.setZero();
            } else {
                cmd_vel = teleop_smoother_.update(sbus_norm_vx_, sbus_norm_vy_, sbus_norm_wz_, dt, false);
            }
        } else {
            cmd_vel = teleop_smoother_.update(0.0, 0.0, 0.0, dt, true);
        }

        chassis_controllers::msg::RoboxChassisControl out;
        out.x_speed            = static_cast<float>(cmd_vel.x());
        out.y_speed            = static_cast<float>(cmd_vel.y());
        out.yaw_speed          = static_cast<float>(cmd_vel.z());
        out.body_height        = static_cast<float>(sbus_body_height_);
        out.body_roll          = 0.0f;
        out.body_pitch         = 0.0f;
        out.emergency_stop     = sbus_estop_ ? 1 : 0;
        out.spin_mode          = sbus_estop_ ? 0 : sbus_spin_mode_;
        out.posture_mode       = sbus_posture_mode_;
        out.climb_mode         = sbus_climb_mode_;
        out.target_curb_height = default_curb_height_;
        out.payload_mass       = default_payload_mass_;
        chassis_cmd_pub_->publish(out);
        return;
    }

    // Fallback: passthrough external RoboxChassisControl when SBUS is inactive/disabled
    if (external_cmd_received_) {
        chassis_controllers::msg::RoboxChassisControl out = external_cmd_;
        if (external_cmd_timeout_ > 0.0 &&
            (now - last_external_cmd_time_).seconds() > external_cmd_timeout_) {
            out.x_speed   = 0.0f;
            out.y_speed   = 0.0f;
            out.yaw_speed = 0.0f;
            out.spin_mode = 0;
        }
        chassis_cmd_pub_->publish(out);
    }
}

int main(const int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<RoboxHubController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
