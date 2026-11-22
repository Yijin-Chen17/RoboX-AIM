#include "chassis_controllers/dm_lk_robox_chassis_controller.hpp"

#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;
using namespace robot_control;

namespace {
double normalize_angle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}
}  // namespace

DmLkRoboxChassisController::DmLkRoboxChassisController(const rclcpp::NodeOptions& options)
    : Node("dm_lk_robox_chassis_controller", options)
{
    init_parameters();
    init_communications();

    last_time_     = this->now();
    last_cmd_time_ = last_time_;
    last_imu_time_ = last_time_;
    last_steer_time_.fill(last_time_);
    last_ecc_time_.fill(last_time_);
    last_wheel_time_.fill(last_time_);

    RCLCPP_INFO(this->get_logger(),
        "DM/LK RoboX chassis controller started (enable_motors=%s). Waiting for 12 motor feedbacks...",
        enable_motors_ ? "true" : "false");
}

void DmLkRoboxChassisController::init_parameters()
{
    this->declare_parameter<std::string>("robot_model", "robox_v2");
    const std::string robot_model = this->get_parameter("robot_model").as_string();
    const RobotModelType model_type =
        (robot_model == "legacy" || robot_model == "mobed_legacy")
            ? RobotModelType::MOBED_LEGACY
            : RobotModelType::ROBOX_V2;
    robox_solver_.setRobotModel(model_type);
    command_state_.target_height = robox_solver_.nominalDriveHeight();

    if (model_type == RobotModelType::MOBED_LEGACY) {
        steer_signs_   = {1.0, 1.0, 1.0, 1.0};
        steer_offsets_ = {0.0, 0.0, 0.0, 0.0};
        ecc_signs_     = {1.0, -1.0, -1.0, 1.0};
        ecc_offsets_   = {-1.9003, -3.0526, 1.5320, -3.0473};
        wheel_signs_   = {1.0, -1.0, 1.0, -1.0};
    } else {
        steer_signs_   = {1.0, 1.0, 1.0, 1.0};
        steer_offsets_ = {0.0, 0.0, 0.0, 0.0};
        ecc_signs_     = {1.0, -1.0, 1.0, -1.0};
        ecc_offsets_   = {M_PI, M_PI, M_PI, M_PI};
        wheel_signs_   = {-1.0, 1.0, -1.0, 1.0};
    }

    auto load_array4_param = [this](const std::string& name, std::array<double, NUM_LEGS>& target) {
        std::vector<double> defaults(target.begin(), target.end());
        this->declare_parameter<std::vector<double>>(name, defaults);
        const auto val = this->get_parameter(name).as_double_array();
        if (val.size() == NUM_LEGS) {
            std::copy_n(val.begin(), NUM_LEGS, target.begin());
        } else {
            RCLCPP_WARN(this->get_logger(),
                "Parameter '%s' must have 4 elements (got %zu); keeping defaults.",
                name.c_str(), val.size());
        }
    };

    load_array4_param("steer_signs", steer_signs_);
    load_array4_param("steer_offsets", steer_offsets_);
    load_array4_param("ecc_signs", ecc_signs_);
    load_array4_param("ecc_offsets", ecc_offsets_);
    load_array4_param("wheel_signs", wheel_signs_);

    this->declare_parameter<bool>("enable_motors", false);
    this->declare_parameter<bool>("use_ecc_torque_ff", true);
    this->declare_parameter<double>("motor_watchdog_timeout", 0.25);
    this->declare_parameter<double>("cmd_timeout", 0.5);

    this->declare_parameter<double>("dm_steer_kp", 360.0);
    this->declare_parameter<double>("dm_steer_kd", 5.0);
    this->declare_parameter<double>("dm_steer_pmax", M_PI);

    this->declare_parameter<double>("dm_ecc_kp", 125.0);
    this->declare_parameter<double>("dm_ecc_kd", 4.8);
    this->declare_parameter<double>("dm_ecc_pmax", M_PI);

    this->declare_parameter<double>("wheel_kt", 0.16);
    this->declare_parameter<double>("wheel_max_torque_nm", 2.2);
    this->declare_parameter<double>("wheel_mf_max_current_a", 16.5);

    enable_motors_          = this->get_parameter("enable_motors").as_bool();
    use_ecc_torque_ff_      = this->get_parameter("use_ecc_torque_ff").as_bool();
    motor_watchdog_timeout_ = this->get_parameter("motor_watchdog_timeout").as_double();
    cmd_timeout_            = this->get_parameter("cmd_timeout").as_double();

    dm_steer_kp_            = this->get_parameter("dm_steer_kp").as_double();
    dm_steer_kd_            = this->get_parameter("dm_steer_kd").as_double();
    dm_steer_pmax_          = this->get_parameter("dm_steer_pmax").as_double();

    dm_ecc_kp_              = this->get_parameter("dm_ecc_kp").as_double();
    dm_ecc_kd_              = this->get_parameter("dm_ecc_kd").as_double();
    dm_ecc_pmax_            = this->get_parameter("dm_ecc_pmax").as_double();

    wheel_kt_               = this->get_parameter("wheel_kt").as_double();
    wheel_max_torque_nm_    = this->get_parameter("wheel_max_torque_nm").as_double();
    wheel_mf_max_current_a_ = this->get_parameter("wheel_mf_max_current_a").as_double();

    // Topics (relative defaults aligned with universal_controllers_v2)
    static constexpr std::array<const char*, NUM_LEGS> kLegNames = {"lf", "rf", "lb", "rb"};
    auto make_default_topics = [](const std::string& group, const std::string& suffix) {
        std::vector<std::string> out;
        out.reserve(NUM_LEGS);
        for (const char* leg : kLegNames) {
            out.push_back(group + "_" + leg + "_" + suffix);
        }
        return out;
    };
    auto load_topics_param = [this](const std::string& name,
                                    const std::vector<std::string>& defaults) {
        this->declare_parameter<std::vector<std::string>>(name, defaults);
        const auto val = this->get_parameter(name).as_string_array();
        if (val.size() == NUM_LEGS) {
            return val;
        }
        RCLCPP_WARN(this->get_logger(),
            "Parameter '%s' must have 4 topics (got %zu); using defaults.",
            name.c_str(), val.size());
        return defaults;
    };

    this->declare_parameter<std::string>("chassis_cmd_read_topic", "robox_chassis_command");
    this->declare_parameter<std::string>("imu_read_topic", "imu");
    cmd_topic_ = this->get_parameter("chassis_cmd_read_topic").as_string();
    imu_topic_ = this->get_parameter("imu_read_topic").as_string();

    steer_read_topics_  = load_topics_param("steer_motor_read_topics",  make_default_topics("steer", "read"));
    steer_write_topics_ = load_topics_param("steer_motor_write_topics", make_default_topics("steer", "write"));
    ecc_read_topics_    = load_topics_param("ecc_motor_read_topics",    make_default_topics("ecc", "read"));
    ecc_write_topics_   = load_topics_param("ecc_motor_write_topics",   make_default_topics("ecc", "write"));
    wheel_read_topics_  = load_topics_param("wheel_motor_read_topics",  make_default_topics("wheel", "read"));
    wheel_write_topics_ = load_topics_param("wheel_motor_write_topics", make_default_topics("wheel", "write"));

    // IMU configuration
    this->declare_parameter<bool>("require_imu_for_homing", true);
    this->declare_parameter<std::vector<double>>("imu_mount_rpy_deg", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("imu_lever_arm", {0.0, 0.0, 0.0});
    robox_solver_.setRequireImuForHoming(this->get_parameter("require_imu_for_homing").as_bool());

    const auto rpy_param = this->get_parameter("imu_mount_rpy_deg").as_double_array();
    if (rpy_param.size() == 3) {
        constexpr double kDegToRad = M_PI / 180.0;
        const Eigen::AngleAxisd roll_aa(rpy_param[0] * kDegToRad, Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd pitch_aa(rpy_param[1] * kDegToRad, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd yaw_aa(rpy_param[2] * kDegToRad, Eigen::Vector3d::UnitZ());
        R_body_imu_ = (yaw_aa * pitch_aa * roll_aa).toRotationMatrix();
    } else {
        R_body_imu_.setIdentity();
        RCLCPP_WARN(this->get_logger(),
            "Parameter 'imu_mount_rpy_deg' must have 3 values; using identity.");
    }

    const auto lever_param = this->get_parameter("imu_lever_arm").as_double_array();
    if (lever_param.size() == 3) {
        robox_solver_.setImuLeverArm(
            Eigen::Vector3d(lever_param[0], lever_param[1], lever_param[2]));
    }

    // Business defaults
    this->declare_parameter<double>("default_payload_mass", 0.0);
    this->declare_parameter<int>("default_posture_mode", 0);
    this->declare_parameter<int>("default_climb_mode", 1);
    this->declare_parameter<double>("default_curb_height", 0.075);

    robox_solver_.setPayloadMass(this->get_parameter("default_payload_mass").as_double());
    const int init_posture = this->get_parameter("default_posture_mode").as_int();
    if (init_posture >= 0 && init_posture <= 3) {
        robox_solver_.setPostureMode(static_cast<PostureConfiguration>(init_posture));
    }
    const int init_climb = this->get_parameter("default_climb_mode").as_int();
    if (init_climb >= 0 && init_climb <= 2) {
        robox_solver_.setClimbMode(static_cast<ClimbMode>(init_climb));
    }
    robox_solver_.setCurbHeight(this->get_parameter("default_curb_height").as_double());
}

void DmLkRoboxChassisController::init_communications()
{
    const auto sensor_qos = rclcpp::SensorDataQoS();

    cmd_sub_ = this->create_subscription<chassis_controllers::msg::RoboxChassisControl>(
        cmd_topic_, sensor_qos,
        [this](const chassis_controllers::msg::RoboxChassisControl::SharedPtr msg) {
            command_callback(msg);
        });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, sensor_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            imu_callback(msg);
        });

    for (int i = 0; i < NUM_LEGS; ++i) {
        steer_subs_[i] = this->create_subscription<custom_msgs::msg::ReadDmMotor>(
            steer_read_topics_[i], sensor_qos,
            [this, i](const custom_msgs::msg::ReadDmMotor::SharedPtr msg) {
                dm_motor_callback(i, true, msg);
            });

        ecc_subs_[i] = this->create_subscription<custom_msgs::msg::ReadDmMotor>(
            ecc_read_topics_[i], sensor_qos,
            [this, i](const custom_msgs::msg::ReadDmMotor::SharedPtr msg) {
                dm_motor_callback(i, false, msg);
            });

        wheel_subs_[i] = this->create_subscription<custom_msgs::msg::ReadLkMotor>(
            wheel_read_topics_[i], sensor_qos,
            [this, i](const custom_msgs::msg::ReadLkMotor::SharedPtr msg) {
                wheel_motor_callback(i, msg);
            });

        steer_pubs_[i] = this->create_publisher<custom_msgs::msg::WriteDmMotorMITControl>(
            steer_write_topics_[i], sensor_qos);
        ecc_pubs_[i] = this->create_publisher<custom_msgs::msg::WriteDmMotorMITControl>(
            ecc_write_topics_[i], sensor_qos);
        wheel_pubs_[i] =
            this->create_publisher<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>(
                wheel_write_topics_[i], sensor_qos);
    }

    control_timer_ = this->create_wall_timer(10ms, [this] { control_loop(); });
}

void DmLkRoboxChassisController::command_callback(
    const chassis_controllers::msg::RoboxChassisControl::SharedPtr msg)
{
    last_cmd_time_ = this->now();

    const bool req_estop = (msg->emergency_stop != 0);
    if (req_estop && !command_state_.emergency_stop) {
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED! Disabling all 12 motors.");
        command_state_.emergency_stop = true;
        command_state_.cmd_vel.setZero();
        command_state_.spin_request = 0;
        publish_disable_all_motors();
        robox_solver_.resetOnEmergencyStop();
        return;
    } else if (!req_estop && command_state_.emergency_stop) {
        RCLCPP_INFO(this->get_logger(), "Emergency stop cleared.");
        command_state_.emergency_stop = false;
    }

    if (command_state_.emergency_stop) {
        return;
    }

    command_state_.cmd_vel(0)   = std::clamp(static_cast<double>(msg->x_speed), -1.2, 1.2);
    command_state_.cmd_vel(1)   = std::clamp(static_cast<double>(msg->y_speed), -1.2, 1.2);
    command_state_.cmd_vel(2)   = std::clamp(static_cast<double>(msg->yaw_speed), -2.0, 2.0);
    command_state_.spin_request = (msg->spin_mode == -1 || msg->spin_mode == 1)
                                ? static_cast<int>(msg->spin_mode)
                                : 0;

    if (msg->body_height >= 0.08f) {
        command_state_.target_height =
            std::clamp(static_cast<double>(msg->body_height), 0.08, 0.22);
    }
    command_state_.target_roll  = std::clamp(static_cast<double>(msg->body_roll), -0.35, 0.35);
    command_state_.target_pitch = std::clamp(static_cast<double>(msg->body_pitch), -0.35, 0.35);

    if (msg->posture_mode <= 3) {
        if (robox_solver_.setPostureMode(static_cast<PostureConfiguration>(msg->posture_mode))) {
            RCLCPP_INFO(this->get_logger(), "Posture mode changed to: %u", msg->posture_mode);
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Invalid posture_mode: %u (expected 0..3)", msg->posture_mode);
    }

    if (msg->climb_mode <= 2) {
        if (robox_solver_.setClimbMode(static_cast<ClimbMode>(msg->climb_mode))) {
            RCLCPP_INFO(this->get_logger(), "Climb mode changed to: %u", msg->climb_mode);
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Invalid climb_mode: %u (expected 0..2)", msg->climb_mode);
    }

    if (msg->target_curb_height > 0.0f) {
        if (robox_solver_.setCurbHeight(static_cast<double>(msg->target_curb_height))) {
            RCLCPP_INFO(this->get_logger(), "Curb height baseline updated: %.3f m",
                        static_cast<double>(msg->target_curb_height));
        }
    }

    if (msg->payload_mass >= 0.0f) {
        if (robox_solver_.setPayloadMass(static_cast<double>(msg->payload_mass))) {
            RCLCPP_INFO(this->get_logger(), "Payload mass updated: %.2f kg",
                        static_cast<double>(msg->payload_mass));
        }
    }
}

void DmLkRoboxChassisController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const Eigen::Vector3d raw_acc(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    if (!raw_acc.allFinite() || raw_acc.norm() < 0.5) {
        return;
    }

    const Eigen::Vector3d raw_gyro(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z);
    if (!raw_gyro.allFinite()) {
        return;
    }

    ImuData imu;
    imu.linear_acceleration = R_body_imu_ * raw_acc;
    imu.angular_velocity    = R_body_imu_ * raw_gyro;

    const bool quat_valid = msg->orientation_covariance[0] != -1.0 &&
                            (msg->orientation.w != 0.0 ||
                             msg->orientation.x != 0.0 ||
                             msg->orientation.y != 0.0 ||
                             msg->orientation.z != 0.0);
    if (quat_valid) {
        Eigen::Quaterniond q_world_imu(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z);
        q_world_imu.normalize();
        const Eigen::Matrix3d R_world_body =
            q_world_imu.toRotationMatrix() * R_body_imu_.transpose();
        imu.orientation     = Eigen::Quaterniond(R_world_body).normalized();
        imu.has_orientation = true;
    } else {
        imu.orientation     = Eigen::Quaterniond::Identity();
        imu.has_orientation = false;
    }

    sensor_state_.imu          = imu;
    sensor_state_.imu_received = true;
    last_imu_time_             = this->now();
}

void DmLkRoboxChassisController::dm_motor_callback(
    const int leg_idx, const bool is_steer,
    const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
{
    const auto now = this->now();
    const char* group = is_steer ? "steer" : "ecc";
    const bool has_fault = (msg->overvoltage || msg->undervoltage || msg->overcurrent ||
                            msg->mos_overtemperature || msg->rotor_overtemperature ||
                            msg->communication_lost || msg->overload);
    if (!msg->online || has_fault) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[DM %s[%d]] online=%u fault(ov=%u uv=%u oc=%u mosT=%u rotT=%u lost=%u load=%u)",
            group, leg_idx, msg->online,
            msg->overvoltage, msg->undervoltage, msg->overcurrent,
            msg->mos_overtemperature, msg->rotor_overtemperature,
            msg->communication_lost, msg->overload);
    }

    const double pos    = static_cast<double>(msg->position);
    const double vel    = static_cast<double>(msg->velocity);
    const double torque = static_cast<double>(msg->torque);

    if (is_steer) {
        raw_steer_pos_[leg_idx]                   = pos;
        sensor_state_.steer_angles(leg_idx)       = normalize_angle(
            (pos - steer_offsets_[leg_idx]) * steer_signs_[leg_idx]);
        sensor_state_.steer_velocities(leg_idx)   = vel * steer_signs_[leg_idx];
        steer_received_[leg_idx]                  = true;
        steer_online_[leg_idx]                    = (msg->online != 0 && !has_fault);
        last_steer_time_[leg_idx]                 = now;
    } else {
        raw_ecc_pos_[leg_idx]                     = pos;
        sensor_state_.ecc_angles(leg_idx)         = normalize_angle(
            (pos - ecc_offsets_[leg_idx]) * ecc_signs_[leg_idx]);
        sensor_state_.ecc_velocities(leg_idx)     = vel * ecc_signs_[leg_idx];
        sensor_state_.ecc_efforts(leg_idx)        = torque * ecc_signs_[leg_idx];
        ecc_received_[leg_idx]                    = true;
        ecc_online_[leg_idx]                      = (msg->online != 0 && !has_fault);
        last_ecc_time_[leg_idx]                   = now;
    }

    if (!joints_initialized_ && all_motors_initialized()) {
        joints_initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
            "All 12 motors online and initialized (4x DM steer, 4x DM ecc, 4x LK wheel).");
    }
}

void DmLkRoboxChassisController::wheel_motor_callback(
    const int leg_idx, const custom_msgs::msg::ReadLkMotor::SharedPtr msg)
{
    const auto now = this->now();
    if (!msg->online) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[LK wheel[%d]] motor offline (enabled=%u, temp=%u C)",
            leg_idx, msg->enabled, msg->temperature);
    }

    // ReadLkMotor speed: 1 deg/s; current (MF series): 33/4096 A per LSB
    constexpr double kDegToRad        = M_PI / 180.0;
    constexpr double kLkMfCurrentScale = 33.0 / 4096.0;

    const double raw_vel_rad_s = static_cast<double>(msg->speed) * kDegToRad;
    const double raw_torque_nm =
        static_cast<double>(msg->current) * kLkMfCurrentScale * wheel_kt_;

    sensor_state_.wheel_velocities(leg_idx) = raw_vel_rad_s * wheel_signs_[leg_idx];
    sensor_state_.wheel_efforts(leg_idx)    = raw_torque_nm * wheel_signs_[leg_idx];
    wheel_received_[leg_idx]                = true;
    wheel_online_[leg_idx]                  = (msg->online != 0);
    last_wheel_time_[leg_idx]               = now;

    if (!joints_initialized_ && all_motors_initialized()) {
        joints_initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
            "All 12 motors online and initialized (4x DM steer, 4x DM ecc, 4x LK wheel).");
    }
}

bool DmLkRoboxChassisController::all_motors_initialized() const
{
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!steer_received_[i] || !ecc_received_[i] || !wheel_received_[i]) {
            return false;
        }
    }
    return true;
}

bool DmLkRoboxChassisController::check_motors_healthy(
    const rclcpp::Time& now, double& oldest_stamp_sec) const
{
    rclcpp::Time oldest = now;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!steer_received_[i] || !ecc_received_[i] || !wheel_received_[i]) {
            return false;
        }
        if (!steer_online_[i] || !ecc_online_[i] || !wheel_online_[i]) {
            return false;
        }
        if (last_steer_time_[i] < oldest) oldest = last_steer_time_[i];
        if (last_ecc_time_[i] < oldest)   oldest = last_ecc_time_[i];
        if (last_wheel_time_[i] < oldest) oldest = last_wheel_time_[i];
    }

    oldest_stamp_sec = oldest.seconds();
    if (motor_watchdog_timeout_ > 0.0 &&
        (now - oldest).seconds() > motor_watchdog_timeout_) {
        return false;
    }
    return true;
}

void DmLkRoboxChassisController::publish_disable_all_motors()
{
    custom_msgs::msg::WriteDmMotorMITControl dm_off;
    dm_off.enable = 0;
    dm_off.p_des  = 0.0f;
    dm_off.v_des  = 0.0f;
    dm_off.kp     = 0.0f;
    dm_off.kd     = 0.0f;
    dm_off.torque = 0.0f;

    custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit lk_off;
    lk_off.enable       = 0;
    lk_off.torque_limit = 0;
    lk_off.speed        = 0;

    for (int i = 0; i < NUM_LEGS; ++i) {
        steer_pubs_[i]->publish(dm_off);
        ecc_pubs_[i]->publish(dm_off);
        wheel_pubs_[i]->publish(lk_off);
    }
}

void DmLkRoboxChassisController::control_loop()
{
    const auto now = this->now();
    const double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 0.5) {
        return;
    }
    if (!joints_initialized_) {
        return;
    }

    // 1. Runaway / Motor offline / Timeout protection
    double oldest_motor_stamp_sec = now.seconds();
    if (!check_motors_healthy(now, oldest_motor_stamp_sec)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Motor offline/fault or feedback timeout (> %.3f s); disabling all 12 motors.",
            motor_watchdog_timeout_);
        publish_disable_all_motors();
        robox_solver_.resetOnWatchdogTimeout();
        return;
    }

    // 2. Command timeout guard (zeros twist & spin, preserves posture/climb params)
    if (cmd_timeout_ > 0.0 && (now - last_cmd_time_).seconds() > cmd_timeout_) {
        command_state_.cmd_vel.setZero();
        command_state_.spin_request = 0;
    }

    // 3. Emergency stop / Master enable switch
    if (command_state_.emergency_stop || !enable_motors_) {
        publish_disable_all_motors();
        return;
    }

    // 4. Whole-body solver update
    sensor_state_.imu_age_sec   = (now - last_imu_time_).seconds();
    sensor_state_.joint_age_sec = now.seconds() - oldest_motor_stamp_sec;

    const RoboxSolverOutput out = robox_solver_.update(sensor_state_, command_state_, dt);

    if (out.imu_just_calibrated) {
        const auto b_a = robox_solver_.imuAccelBias();
        const auto b_g = robox_solver_.imuGyroBias();
        RCLCPP_INFO(this->get_logger(),
            "Static IMU calibration complete! Accel bias: [%.3f, %.3f, %.3f], Gyro bias: [%.4f, %.4f, %.4f]",
            b_a.x(), b_a.y(), b_a.z(), b_g.x(), b_g.y(), b_g.z());
    }
    if (out.homing_just_completed) {
        RCLCPP_INFO(this->get_logger(), "Homing complete. Full control pipeline active.");
    }
    if (out.fsm_state_changed) {
        RCLCPP_INFO(this->get_logger(),
            "[Adaptive FSM] %s -> %s | Overrides: [FL:%d FR:%d RL:%d RR:%d]",
            out.prev_fsm_state.c_str(), out.curr_fsm_state.c_str(),
            out.ecc_override_mask[FL], out.ecc_override_mask[FR],
            out.ecc_override_mask[RL], out.ecc_override_mask[RR]);
    }

    // 5. Solver NaN / Inf safety guard
    if (out.unsafe) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "RoboX solver produced non-finite targets! Disabling all 12 motors.");
        publish_disable_all_motors();
        return;
    }

    // 6. Direct motor command packing & publishing
    constexpr double kRadToDeg = 180.0 / M_PI;
    const double safe_kt       = std::max(wheel_kt_, 1e-4);
    const double safe_imax     = std::max(wheel_mf_max_current_a_, 1e-4);
    const int16_t wheel_torque_limit_raw = static_cast<int16_t>(std::clamp(
        std::llround((wheel_max_torque_nm_ / safe_kt) / safe_imax * 2048.0),
        1LL, 2048LL));

    for (int i = 0; i < NUM_LEGS; ++i) {
        // Steering: DM-J4310P-2EC (MIT position/damping mode)
        const double raw_steer_target =
            out.target_steer_angles(i) * steer_signs_[i] + steer_offsets_[i];
        const auto mit_steer = controllers::steeringMitCommand(
            raw_steer_target, raw_steer_pos_[i], 0.0,
            dm_steer_kp_, dm_steer_kd_, dm_steer_pmax_);

        custom_msgs::msg::WriteDmMotorMITControl steer_msg;
        steer_msg.enable = 1;
        steer_msg.p_des  = static_cast<float>(mit_steer.position);
        steer_msg.v_des  = static_cast<float>(mit_steer.velocity);
        steer_msg.kp     = static_cast<float>(mit_steer.kp);
        steer_msg.kd     = static_cast<float>(mit_steer.kd);
        steer_msg.torque = static_cast<float>(mit_steer.torque_feedforward);
        steer_pubs_[i]->publish(steer_msg);

        // Eccentric: DM-J4340P-2EC V1.1 (MIT position + feedforward torque, 12 Nm rated)
        const double raw_ecc_target =
            out.target_ecc_angles(i) * ecc_signs_[i] + ecc_offsets_[i];
        const double effort_cmd = use_ecc_torque_ff_
            ? std::clamp(out.target_ecc_torques(i) * ecc_signs_[i], -12.0, 12.0)
            : 0.0;

        auto mit_ecc = controllers::eccentricMitCommand(
            raw_ecc_target, raw_ecc_pos_[i],
            dm_ecc_kp_, dm_ecc_kd_, effort_cmd, dm_ecc_pmax_);
        controllers::limitMitFeedforward(
            mit_ecc, raw_ecc_pos_[i],
            sensor_state_.ecc_velocities(i) * ecc_signs_[i], 12.0);

        custom_msgs::msg::WriteDmMotorMITControl ecc_msg;
        ecc_msg.enable = 1;
        ecc_msg.p_des  = static_cast<float>(mit_ecc.position);
        ecc_msg.v_des  = static_cast<float>(mit_ecc.velocity);
        ecc_msg.kp     = static_cast<float>(mit_ecc.kp);
        ecc_msg.kd     = static_cast<float>(mit_ecc.kd);
        ecc_msg.torque = static_cast<float>(mit_ecc.torque_feedforward);
        ecc_pubs_[i]->publish(ecc_msg);

        // Wheels: LK MF9010 V2 17T (Speed With Torque Limit, unit: 0.01 deg/s)
        const double raw_wheel_vel = out.target_wheel_speeds(i) * wheel_signs_[i];
        custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit wheel_msg;
        wheel_msg.enable       = 1;
        wheel_msg.torque_limit = wheel_torque_limit_raw;
        wheel_msg.speed        = static_cast<int32_t>(std::llround(raw_wheel_vel * kRadToDeg * 100.0));
        wheel_pubs_[i]->publish(wheel_msg);
    }
}

int main(const int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<DmLkRoboxChassisController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
