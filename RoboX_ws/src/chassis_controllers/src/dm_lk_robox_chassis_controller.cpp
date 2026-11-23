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
    last_imu_time_ = last_time_;

    RCLCPP_INFO(this->get_logger(),
        "DM/LK RoboX chassis controller started. Waiting for motor feedbacks...");
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

    this->declare_parameter<double>("dm_steer_speed_limit", 12.0);
    this->declare_parameter<double>("dm_ecc_speed_limit", 3.5);
    this->declare_parameter<double>("wheel_speed_kp", 15.0);
    this->declare_parameter<double>("wheel_speed_ki", 0.5);
    this->declare_parameter<double>("wheel_kt", 0.16);
    this->declare_parameter<double>("wheel_max_torque_nm", 2.2);
    this->declare_parameter<double>("wheel_mf_max_current_a", 16.5);

    dm_steer_speed_limit_   = this->get_parameter("dm_steer_speed_limit").as_double();
    dm_ecc_speed_limit_     = this->get_parameter("dm_ecc_speed_limit").as_double();
    wheel_speed_kp_         = this->get_parameter("wheel_speed_kp").as_double();
    wheel_speed_ki_         = this->get_parameter("wheel_speed_ki").as_double();
    wheel_kt_               = this->get_parameter("wheel_kt").as_double();
    wheel_max_torque_nm_    = this->get_parameter("wheel_max_torque_nm").as_double();
    wheel_mf_max_current_a_ = this->get_parameter("wheel_mf_max_current_a").as_double();

    robox_solver_.setWheelPiParams(wheel_speed_kp_, wheel_speed_ki_, wheel_max_torque_nm_);

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
    this->declare_parameter<std::string>("wheel_motor_read_topic", "wheel_motor_read");
    this->declare_parameter<std::string>("wheel_motor_write_topic", "wheel_motor_write");

    cmd_topic_         = this->get_parameter("chassis_cmd_read_topic").as_string();
    imu_topic_         = this->get_parameter("imu_read_topic").as_string();
    wheel_read_topic_  = this->get_parameter("wheel_motor_read_topic").as_string();
    wheel_write_topic_ = this->get_parameter("wheel_motor_write_topic").as_string();

    steer_read_topics_  = load_topics_param("steer_motor_read_topics",  make_default_topics("steer", "read"));
    steer_write_topics_ = load_topics_param("steer_motor_write_topics", make_default_topics("steer", "write"));
    ecc_read_topics_    = load_topics_param("ecc_motor_read_topics",    make_default_topics("ecc", "read"));
    ecc_write_topics_   = load_topics_param("ecc_motor_write_topics",   make_default_topics("ecc", "write"));

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

        steer_pubs_[i] =
            this->create_publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>(
                steer_write_topics_[i], sensor_qos);
        ecc_pubs_[i] =
            this->create_publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>(
                ecc_write_topics_[i], sensor_qos);
    }

    wheel_sub_ = this->create_subscription<custom_msgs::msg::ReadLkMotorMulti>(
        wheel_read_topic_, sensor_qos,
        [this](const custom_msgs::msg::ReadLkMotorMulti::SharedPtr msg) {
            wheel_motor_callback(msg);
        });

    wheel_pub_ =
        this->create_publisher<custom_msgs::msg::WriteLkMotorBroadcastCurrentControl>(
            wheel_write_topic_, sensor_qos);

    control_timer_ = this->create_wall_timer(10ms, [this] { control_loop(); });
}

void DmLkRoboxChassisController::command_callback(
    const chassis_controllers::msg::RoboxChassisControl::SharedPtr msg)
{
    const bool req_estop = (msg->emergency_stop != 0);
    if (req_estop) {
        if (!command_state_.emergency_stop) {
            RCLCPP_WARN(this->get_logger(),
                "EMERGENCY STOP ACTIVATED! Disabling DM motors and braking LK wheels to stop.");
        }
        command_state_.emergency_stop = true;
        command_state_.cmd_vel.setZero();
        command_state_.spin_request = 0;
        robox_solver_.resetOnEmergencyStop();
        publish_emergency_stop_commands();
        return;
    }

    if (command_state_.emergency_stop) {
        RCLCPP_INFO(this->get_logger(),
            "Emergency stop cleared. Resuming smooth startup from current joint positions.");
        command_state_.emergency_stop = false;
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
        sensor_state_.steer_angles(leg_idx)     = normalize_angle(
            (pos - steer_offsets_[leg_idx]) * steer_signs_[leg_idx]);
        sensor_state_.steer_velocities(leg_idx) = vel * steer_signs_[leg_idx];
        steer_received_[leg_idx]                = true;
    } else {
        sensor_state_.ecc_angles(leg_idx)       = normalize_angle(
            (pos - ecc_offsets_[leg_idx]) * ecc_signs_[leg_idx]);
        sensor_state_.ecc_velocities(leg_idx)   = vel * ecc_signs_[leg_idx];
        sensor_state_.ecc_efforts(leg_idx)      = torque * ecc_signs_[leg_idx];
        ecc_received_[leg_idx]                  = true;
    }

    if (!joints_initialized_ && all_motors_initialized()) {
        joints_initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
            "All 12 motors online and initialized (4x DM steer, 4x DM ecc, 4x LK wheel).");
    }
}

void DmLkRoboxChassisController::wheel_motor_callback(
    const custom_msgs::msg::ReadLkMotorMulti::SharedPtr msg)
{
    const std::array<uint8_t, NUM_LEGS> onlines = {
        msg->motor1_online, msg->motor2_online, msg->motor3_online, msg->motor4_online
    };
    const std::array<int16_t, NUM_LEGS> speeds = {
        msg->motor1_speed, msg->motor2_speed, msg->motor3_speed, msg->motor4_speed
    };
    const std::array<int16_t, NUM_LEGS> currents = {
        msg->motor1_current, msg->motor2_current, msg->motor3_current, msg->motor4_current
    };

    // ReadLkMotorMulti speed: 1 deg/s; current (MF series): 33/4096 A per LSB
    constexpr double kDegToRad         = M_PI / 180.0;
    constexpr double kLkMfCurrentScale = 33.0 / 4096.0;

    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!onlines[i]) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[LK wheel[%d]] motor offline", i);
        }
        const double raw_vel_rad_s = static_cast<double>(speeds[i]) * kDegToRad;
        const double raw_torque_nm =
            static_cast<double>(currents[i]) * kLkMfCurrentScale * wheel_kt_;
        sensor_state_.wheel_velocities(i) = raw_vel_rad_s * wheel_signs_[i];
        sensor_state_.wheel_efforts(i)    = raw_torque_nm * wheel_signs_[i];
    }

    wheel_received_.fill(true);

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

int16_t DmLkRoboxChassisController::torque_to_broadcast_cmd(const double torque_nm) const
{
    const double clamped_torque = std::clamp(torque_nm, -wheel_max_torque_nm_, wheel_max_torque_nm_);
    const double current_a      = clamped_torque / std::max(wheel_kt_, 1e-4);
    const double raw_cmd        = current_a / std::max(wheel_mf_max_current_a_, 1e-4) * 2000.0;
    return static_cast<int16_t>(std::clamp(std::llround(raw_cmd), -2000LL, 2000LL));
}

void DmLkRoboxChassisController::publish_emergency_stop_commands()
{
    custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit dm_off;
    dm_off.enable   = 0;
    dm_off.position = 0.0f;
    dm_off.speed    = 0.0f;

    for (int i = 0; i < NUM_LEGS; ++i) {
        steer_pubs_[i]->publish(dm_off);
        ecc_pubs_[i]->publish(dm_off);
    }

    const bool wheels_moving =
        sensor_state_.wheel_velocities.cwiseAbs().maxCoeff() > kWheelStopThresholdRadPerSec;

    custom_msgs::msg::WriteLkMotorBroadcastCurrentControl wheel_msg;
    if (wheels_moving) {
        std::array<int16_t, NUM_LEGS> cmds{};
        for (int i = 0; i < NUM_LEGS; ++i) {
            const double brake_tau_cad = std::clamp(
                -wheel_speed_kp_ * sensor_state_.wheel_velocities(i),
                -wheel_max_torque_nm_, wheel_max_torque_nm_);
            cmds[i] = torque_to_broadcast_cmd(brake_tau_cad * wheel_signs_[i]);
        }
        wheel_msg.motor1_cmd = cmds[0];
        wheel_msg.motor2_cmd = cmds[1];
        wheel_msg.motor3_cmd = cmds[2];
        wheel_msg.motor4_cmd = cmds[3];
    } else {
        wheel_msg.motor1_cmd = 0;
        wheel_msg.motor2_cmd = 0;
        wheel_msg.motor3_cmd = 0;
        wheel_msg.motor4_cmd = 0;
    }
    wheel_pub_->publish(wheel_msg);
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

    // 1. Manual emergency stop: disable DM steering/eccentric immediately, brake LK wheels then zero
    if (command_state_.emergency_stop) {
        robox_solver_.resetOnEmergencyStop();
        publish_emergency_stop_commands();
        return;
    }

    // 2. Whole-body solver update
    sensor_state_.imu_age_sec   = (now - last_imu_time_).seconds();
    sensor_state_.joint_age_sec = 0.0;

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

    // 3. Solver NaN / Inf safety guard
    if (out.unsafe) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "RoboX solver produced non-finite targets! Activating emergency stop output.");
        robox_solver_.resetOnEmergencyStop();
        publish_emergency_stop_commands();
        return;
    }

    // 4. Direct motor command packing & publishing
    for (int i = 0; i < NUM_LEGS; ++i) {
        custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit steer_msg;
        steer_msg.enable   = 1;
        steer_msg.position = static_cast<float>(
            out.target_steer_angles(i) * steer_signs_[i] + steer_offsets_[i]);
        steer_msg.speed    = static_cast<float>(dm_steer_speed_limit_);
        steer_pubs_[i]->publish(steer_msg);

        custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit ecc_msg;
        ecc_msg.enable   = 1;
        ecc_msg.position = static_cast<float>(
            out.target_ecc_angles(i) * ecc_signs_[i] + ecc_offsets_[i]);
        ecc_msg.speed    = static_cast<float>(dm_ecc_speed_limit_);
        ecc_pubs_[i]->publish(ecc_msg);
    }

    custom_msgs::msg::WriteLkMotorBroadcastCurrentControl wheel_msg;
    wheel_msg.motor1_cmd = torque_to_broadcast_cmd(out.target_wheel_torques(0) * wheel_signs_[0]);
    wheel_msg.motor2_cmd = torque_to_broadcast_cmd(out.target_wheel_torques(1) * wheel_signs_[1]);
    wheel_msg.motor3_cmd = torque_to_broadcast_cmd(out.target_wheel_torques(2) * wheel_signs_[2]);
    wheel_msg.motor4_cmd = torque_to_broadcast_cmd(out.target_wheel_torques(3) * wheel_signs_[3]);
    wheel_pub_->publish(wheel_msg);
}

int main(const int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<DmLkRoboxChassisController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
