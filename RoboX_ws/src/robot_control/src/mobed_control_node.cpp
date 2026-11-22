#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_interfaces/msg/mob_ed_command.hpp"

// EcatV2_Master hardware messages (12 real motors + SBUS RC)
#include "custom_msgs/msg/read_dm_motor.hpp"
#include "custom_msgs/msg/read_lk_motor.hpp"
#include "custom_msgs/msg/read_sbusrc.hpp"
#include "custom_msgs/msg/write_dm_motor_mit_control.hpp"
#include "custom_msgs/msg/write_lk_motor_speed_control_with_torque_limit.hpp"

// Core
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"

// Kinematics & Controllers
#include "robot_control/controllers/driving_controller.hpp"
#include "robot_control/controllers/mit_servo.hpp"
#include "robot_control/controllers/spin_drive.hpp"
#include "robot_control/controllers/teleop_smoother.hpp"
#include "robot_control/controllers/balance_controller.hpp"

// Estimation
#include "robot_control/estimation/state_estimator.hpp"
#include "robot_control/estimation/terrain_estimator.hpp"
#include "robot_control/estimation/contact_detector.hpp"
#include "robot_control/estimation/climb_geometry_estimator.hpp"

// Tasks
#include "robot_control/tasks/task_controller.hpp"

using namespace std::chrono_literals;

namespace robot_control {

class MobedControlNode : public rclcpp::Node {
public:
    MobedControlNode() : Node("mobed_control_node") {
        // ============================================================
        // Initialize shared parameters and kinematics
        // ============================================================
        robot_params_ = RobotParams();
        kinematics_ = std::make_shared<kinematics::MobedKinematics>();

        // ============================================================
        // Initialize all algorithm modules
        // ============================================================
        controllers::DrivingControllerParams drive_params;

        driving_controller_  = std::make_unique<controllers::DrivingController>(drive_params, kinematics_);
        state_estimator_     = std::make_unique<estimation::StateEstimator>(robot_params_);
        state_estimator_->startStaticCalibration(200);
        terrain_estimator_   = std::make_unique<estimation::TerrainEstimator>(robot_params_);
        contact_detector_    = std::make_unique<estimation::ContactDetector>(robot_params_);
        climb_geometry_estimator_ = std::make_unique<estimation::ClimbGeometryEstimator>(robot_params_);
        task_controller_     = std::make_unique<tasks::TaskController>();
        task_controller_->setClimbMode(climb_mode_);

        // ============================================================
        // Initialize state variables
        // ============================================================
        curr_steer_angles_.setZero();
        curr_steer_velocities_.setZero();
        curr_ecc_angles_.setZero();
        curr_ecc_velocities_.setZero();
        curr_ecc_efforts_.setZero();
        curr_wheel_efforts_.setZero();
        curr_wheel_velocities_.setZero();
        cmd_vel_.setZero();

        // Polarity & Zero-Offset Mapping Setup (FL, FR, RL, RR)
        this->declare_parameter<std::string>("robot_model", "robox_v2");
        std::string robot_model = this->get_parameter("robot_model").as_string();

        if (robot_model == "legacy" || robot_model == "mobed_legacy") {
            steer_signs_   = {1.0, 1.0, 1.0, 1.0};
            steer_offsets_ = {0.0, 0.0, 0.0, 0.0};
            ecc_signs_     = {1.0, -1.0, -1.0, 1.0};
            ecc_offsets_   = {-1.9003, -3.0526, 1.5320, -3.0473};
            wheel_signs_   = {1.0, -1.0, 1.0, -1.0};
            robot_params_.applyModel(RobotModelType::MOBED_LEGACY);
        } else {
            // ROBOX V2 Standard (default)
            steer_signs_   = {1.0, 1.0, 1.0, 1.0};
            steer_offsets_ = {0.0, 0.0, 0.0, 0.0};
            ecc_signs_     = {1.0, -1.0, 1.0, -1.0};
            ecc_offsets_   = {M_PI, M_PI, M_PI, M_PI};
            wheel_signs_   = {-1.0, 1.0, -1.0, 1.0};
            robot_params_.applyModel(RobotModelType::ROBOX_V2);
        }
        nominal_drive_height_ = robot_params_.model_type == RobotModelType::ROBOX_V2 ? 0.175 : 0.205;
        target_height_ = nominal_drive_height_;
        balance_controller_ = std::make_unique<controllers::BalanceController>(
            controllers::BalanceControllerParams{}, kinematics_, robot_params_);

        // Allow overriding polarities & zero offsets from YAML for real-robot calibration
        auto loadVec4Param = [this](const std::string& name, std::vector<double>& target) {
            this->declare_parameter<std::vector<double>>(name, target);
            auto val = this->get_parameter(name).as_double_array();
            if (val.size() == NUM_LEGS) {
                target = val;
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Parameter '%s' must have 4 elements (got %zu); keeping default.", name.c_str(), val.size());
            }
        };
        loadVec4Param("steer_signs", steer_signs_);
        loadVec4Param("steer_offsets", steer_offsets_);
        loadVec4Param("ecc_signs", ecc_signs_);
        loadVec4Param("ecc_offsets", ecc_offsets_);
        loadVec4Param("wheel_signs", wheel_signs_);

        // Real-hardware actuator parameters (DM-J4310P-2EC / DM-J4340P-2EC V1.1 / LK MF9010 V2 17T)
        this->declare_parameter<bool>("enable_motors", false);
        this->declare_parameter<bool>("use_ecc_torque_ff", true);
        this->declare_parameter<double>("motor_watchdog_timeout", 0.25);

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
        dm_steer_kp_            = this->get_parameter("dm_steer_kp").as_double();
        dm_steer_kd_            = this->get_parameter("dm_steer_kd").as_double();
        dm_steer_pmax_          = this->get_parameter("dm_steer_pmax").as_double();
        dm_ecc_kp_              = this->get_parameter("dm_ecc_kp").as_double();
        dm_ecc_kd_              = this->get_parameter("dm_ecc_kd").as_double();
        dm_ecc_pmax_            = this->get_parameter("dm_ecc_pmax").as_double();
        wheel_kt_               = this->get_parameter("wheel_kt").as_double();
        wheel_max_torque_nm_    = this->get_parameter("wheel_max_torque_nm").as_double();
        wheel_mf_max_current_a_ = this->get_parameter("wheel_mf_max_current_a").as_double();

        static constexpr std::array<const char*, NUM_LEGS> kLegNames = {"lf", "rf", "lb", "rb"};
        auto makeDefaultTopics = [&](const std::string& group, const std::string& suffix) {
            std::vector<std::string> out;
            out.reserve(NUM_LEGS);
            for (const char* leg : kLegNames) {
                out.push_back("/ecat/" + group + "_" + leg + "/" + suffix);
            }
            return out;
        };
        auto loadTopicsParam = [&](const std::string& name, const std::vector<std::string>& defaults) {
            this->declare_parameter<std::vector<std::string>>(name, defaults);
            auto val = this->get_parameter(name).as_string_array();
            if (val.size() == NUM_LEGS) {
                return val;
            }
            RCLCPP_WARN(this->get_logger(),
                "Parameter '%s' must have 4 topics (got %zu); using defaults.", name.c_str(), val.size());
            return defaults;
        };

        auto steer_state_topics = loadTopicsParam("steer_state_topics", makeDefaultTopics("steer", "state"));
        auto steer_cmd_topics   = loadTopicsParam("steer_cmd_topics",   makeDefaultTopics("steer", "cmd"));
        auto ecc_state_topics   = loadTopicsParam("ecc_state_topics",   makeDefaultTopics("ecc", "state"));
        auto ecc_cmd_topics     = loadTopicsParam("ecc_cmd_topics",     makeDefaultTopics("ecc", "cmd"));
        auto wheel_state_topics = loadTopicsParam("wheel_state_topics", makeDefaultTopics("wheel", "state"));
        auto wheel_cmd_topics   = loadTopicsParam("wheel_cmd_topics",   makeDefaultTopics("wheel", "cmd"));

        // IMU hardware configuration (HiPNUC CH010 + G431 CAN forwarding board)
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<bool>("require_imu_for_homing", true);
        std::vector<double> imu_rpy_deg = {0.0, 0.0, 0.0};
        std::vector<double> imu_lever_arm = {0.0, 0.0, 0.0};
        this->declare_parameter<std::vector<double>>("imu_mount_rpy_deg", imu_rpy_deg);
        this->declare_parameter<std::vector<double>>("imu_lever_arm", imu_lever_arm);

        const std::string imu_topic = this->get_parameter("imu_topic").as_string();
        require_imu_for_homing_ = this->get_parameter("require_imu_for_homing").as_bool();
        auto rpy_param = this->get_parameter("imu_mount_rpy_deg").as_double_array();
        if (rpy_param.size() == 3) {
            constexpr double kDegToRad = M_PI / 180.0;
            const Eigen::AngleAxisd roll_aa(rpy_param[0] * kDegToRad, Eigen::Vector3d::UnitX());
            const Eigen::AngleAxisd pitch_aa(rpy_param[1] * kDegToRad, Eigen::Vector3d::UnitY());
            const Eigen::AngleAxisd yaw_aa(rpy_param[2] * kDegToRad, Eigen::Vector3d::UnitZ());
            R_body_imu_ = (yaw_aa * pitch_aa * roll_aa).toRotationMatrix();
        } else {
            R_body_imu_.setIdentity();
            RCLCPP_WARN(this->get_logger(), "Parameter 'imu_mount_rpy_deg' must have 3 values; using identity.");
        }
        auto lever_param = this->get_parameter("imu_lever_arm").as_double_array();
        if (lever_param.size() == 3) {
            state_estimator_->setImuLeverArm(Eigen::Vector3d(lever_param[0], lever_param[1], lever_param[2]));
        }

        // SBUS RC & Teleop smoother configuration
        this->declare_parameter<bool>("sbus_enabled", true);
        this->declare_parameter<std::string>("sbus_topic", "/ecat/sbus_rc");
        this->declare_parameter<bool>("sbus_failsafe_estop", true);
        this->declare_parameter<double>("sbus_timeout", 0.5);
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

        controllers::TeleopSmootherParams teleop_params;
        this->declare_parameter<double>("teleop_max_vx", teleop_params.max_lin_vel);
        this->declare_parameter<double>("teleop_max_vy", teleop_params.max_strafe_vel);
        this->declare_parameter<double>("teleop_max_wz", teleop_params.max_ang_vel);
        this->declare_parameter<double>("teleop_accel_lin_fwd", teleop_params.accel_lin_fwd);
        this->declare_parameter<double>("teleop_accel_lin_brake", teleop_params.accel_lin_brake);
        this->declare_parameter<double>("teleop_accel_strafe", teleop_params.accel_strafe);
        this->declare_parameter<double>("teleop_accel_angular", teleop_params.accel_angular);
        this->declare_parameter<double>("teleop_max_lateral_acc", teleop_params.max_lateral_acc);

        sbus_enabled_        = this->get_parameter("sbus_enabled").as_bool();
        const std::string sbus_topic = this->get_parameter("sbus_topic").as_string();
        sbus_failsafe_estop_ = this->get_parameter("sbus_failsafe_estop").as_bool();
        sbus_timeout_        = this->get_parameter("sbus_timeout").as_double();
        sbus_ch_min_         = this->get_parameter("sbus_ch_min").as_double();
        sbus_ch_mid_         = this->get_parameter("sbus_ch_mid").as_double();
        sbus_ch_max_         = this->get_parameter("sbus_ch_max").as_double();
        sbus_deadband_       = this->get_parameter("sbus_deadband").as_double();
        sbus_ch_vx_          = this->get_parameter("sbus_ch_vx").as_int();
        sbus_ch_vy_          = this->get_parameter("sbus_ch_vy").as_int();
        sbus_ch_wz_          = this->get_parameter("sbus_ch_wz").as_int();
        sbus_ch_height_      = this->get_parameter("sbus_ch_height").as_int();
        sbus_ch_estop_       = this->get_parameter("sbus_ch_estop").as_int();
        sbus_ch_climb_mode_  = this->get_parameter("sbus_ch_climb_mode").as_int();
        sbus_ch_spin_        = this->get_parameter("sbus_ch_spin").as_int();
        sbus_ch_posture_     = this->get_parameter("sbus_ch_posture").as_int();
        sbus_inv_vx_         = this->get_parameter("sbus_inv_vx").as_bool();
        sbus_inv_vy_         = this->get_parameter("sbus_inv_vy").as_bool();
        sbus_inv_wz_         = this->get_parameter("sbus_inv_wz").as_bool();
        sbus_inv_height_     = this->get_parameter("sbus_inv_height").as_bool();
        sbus_inv_estop_      = this->get_parameter("sbus_inv_estop").as_bool();
        sbus_height_min_     = this->get_parameter("sbus_height_min").as_double();
        sbus_height_max_     = this->get_parameter("sbus_height_max").as_double();

        teleop_params.max_lin_vel     = this->get_parameter("teleop_max_vx").as_double();
        teleop_params.max_strafe_vel  = this->get_parameter("teleop_max_vy").as_double();
        teleop_params.max_ang_vel     = this->get_parameter("teleop_max_wz").as_double();
        teleop_params.accel_lin_fwd   = this->get_parameter("teleop_accel_lin_fwd").as_double();
        teleop_params.accel_lin_brake = this->get_parameter("teleop_accel_lin_brake").as_double();
        teleop_params.accel_strafe    = this->get_parameter("teleop_accel_strafe").as_double();
        teleop_params.accel_angular   = this->get_parameter("teleop_accel_angular").as_double();
        teleop_params.max_lateral_acc = this->get_parameter("teleop_max_lateral_acc").as_double();
        teleop_smoother_.setParams(teleop_params);

        // ============================================================
        // ROS 2 Subscriptions
        // ============================================================
        sub_cmd_ = this->create_subscription<robot_interfaces::msg::MobEDCommand>(
            "/mobed/command", 10,
            std::bind(&MobedControlNode::commandCallback, this, std::placeholders::_1));

        sub_spin_ = this->create_subscription<std_msgs::msg::Int8>(
            "/mobed/spin", 1, [this](const std_msgs::msg::Int8::SharedPtr msg) {
                spin_request_ = (msg->data == -1 || msg->data == 1) ? msg->data : 0;
                last_spin_time_ = this->now();
            });

        // Match EcatV2_Master SensorDataQoS() for IMU, SBUS RC, and all 12 motor topics
        const auto sensor_qos = rclcpp::SensorDataQoS();

        for (int i = 0; i < NUM_LEGS; ++i) {
            sub_steer_motors_[i] = this->create_subscription<custom_msgs::msg::ReadDmMotor>(
                steer_state_topics[i], sensor_qos,
                [this, i](const custom_msgs::msg::ReadDmMotor::SharedPtr msg) {
                    dmMotorCallback(i, true, msg);
                });

            sub_ecc_motors_[i] = this->create_subscription<custom_msgs::msg::ReadDmMotor>(
                ecc_state_topics[i], sensor_qos,
                [this, i](const custom_msgs::msg::ReadDmMotor::SharedPtr msg) {
                    dmMotorCallback(i, false, msg);
                });

            sub_wheel_motors_[i] = this->create_subscription<custom_msgs::msg::ReadLkMotor>(
                wheel_state_topics[i], sensor_qos,
                [this, i](const custom_msgs::msg::ReadLkMotor::SharedPtr msg) {
                    wheelMotorCallback(i, msg);
                });

            pub_steer_cmds_[i] = this->create_publisher<custom_msgs::msg::WriteDmMotorMITControl>(
                steer_cmd_topics[i], sensor_qos);
            pub_ecc_cmds_[i] = this->create_publisher<custom_msgs::msg::WriteDmMotorMITControl>(
                ecc_cmd_topics[i], sensor_qos);
            pub_wheel_cmds_[i] = this->create_publisher<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>(
                wheel_cmd_topics[i], sensor_qos);
        }

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, sensor_qos,
            std::bind(&MobedControlNode::imuCallback, this, std::placeholders::_1));

        if (sbus_enabled_) {
            sub_sbus_ = this->create_subscription<custom_msgs::msg::ReadSBUSRC>(
                sbus_topic, sensor_qos,
                std::bind(&MobedControlNode::sbusCallback, this, std::placeholders::_1));
        }

        sub_e_stop_ = this->create_subscription<std_msgs::msg::Bool>(
            "/e_stop", 10,
            std::bind(&MobedControlNode::eStopCallback, this, std::placeholders::_1));

        // Dynamic Payload Mass & Posture Configuration (IEEE RA-L 2026 Eq 11, 16, 17)
        this->declare_parameter<double>("payload_mass", 0.0);
        this->declare_parameter<int>("posture_configuration", 0);

        double init_payload = this->get_parameter("payload_mass").as_double();
        int init_config = this->get_parameter("posture_configuration").as_int();

        if (init_payload > 0.0) {
            balance_controller_->setPayloadMass(init_payload);
        }
        if (init_config >= 0 && init_config <= 3) {
            user_posture_config_ = static_cast<PostureConfiguration>(init_config);
            balance_controller_->setPostureConfiguration(user_posture_config_);
        }

        // Adaptive Obstacle Curb Height Parameter & Subscription (0.04m ~ 0.15m)
        this->declare_parameter<double>("target_curb_height", 0.075);
        double init_curb = this->get_parameter("target_curb_height").as_double();
        task_controller_->setCurbHeight(init_curb);

        sub_curb_height_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mobed/curb_height", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                double h = std::clamp(msg->data, 0.04, 0.22);
                if (!task_controller_->isActive() && !climb_geometry_estimator_->hasLatchedHeight()) {
                    RCLCPP_INFO(this->get_logger(), "Manual curb height baseline preset: %.3f m", h);
                    task_controller_->setCurbHeight(h);
                }
            });

        sub_payload_mass_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mobed/payload_mass", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                double mass = std::max(0.0, msg->data);
                RCLCPP_INFO(this->get_logger(), "Dynamic payload mass updated: %.2f kg", mass);
                balance_controller_->setPayloadMass(mass);
            });

        sub_posture_mode_int_ = this->create_subscription<std_msgs::msg::Int32>(
            "/mobed/posture_mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (msg->data >= 0 && msg->data <= 3) {
                    applyPostureMode(static_cast<PostureConfiguration>(msg->data));
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid posture configuration integer: %d (0:OUTWARD, 1:INWARD, 2:SPIN, 3:STRAFE_OUTWARD)", msg->data);
                }
            });

        sub_posture_mode_str_ = this->create_subscription<std_msgs::msg::String>(
            "/mobed/posture_mode_str", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "OUTWARD" || msg->data == "outward") {
                    applyPostureMode(PostureConfiguration::OUTWARD);
                } else if (msg->data == "INWARD" || msg->data == "inward") {
                    applyPostureMode(PostureConfiguration::INWARD);
                } else if (msg->data == "SPIN" || msg->data == "spin") {
                    applyPostureMode(PostureConfiguration::SPIN);
                } else if (msg->data == "STRAFE_OUTWARD" || msg->data == "strafe_outward") {
                    applyPostureMode(PostureConfiguration::STRAFE_OUTWARD);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid posture configuration string: '%s'", msg->data.c_str());
                }
            });

        // Climb Mode Subscription: 0 = DISABLED, 1 = PAPER_NORMAL, 2 = STAIR_200MM
        sub_climb_mode_ = this->create_subscription<std_msgs::msg::Int32>(
            "/mobed/climb_mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (msg->data >= 0 && msg->data <= 2) {
                    applyClimbMode(static_cast<ClimbMode>(msg->data));
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid climb mode: %d (0:DISABLED, 1:PAPER_NORMAL, 2:STAIR_200MM)", msg->data);
                }
            });

        // ============================================================
        // Control Loop Timer (100 Hz)
        // ============================================================
        timer_ = this->create_wall_timer(10ms,
            std::bind(&MobedControlNode::timerCallback, this));
        last_time_ = this->now();
        last_cmd_vel_time_ = last_time_;
        last_spin_time_ = last_time_;
        last_imu_time_ = last_time_;
        last_sbus_time_ = last_time_;
        last_joint_state_time_ = last_time_;
        last_steer_time_.fill(last_time_);
        last_ecc_time_.fill(last_time_);
        last_wheel_time_.fill(last_time_);

        RCLCPP_INFO(this->get_logger(),
            "MobED Control Node started (12-motor EcatV2 + HiPNUC IMU + SBUS RC=%s, enable_motors=%s). "
            "Waiting for motor feedback...",
            sbus_enabled_ ? "enabled" : "disabled",
            enable_motors_ ? "true" : "false");
    }

private:
    void applyPostureMode(PostureConfiguration cfg) {
        if (cfg == user_posture_config_) {
            return;
        }
        const char* name = (cfg == PostureConfiguration::OUTWARD)        ? "OUTWARD" :
                           (cfg == PostureConfiguration::INWARD)         ? "INWARD" :
                           (cfg == PostureConfiguration::SPIN)           ? "SPIN" : "STRAFE_OUTWARD";
        RCLCPP_INFO(this->get_logger(), "Posture configuration changed to: %s (%d)",
                    name, static_cast<int>(cfg));
        user_posture_config_ = cfg;
        balance_controller_->setPostureConfiguration(cfg);
    }

    void applyClimbMode(ClimbMode new_mode) {
        if (new_mode == climb_mode_) {
            return;
        }
        const char* mode_name = (new_mode == ClimbMode::DISABLED) ? "DISABLED" :
                                (new_mode == ClimbMode::PAPER_NORMAL ? "PAPER_NORMAL" : "STAIR_200MM");
        RCLCPP_INFO(this->get_logger(), "Climb mode changed to: %s (%d)",
                    mode_name, static_cast<int>(new_mode));
        if (climb_mode_ == ClimbMode::STAIR_200MM && new_mode != ClimbMode::STAIR_200MM) {
            if (task_controller_->isActive()) {
                task_controller_->abort();
            }
        }
        climb_mode_ = new_mode;
        task_controller_->setClimbMode(climb_mode_);
        if (climb_mode_ == ClimbMode::STAIR_200MM) {
            task_controller_->setCurbHeight(0.20);
        }
    }

    double normalizeSbusChannel(uint16_t raw, bool invert, bool apply_deadband) const {
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

    // ================================================================
    // Callback: Teleop/Navigation Command
    // ================================================================
    void commandCallback(const robot_interfaces::msg::MobEDCommand::SharedPtr msg) {
        if (sbus_enabled_ && sbus_active_) {
            return;  // Hardware SBUS RC takes priority while actively connected
        }
        cmd_vel_(0) = std::clamp(msg->twist.linear.x, -1.2, 1.2);
        cmd_vel_(1) = std::clamp(msg->twist.linear.y, -1.2, 1.2);
        cmd_vel_(2) = std::clamp(msg->twist.angular.z, -2.0, 2.0);
        last_cmd_vel_time_ = this->now();

        if (msg->body_height >= 0.08) {
            target_height_ = std::clamp(msg->body_height, 0.08, 0.22);
        }
        target_roll_   = std::clamp(msg->body_roll,  -0.35, 0.35);
        target_pitch_  = std::clamp(msg->body_pitch, -0.35, 0.35);
    }

    // ================================================================
    // Callback: SBUS RC Receiver (via EcatV2_Master ReadSBUSRC)
    // ================================================================
    void sbusCallback(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg) {
        const auto now = this->now();
        last_sbus_time_ = now;
        sbus_received_ = true;

        if (!msg->online || msg->fail_safe || msg->channels.size() < 16) {
            sbus_active_ = false;
            sbus_norm_vx_ = 0.0;
            sbus_norm_vy_ = 0.0;
            sbus_norm_wz_ = 0.0;
            spin_request_ = 0;
            if (sbus_failsafe_estop_) {
                setSbusEStop(true);
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[SBUS RC] offline=%u fail_safe=%u frame_lost=%u ch_size=%zu",
                msg->online, msg->fail_safe, msg->frame_lost, msg->channels.size());
            return;
        }

        sbus_active_ = true;

        auto readSwitch = [&](int ch_idx, bool invert = false) -> double {
            if (ch_idx < 0 || ch_idx >= static_cast<int>(msg->channels.size())) {
                return 0.0;
            }
            return normalizeSbusChannel(msg->channels[ch_idx], invert, false);
        };
        auto readStick = [&](int ch_idx, bool invert) -> double {
            if (ch_idx < 0 || ch_idx >= static_cast<int>(msg->channels.size())) {
                return 0.0;
            }
            return normalizeSbusChannel(msg->channels[ch_idx], invert, true);
        };

        // Emergency stop switch (if configured)
        if (sbus_ch_estop_ >= 0) {
            setSbusEStop(readSwitch(sbus_ch_estop_, sbus_inv_estop_) > 0.5);
        } else {
            setSbusEStop(false);
        }

        // Stick velocity commands (smoothed at 100 Hz in timerCallback)
        sbus_norm_vx_ = readStick(sbus_ch_vx_, sbus_inv_vx_);
        sbus_norm_vy_ = readStick(sbus_ch_vy_, sbus_inv_vy_);
        sbus_norm_wz_ = readStick(sbus_ch_wz_, sbus_inv_wz_);

        // Optional chassis height channel
        if (sbus_ch_height_ >= 0) {
            const double h_norm = readSwitch(sbus_ch_height_, sbus_inv_height_);
            const double h_ratio = 0.5 * (h_norm + 1.0);
            target_height_ = std::clamp(
                sbus_height_min_ + h_ratio * (sbus_height_max_ - sbus_height_min_),
                0.08, 0.22);
        }

        // 3-position Climb Mode switch: low=DISABLED, mid=PAPER_NORMAL, high=STAIR_200MM
        if (sbus_ch_climb_mode_ >= 0) {
            const double c_val = readSwitch(sbus_ch_climb_mode_);
            const ClimbMode requested_climb = (c_val < -0.4) ? ClimbMode::DISABLED :
                                              (c_val >  0.4) ? ClimbMode::STAIR_200MM :
                                                               ClimbMode::PAPER_NORMAL;
            applyClimbMode(requested_climb);
        }

        // 3-position Spin Mode switch: low=-1 (CW), mid=0 (off), high=+1 (CCW)
        if (sbus_ch_spin_ >= 0) {
            const double s_val = readSwitch(sbus_ch_spin_);
            spin_request_ = (s_val > 0.4) ? 1 : ((s_val < -0.4) ? -1 : 0);
            last_spin_time_ = now;
        }

        // 3-position Posture switch: low=OUTWARD, mid=INWARD, high=STRAFE_OUTWARD
        if (sbus_ch_posture_ >= 0) {
            const double p_val = readSwitch(sbus_ch_posture_);
            const PostureConfiguration requested_posture =
                (p_val < -0.4) ? PostureConfiguration::OUTWARD :
                (p_val >  0.4) ? PostureConfiguration::STRAFE_OUTWARD :
                                 PostureConfiguration::INWARD;
            applyPostureMode(requested_posture);
        }
    }

    // ================================================================
    // Callback: IMU Data (HiPNUC CH010) → State Estimator (EKF Predict)
    // ================================================================
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        const Eigen::Vector3d raw_acc(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        // Ignore empty startup frames published by EcatV2_Master before CAN data arrives
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
            imu.orientation = Eigen::Quaterniond(R_world_body).normalized();
            imu.has_orientation = true;
        } else {
            imu.orientation = Eigen::Quaterniond::Identity();
            imu.has_orientation = false;
        }

        // Cache for the control loop
        latest_imu_ = imu;
        last_imu_time_ = this->now();
        imu_received_ = true;
    }

    // ================================================================
    // Callback: Damiao DM-J4310P-2EC (Steer) & DM-J4340P-2EC V1.1 (Ecc)
    // ================================================================
    void dmMotorCallback(int leg_idx, bool is_steer,
                         const custom_msgs::msg::ReadDmMotor::SharedPtr msg) {
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

        const double pos = static_cast<double>(msg->position);
        const double vel = static_cast<double>(msg->velocity);
        const double torque = static_cast<double>(msg->torque);

        if (is_steer) {
            raw_steer_pos_[leg_idx] = pos;
            curr_steer_angles_(leg_idx) = kinematics_->normalize_angle(
                (pos - steer_offsets_[leg_idx]) * steer_signs_[leg_idx]);
            curr_steer_velocities_(leg_idx) = vel * steer_signs_[leg_idx];
            steer_received_[leg_idx] = true;
            last_steer_time_[leg_idx] = now;
        } else {
            raw_ecc_pos_[leg_idx] = pos;
            curr_ecc_angles_(leg_idx) = kinematics_->normalize_angle(
                (pos - ecc_offsets_[leg_idx]) * ecc_signs_[leg_idx]);
            curr_ecc_velocities_(leg_idx) = vel * ecc_signs_[leg_idx];
            curr_ecc_efforts_(leg_idx) = torque * ecc_signs_[leg_idx];
            ecc_received_[leg_idx] = true;
            last_ecc_time_[leg_idx] = now;
        }

        checkJointsInitialized();
    }

    // ================================================================
    // Callback: LK MF9010 V2 17T (Wheel Drive)
    // ================================================================
    void wheelMotorCallback(int leg_idx, const custom_msgs::msg::ReadLkMotor::SharedPtr msg) {
        const auto now = this->now();
        if (!msg->online) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[LK wheel[%d]] motor offline (enabled=%u, temp=%u C)",
                leg_idx, msg->enabled, msg->temperature);
        }

        // ReadLkMotor speed: 1 deg/s; current (MF serial): 33/4096 A per LSB
        constexpr double kDegToRad = M_PI / 180.0;
        constexpr double kLkMfCurrentScale = 33.0 / 4096.0;

        const double raw_vel_rad_s = static_cast<double>(msg->speed) * kDegToRad;
        const double raw_torque_nm = static_cast<double>(msg->current) * kLkMfCurrentScale * wheel_kt_;

        curr_wheel_velocities_(leg_idx) = raw_vel_rad_s * wheel_signs_[leg_idx];
        curr_wheel_efforts_(leg_idx) = raw_torque_nm * wheel_signs_[leg_idx];
        wheel_received_[leg_idx] = true;
        last_wheel_time_[leg_idx] = now;

        checkJointsInitialized();
    }

    void checkJointsInitialized() {
        if (joints_initialized_) {
            return;
        }
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (!steer_received_[i] || !ecc_received_[i] || !wheel_received_[i]) {
                return;
            }
        }
        joints_initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
            "All 12 motors online and initialized (4x DM-J4310P-2EC, 4x DM-J4340P-2EC V1.1, 4x LK MF9010 V2).");
    }

    bool updateMotorFreshness(const rclcpp::Time& now) {
        rclcpp::Time oldest = now;
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (!steer_received_[i] || !ecc_received_[i] || !wheel_received_[i]) {
                return false;
            }
            if (last_steer_time_[i] < oldest) oldest = last_steer_time_[i];
            if (last_ecc_time_[i] < oldest) oldest = last_ecc_time_[i];
            if (last_wheel_time_[i] < oldest) oldest = last_wheel_time_[i];
        }
        last_joint_state_time_ = oldest;
        if (motor_watchdog_timeout_ > 0.0 &&
            (now - oldest).seconds() > motor_watchdog_timeout_) {
            return false;
        }
        return true;
    }

    void publishDisableAllMotors() {
        custom_msgs::msg::WriteDmMotorMITControl dm_off;
        dm_off.enable = 0;
        dm_off.p_des = 0.0f;
        dm_off.v_des = 0.0f;
        dm_off.kp = 0.0f;
        dm_off.kd = 0.0f;
        dm_off.torque = 0.0f;

        custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit lk_off;
        lk_off.enable = 0;
        lk_off.torque_limit = 0;
        lk_off.speed = 0;

        for (int i = 0; i < NUM_LEGS; ++i) {
            pub_steer_cmds_[i]->publish(dm_off);
            pub_ecc_cmds_[i]->publish(dm_off);
            pub_wheel_cmds_[i]->publish(lk_off);
        }
    }

    void updateCombinedEStop(const char* source) {
        const bool combined = ros_e_stop_active_ || sbus_e_stop_active_;
        if (combined && !e_stop_active_) {
            RCLCPP_WARN(this->get_logger(),
                "EMERGENCY STOP ACTIVATED (%s)! Disabling all 12 motors.", source);
            publishDisableAllMotors();
            teleop_smoother_.reset();
            cmd_vel_.setZero();
            spin_request_ = 0;
            task_controller_->abort();
            task_controller_->reset();
            contact_detector_->reset();
            wheel_support_estimator_.reset();
            climb_geometry_estimator_->reset();
            terrain_estimator_->reset();
            state_estimator_->reset();
        } else if (!combined && e_stop_active_) {
            RCLCPP_INFO(this->get_logger(), "Emergency stop cleared.");
        }
        e_stop_active_ = combined;
    }

    void setSbusEStop(bool active) {
        if (sbus_e_stop_active_ == active) {
            return;
        }
        sbus_e_stop_active_ = active;
        updateCombinedEStop("SBUS RC");
    }

    // ================================================================
    // Callback: Emergency Stop
    // ================================================================
    void eStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        ros_e_stop_active_ = msg->data;
        updateCombinedEStop("ROS /e_stop");
        if (e_stop_active_) {
            publishDisableAllMotors();
        }
    }

    // ================================================================
    // Main Control Loop Callback
    // ================================================================
    void timerCallback() {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 0.5) return;
        if (!joints_initialized_) return;  // Wait for first feedback from all 12 motors

        if (!updateMotorFreshness(now)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Motor feedback timeout (> %.3f s); sending disable commands to all 12 motors.",
                motor_watchdog_timeout_);
            publishDisableAllMotors();
            wheel_support_estimator_.reset();
            balance_controller_->reset();
            return;
        }

        // Command source update & watchdog:
        // - While SBUS RC is connected, smooth stick inputs at 100 Hz via TeleopSmoother.
        // - Otherwise fall back to ROS /mobed/command with a 0.5 s timeout.
        if (sbus_enabled_ && sbus_received_) {
            if ((now - last_sbus_time_).seconds() > sbus_timeout_) {
                sbus_active_ = false;
                sbus_norm_vx_ = 0.0;
                sbus_norm_vy_ = 0.0;
                sbus_norm_wz_ = 0.0;
                spin_request_ = 0;
                cmd_vel_ = teleop_smoother_.update(0.0, 0.0, 0.0, dt, true);
                if (sbus_failsafe_estop_) {
                    setSbusEStop(true);
                }
            } else if (sbus_active_) {
                if (e_stop_active_) {
                    teleop_smoother_.reset();
                    cmd_vel_.setZero();
                } else {
                    cmd_vel_ = teleop_smoother_.update(sbus_norm_vx_, sbus_norm_vy_, sbus_norm_wz_, dt, false);
                    last_cmd_vel_time_ = now;
                }
            } else {
                cmd_vel_ = teleop_smoother_.update(0.0, 0.0, 0.0, dt, true);
            }
        } else {
            if ((now - last_cmd_vel_time_).seconds() > 0.5) {
                cmd_vel_.setZero();
                spin_request_ = 0;
            }
            if (spin_request_ != 0 && (now - last_spin_time_).seconds() > 0.5) {
                spin_request_ = 0;
                cmd_vel_.setZero();
            }
        }

        transition_smoother_.update(dt);

        // ============================================================
        // Phase 1: Homing Initialization
        // ============================================================
        if (is_homing_) {
            if (!require_imu_for_homing_ || imu_received_) {
                homing_timer_ += dt;
            }
            if (imu_received_ && !state_estimator_->isCalibrated()) {
                if (state_estimator_->updateStaticCalibration(latest_imu_)) {
                    auto b_a = state_estimator_->getAccelBias();
                    auto b_g = state_estimator_->getGyroBias();
                    RCLCPP_INFO(this->get_logger(),
                        "Static IMU calibration complete! Accel bias: [%.3f, %.3f, %.3f] m/s^2, Gyro bias: [%.4f, %.4f, %.4f] rad/s",
                        b_a.x(), b_a.y(), b_a.z(), b_g.x(), b_g.y(), b_g.z());
                }
            }
            bool has_teleop_input =
                (cmd_vel_.norm() > 0.02 || std::abs(target_height_ - nominal_drive_height_) > 0.01);
            bool imu_ready = !require_imu_for_homing_ ||
                             (imu_received_ && (state_estimator_->isCalibrated() || homing_timer_ > 2.5));
            if (imu_ready && (homing_timer_ > 1.0 || (homing_timer_ > 0.3 && has_teleop_input))) {
                is_homing_ = false;
                RCLCPP_INFO(this->get_logger(), "Homing complete (active teleop preemption enabled). Full control pipeline active.");
            }
        }

        // ============================================================
        // Phase 2: State Estimation (EKF predict + odometry correct Eq 3 with contact gating)
        // ============================================================
        // Query FSM task controller for airborne legs during climbing
        std::array<bool, NUM_LEGS> contact_valid = task_controller_->getContactValid();

        const estimation::WheelSupport* wheel_support = nullptr;
        if (!is_homing_ && !e_stop_active_ && spin_request_ == 0 &&
            spin_drive_.phase() == controllers::SpinDrive::Phase::IDLE &&
            climb_mode_ != ClimbMode::STAIR_200MM && !task_controller_->isActive() &&
            imu_received_ && (now - last_imu_time_).seconds() < 0.15) {
            wheel_support = &wheel_support_estimator_.update(
                curr_ecc_angles_, curr_ecc_velocities_, curr_ecc_efforts_, robot_params_.l_ecc, dt);
            for (int i = 0; i < NUM_LEGS; ++i) {
                contact_valid[i] = contact_valid[i] && wheel_support->supporting[i];
            }
        } else {
            wheel_support_estimator_.reset();
        }

        if (imu_received_) {
            state_estimator_->predict(latest_imu_, dt);
        }
        Eigen::Vector3d omega_body = imu_received_ ? latest_imu_.angular_velocity : Eigen::Vector3d::Zero();
        state_estimator_->correctWithOdometry(
            curr_wheel_velocities_, curr_steer_angles_, curr_ecc_angles_,
            curr_steer_velocities_, curr_ecc_velocities_, omega_body, contact_valid);

        BodyState body_state = state_estimator_->getState();

        // ============================================================
        // Phase 2.5: Terrain Estimation (Eq 4-6 in paper)
        // ============================================================
        std::array<Eigen::Vector3d, NUM_LEGS> foot_pos_in_base;
        std::array<Eigen::Vector3d, NUM_LEGS> contact_points_world;

        Eigen::Vector3d base_pos = body_state.position;
        Eigen::Matrix3d R_base = body_state.orientation.toRotationMatrix();

        for (int i = 0; i < NUM_LEGS; ++i) {
            foot_pos_in_base[i] = kinematics_->computeFootPositionInBase(
                i, curr_steer_angles_(i), curr_ecc_angles_(i));
            // p_C = p + R * ^B r_i (Eq 2) for srbd_model
            contact_points_world[i] = base_pos + R_base * foot_pos_in_base[i];
        }

        // Update climb geometry estimator (step height, standardized pitch, support margins)
        if (climb_mode_ != ClimbMode::DISABLED || task_controller_->isActive()) {
            climb_geometry_estimator_->update(
                body_state, contact_points_world,
                curr_ecc_angles_,
                curr_ecc_efforts_,
                curr_wheel_velocities_, curr_wheel_efforts_,
                prev_target_wheel_, omega_body,
                task_controller_->getCurbHeight(), dt, curr_ecc_velocities_);
        }

        if (climb_mode_ == ClimbMode::PAPER_NORMAL && spin_request_ != 0 && task_controller_->isActive()) {
            task_controller_->abort();
        }

        const bool spin_available = robot_params_.model_type == RobotModelType::ROBOX_V2 &&
            !is_homing_ && !transition_smoother_.in_transition && !task_controller_->isActive() &&
            climb_mode_ != ClimbMode::STAIR_200MM && imu_received_ &&
            (now - last_imu_time_).seconds() < 0.15 &&
            (now - last_joint_state_time_).seconds() < 0.15 &&
            latest_imu_.angular_velocity.allFinite() &&
            R_base(2, 2) > 0.97;
        const auto spin = spin_drive_.update(
            spin_request_, spin_available, e_stop_active_,
            controllers::SpinDrive::measuredHeading(latest_imu_.orientation, latest_imu_.has_orientation),
            (R_base * (omega_body - state_estimator_->getGyroBias())).z(),
            (R_base.transpose() * body_state.linear_velocity).head<2>(),
            driving_controller_->getSteeringError(),
            cmd_vel_.head<2>(), curr_ecc_angles_, *kinematics_, dt);
        Eigen::Vector3d active_cmd_vel =
            (is_homing_ || spin_request_ != 0) ? Eigen::Vector3d::Zero() : cmd_vel_;
        if (spin.ownsDrive()) active_cmd_vel = spin.twist;
        if (task_controller_->isActive()) {
            double fsm_vx = task_controller_->getForwardVelocityOverride();
            active_cmd_vel = Eigen::Vector3d(fsm_vx, 0.0, 0.0);
        }

        // Run plane fitting via pseudo-inverse least squares in robot heading frame
        bool climbing_active = task_controller_->isActive();
        if (!climbing_active) {
            bool is_stopped = (active_cmd_vel.norm() < 1e-3);
            terrain_estimator_->update(foot_pos_in_base, R_base, contact_valid, is_stopped, dt);
        }
        TerrainState terrain_state = terrain_estimator_->getState();

        // ============================================================
        // Phase 3: Contact Detection (Multi-Modal: Torque + Wheel Stall + Chassis Blocked + Wall Roll)
        // ============================================================
        Eigen::Vector3d v_body = R_base.transpose() * body_state.linear_velocity;
        double chassis_forward_vel = v_body.x();

        double imu_accel_x = imu_received_ ? latest_imu_.linear_acceleration.x() : 0.0;
        double imu_pitch_rate = imu_received_ ? latest_imu_.angular_velocity.y() : 0.0;
        contact_detector_->update(
            curr_ecc_efforts_, curr_wheel_efforts_, curr_wheel_velocities_,
            chassis_forward_vel, active_cmd_vel.x(), imu_accel_x,
            climb_geometry_estimator_->getGeometry(), dt,
            active_cmd_vel.y(), active_cmd_vel.z(),
            imu_pitch_rate);

        // ============================================================
        // Phase 4: FSM Task Controller (climbing state machine)
        // ============================================================
        if (!is_homing_ && !e_stop_active_ && !spin.ownsDrive() && spin_request_ == 0) {
            const auto& geom = climb_geometry_estimator_->getGeometry();

            // Only reset contact detector when explicitly reversing or driving backwards
            if (!task_controller_->isActive() && cmd_vel_(0) < -0.05) {
                contact_detector_->reset();
            }

            task_controller_->setNormalApproachAllowed(
                user_posture_config_ == PostureConfiguration::OUTWARD &&
                std::abs(cmd_vel_.y()) < 0.08 && std::abs(cmd_vel_.z()) < 0.25 &&
                curr_steer_angles_.cwiseAbs().maxCoeff() < 0.25);
            bool was_active = task_controller_->isActive();
            task_controller_->update(
                *contact_detector_, geom, curr_ecc_angles_, curr_ecc_efforts_, dt, chassis_forward_vel, cmd_vel_(0));
            bool is_active = task_controller_->isActive();
            if (was_active && !is_active) {
                climb_geometry_estimator_->reset();
                contact_detector_->reset();
            }
        }

        if (climb_mode_ == ClimbMode::PAPER_NORMAL && task_controller_->isActive()) {
            active_cmd_vel = Eigen::Vector3d(task_controller_->getForwardVelocityOverride(), 0.0, 0.0);
        }

        // Log FSM state transitions with override mask info
        static std::string prev_state_name = "IDLE";
        std::string curr_state_name = task_controller_->getStateName();
        if (curr_state_name != prev_state_name) {
            auto mask = task_controller_->getOverrideMask();
            RCLCPP_INFO(this->get_logger(), "[Adaptive FSM] Transition: %s -> %s | Overrides: [FL:%d FR:%d RL:%d RR:%d]",
                        prev_state_name.c_str(), curr_state_name.c_str(),
                        mask[FL], mask[FR], mask[RL], mask[RR]);
            if (prev_state_name == "REAR_PLACED" && curr_state_name == "IDLE") {
                // Post-climb handover: all 4 wheels settled on plateau.
                // Reset terrain & state estimators so BalanceController treats plateau as new flat ground baseline,
                // and sync balance controller angles to avoid any transient jerk.
                terrain_estimator_->reset();
                state_estimator_->reset();
                climb_geometry_estimator_->reset();
                contact_detector_->reset();
                balance_controller_->syncCurrentAngles(curr_ecc_angles_);
                RCLCPP_INFO(this->get_logger(), "[Adaptive FSM] Post-climb handover complete: plateau synchronized as flat baseline.");
            }
            prev_state_name = curr_state_name;
        }

        // ============================================================
        // Phase 5: Driving Controller (with Bank Angle & Torque Loops)
        // ============================================================
        auto [target_steer, target_wheel] = driving_controller_->update(
            active_cmd_vel, curr_steer_angles_, curr_ecc_angles_, dt,
            e_stop_active_, is_homing_, curr_steer_velocities_, curr_wheel_velocities_,
            curr_ecc_velocities_, contact_valid, spin.alignment_yaw_rate, spin.ownsDrive());

        prev_target_wheel_ = target_wheel;

        // ============================================================
        // Auto-Strafe Outward Stance Detection ("外八字" 判定)
        // When lateral strafe is commanded and swerve steering modules have
        // rotated into lateral alignment (FL/RR: ~ -90°, FR/RL: ~ +90°),
        // automatically transition eccentric arms to STRAFE_OUTWARD stance.
        // The eccentric arms smoothly lift the chassis by ~15mm as they pass
        // through zero, flaring the wheels outward to a wide 391mm track width.
        // ============================================================
        if (spin.ownsDrive()) {
            strafe_outward_active_ = false;
            strafe_exit_timer_ = 0.0;
            balance_controller_->setPostureConfiguration(PostureConfiguration::SPIN);
        } else if (user_posture_config_ == PostureConfiguration::OUTWARD &&
            !task_controller_->isActive() && !is_homing_ && !e_stop_active_) {

            bool strafe_intent = (std::abs(active_cmd_vel.y()) > 0.05 &&
                                  std::abs(active_cmd_vel.y()) >= 0.8 * std::abs(active_cmd_vel.x()));

            // Check if swerve steering modules have turned towards strafe alignment:
            // FL and RR steer negative (< -0.80 rad ≈ -46°), FR and RL steer positive (> 0.80 rad ≈ +46°)
            bool steer_aligned_strafe = (curr_steer_angles_(0) < -0.80 &&
                                         curr_steer_angles_(1) >  0.80 &&
                                         curr_steer_angles_(2) >  0.80 &&
                                         curr_steer_angles_(3) < -0.80);

            if (strafe_intent && steer_aligned_strafe) {
                if (!strafe_outward_active_) {
                    RCLCPP_INFO(this->get_logger(),
                        "[Auto-Strafe] Wheels turned to strafe heading. Smoothly transitioning to wide '外八字' posture (STRAFE_OUTWARD, track: 391mm).");
                }
                strafe_outward_active_ = true;
                strafe_exit_timer_ = 0.0;
            } else if (strafe_outward_active_) {
                // Exit condition: user drives longitudinally or stops commanding strafe
                bool longitudinal_intent = (std::abs(active_cmd_vel.x()) > 0.10 &&
                                            std::abs(active_cmd_vel.x()) > std::abs(active_cmd_vel.y()));
                if (longitudinal_intent) {
                    strafe_outward_active_ = false;
                    strafe_exit_timer_ = 0.0;
                    RCLCPP_INFO(this->get_logger(),
                        "[Auto-Strafe] Longitudinal driving commanded. Returning smoothly to standard OUTWARD posture.");
                } else if (!strafe_intent) {
                    strafe_exit_timer_ += dt;
                    if (strafe_exit_timer_ > 0.25) { // 250ms debounce
                        strafe_outward_active_ = false;
                        RCLCPP_INFO(this->get_logger(),
                            "[Auto-Strafe] Strafe command ended. Returning smoothly to standard OUTWARD posture.");
                    }
                } else {
                    strafe_exit_timer_ = 0.0;
                }
            }

            PostureConfiguration effective_config = strafe_outward_active_
                ? PostureConfiguration::STRAFE_OUTWARD
                : user_posture_config_;
            balance_controller_->setPostureConfiguration(effective_config);
        } else if (!task_controller_->isActive()) {
            balance_controller_->setPostureConfiguration(user_posture_config_);
        }

        // ============================================================
        // Phase 6: Balance Controller (with Bank Angle overlay)
        // ============================================================
        Eigen::Vector4d target_ecc = Eigen::Vector4d::Zero();
        ControlOutput balance_out;

        double active_height = (is_homing_ || spin.ownsDrive()) ? 0.205 : target_height_;
        double active_roll   = is_homing_ ? 0.0 : target_roll_;
        double active_pitch  = is_homing_ ? 0.0 : target_pitch_;

        // Overlay bank angle from driving controller onto target roll (only during active motion, not homing)
        if (!is_homing_ && !spin.ownsDrive()) {
            active_roll += driving_controller_->getBankAngle();
        }

        // Prepare terrain state for BalanceController:
        // During homing, enforce flat horizontal terrain to prevent uncalibrated IMU drift from tilting the posture.
        // When climbing an obstacle, zero out longitudinal pitch slope compensation.
        TerrainState active_terrain_state = terrain_state;
        if (is_homing_) {
            active_terrain_state.pitch_slope = 0.0;
            active_terrain_state.roll_slope = 0.0;
            active_terrain_state.slope_angle = 0.0;
            active_terrain_state.normal = Eigen::Vector3d::UnitZ();
            active_terrain_state.rotation = Eigen::Matrix3d::Identity();
            active_terrain_state.is_valid = true;
        } else if (task_controller_->isActive()) {
            active_terrain_state.pitch_slope = 0.0;
            active_terrain_state.normal = Eigen::Vector3d(0.0, -std::sin(active_terrain_state.roll_slope), std::cos(active_terrain_state.roll_slope));
            active_terrain_state.rotation = Eigen::AngleAxisd(
                active_terrain_state.roll_slope, Eigen::Vector3d::UnitX()).toRotationMatrix();
        }

        // Call updateHybrid using contact points from Phase 2.5 and body_state from Phase 2
        const Eigen::Vector3d balance_twist = spin.ownsDrive()
            ? Eigen::Vector3d::Zero() : active_cmd_vel;
        balance_out = balance_controller_->updateHybrid(
            active_height, active_roll, active_pitch,
            target_steer, curr_ecc_angles_, body_state,
            contact_points_world, contact_valid, active_terrain_state, dt, e_stop_active_, balance_twist,
            climb_mode_ != ClimbMode::STAIR_200MM,
            (task_controller_->isActive() || spin.ownsDrive()) ? nullptr : wheel_support);

        target_ecc = balance_out.ecc_angles;

        // ============================================================
        // Phase 7: FSM Override — replace specific legs if climbing
        // ============================================================
        // Always snapshot the FSM override angles; needed for blend-out lerp in Phase 8
        // even when isActive() is false (during the IDLE blend-out period).
        Eigen::Vector4d ecc_overrides_vec = task_controller_->getEccOverrides();
        std::array<bool, NUM_LEGS> ecc_override_mask = task_controller_->getOverrideMask();
        double blend_w = task_controller_->getBlendWeight();
        if (task_controller_->isActive()) {
            for (int i = 0; i < NUM_LEGS; ++i) {
                if (ecc_override_mask[i]) {
                    target_ecc(i) = ecc_overrides_vec(i);
                }
            }
        } else if (blend_w > 0.0) {
            for (int i = 0; i < NUM_LEGS; ++i) {
                if (ecc_override_mask[i]) {
                    const double delta = climb_mode_ == ClimbMode::PAPER_NORMAL
                    ? target_ecc(i) - ecc_overrides_vec(i)
                    : kinematics_->normalize_angle(target_ecc(i) - ecc_overrides_vec(i));
                    target_ecc(i) = ecc_overrides_vec(i) + (1.0 - blend_w) * delta;
                }
            }
        }

        // Apply TransitionSmoother (Zero-jerk mode switch S-curve)
        if (transition_smoother_.in_transition) {
            double alpha = transition_smoother_.getWeight();
            for (int i = 0; i < NUM_LEGS; ++i) {
                target_ecc(i) = (1.0 - alpha) * transition_smoother_.start_ecc_angles(i) + alpha * target_ecc(i);
            }
        }

        // ============================================================
        // Phase 8: Publish real-hardware commands to 12 motors
        // ============================================================
        if (!enable_motors_ || e_stop_active_) {
            publishDisableAllMotors();
            return;
        }

        constexpr double kRadToDeg = 180.0 / M_PI;
        const double safe_kt = std::max(wheel_kt_, 1e-4);
        const double safe_imax = std::max(wheel_mf_max_current_a_, 1e-4);
        const int16_t wheel_torque_limit_raw = static_cast<int16_t>(std::clamp(
            std::llround((wheel_max_torque_nm_ / safe_kt) / safe_imax * 2048.0),
            1LL, 2048LL));

        for (int i = 0; i < NUM_LEGS; ++i) {
            // 1. Steering: DM-J4310P-2EC (MIT position/damping mode)
            const double raw_steer_target = target_steer(i) * steer_signs_[i] + steer_offsets_[i];
            const auto mit_steer = controllers::steeringMitCommand(
                raw_steer_target, raw_steer_pos_[i], 0.0, dm_steer_kp_, dm_steer_kd_, dm_steer_pmax_);

            custom_msgs::msg::WriteDmMotorMITControl steer_msg;
            steer_msg.enable = 1;
            steer_msg.p_des  = static_cast<float>(mit_steer.position);
            steer_msg.v_des  = static_cast<float>(mit_steer.velocity);
            steer_msg.kp     = static_cast<float>(mit_steer.kp);
            steer_msg.kd     = static_cast<float>(mit_steer.kd);
            steer_msg.torque = static_cast<float>(mit_steer.torque_feedforward);
            pub_steer_cmds_[i]->publish(steer_msg);

            // 2. Eccentric: DM-J4340P-2EC V1.1 (MIT position + feedforward torque, 12 Nm rated)
            const double raw_ecc_target = target_ecc(i) * ecc_signs_[i] + ecc_offsets_[i];

            // Zero out feedforward GRF torque for FSM-overridden legs to prevent conflict with position trajectory
            const bool leg_is_overridden = task_controller_->isActive() && ecc_override_mask[i];
            const double ff_weight = transition_smoother_.getFeedforwardWeight();
            double effort_cmd = (leg_is_overridden || !use_ecc_torque_ff_)
                              ? 0.0
                              : balance_out.ecc_torques(i) * ecc_signs_[i] * ff_weight;
            // DM-J4340P-2EC V1.1 rated continuous torque (12 Nm; 40 Nm peak)
            effort_cmd = std::clamp(effort_cmd, -12.0, 12.0);

            auto mit_ecc = controllers::eccentricMitCommand(
                raw_ecc_target, raw_ecc_pos_[i], dm_ecc_kp_, dm_ecc_kd_, effort_cmd, dm_ecc_pmax_);
            controllers::limitMitFeedforward(
                mit_ecc, raw_ecc_pos_[i], curr_ecc_velocities_(i) * ecc_signs_[i], 12.0);

            custom_msgs::msg::WriteDmMotorMITControl ecc_msg;
            ecc_msg.enable = 1;
            ecc_msg.p_des  = static_cast<float>(mit_ecc.position);
            ecc_msg.v_des  = static_cast<float>(mit_ecc.velocity);
            ecc_msg.kp     = static_cast<float>(mit_ecc.kp);
            ecc_msg.kd     = static_cast<float>(mit_ecc.kd);
            ecc_msg.torque = static_cast<float>(mit_ecc.torque_feedforward);
            pub_ecc_cmds_[i]->publish(ecc_msg);

            // 3. Wheels: LK MF9010 V2 17T (Speed With Torque Limit, unit: 0.01 deg/s)
            double target_wheel_vel = target_wheel(i);
            if (task_controller_->isActive()) {
                auto wheel_override_mask = task_controller_->getWheelOverrideMask();
                if (wheel_override_mask[i]) {
                    target_wheel_vel = task_controller_->getWheelSpeedOverrides()(i);
                }
            }
            if (transition_smoother_.in_transition && !spin.ownsDrive() && !e_stop_active_) {
                double alpha = transition_smoother_.getWeight();
                target_wheel_vel = (1.0 - alpha) * transition_smoother_.start_wheel_vels(i) + alpha * target_wheel_vel;
            }
            const double raw_wheel_vel = target_wheel_vel * wheel_signs_[i];

            custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit wheel_msg;
            wheel_msg.enable       = 1;
            wheel_msg.torque_limit = wheel_torque_limit_raw;
            wheel_msg.speed        = static_cast<int32_t>(std::llround(raw_wheel_vel * kRadToDeg * 100.0));
            pub_wheel_cmds_[i]->publish(wheel_msg);
        }
    }

    // ================================================================
    // Algorithm Modules
    // ================================================================
    std::shared_ptr<kinematics::MobedKinematics>      kinematics_;
    std::unique_ptr<controllers::DrivingController>  driving_controller_;
    std::unique_ptr<controllers::BalanceController>   balance_controller_;
    std::unique_ptr<estimation::StateEstimator>       state_estimator_;
    std::unique_ptr<estimation::TerrainEstimator>     terrain_estimator_;
    std::unique_ptr<estimation::ContactDetector>      contact_detector_;
    estimation::WheelSupportEstimator wheel_support_estimator_;
    std::unique_ptr<tasks::TaskController>            task_controller_;
    std::unique_ptr<estimation::ClimbGeometryEstimator> climb_geometry_estimator_;

    RobotParams robot_params_;
    ClimbMode climb_mode_ = ClimbMode::PAPER_NORMAL;

    // Zero-Jerk Mode Transition Smoother (0.2s Cosine S-curve)
    struct TransitionSmoother {
        bool in_transition = false;
        double timer = 0.0;
        double duration = 0.20;
        Eigen::Vector4d start_ecc_angles = Eigen::Vector4d::Zero();
        Eigen::Vector4d start_wheel_vels = Eigen::Vector4d::Zero();
        double start_ff_torque_weight = 1.0;
        double target_ff_torque_weight = 1.0;

        void start(const Eigen::Vector4d& curr_ecc, const Eigen::Vector4d& curr_wheel_vels,
                   double start_ff_weight, double target_ff_weight, double trans_duration = 0.20) {
            in_transition = true;
            timer = 0.0;
            duration = trans_duration;
            start_ecc_angles = curr_ecc;
            start_wheel_vels = curr_wheel_vels;
            start_ff_torque_weight = start_ff_weight;
            target_ff_torque_weight = target_ff_weight;
        }

        void update(double dt) {
            if (!in_transition) return;
            timer += dt;
            if (timer >= duration) {
                in_transition = false;
                timer = duration;
            }
        }

        double getWeight() const {
            if (!in_transition) return 1.0;
            double s = std::clamp(timer / duration, 0.0, 1.0);
            return 0.5 * (1.0 - std::cos(M_PI * s));
        }

        double getFeedforwardWeight() const {
            if (!in_transition) return target_ff_torque_weight;
            double w = getWeight();
            return (1.0 - w) * start_ff_torque_weight + w * target_ff_torque_weight;
        }
    } transition_smoother_;

    // ================================================================
    // ROS 2 Communication
    // ================================================================
    rclcpp::Subscription<robot_interfaces::msg::MobEDCommand>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<custom_msgs::msg::ReadSBUSRC>::SharedPtr sub_sbus_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_e_stop_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_payload_mass_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_curb_height_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_posture_mode_int_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_posture_mode_str_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_climb_mode_;

    std::array<rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr, NUM_LEGS> sub_steer_motors_;
    std::array<rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr, NUM_LEGS> sub_ecc_motors_;
    std::array<rclcpp::Subscription<custom_msgs::msg::ReadLkMotor>::SharedPtr, NUM_LEGS> sub_wheel_motors_;

    std::array<rclcpp::Publisher<custom_msgs::msg::WriteDmMotorMITControl>::SharedPtr, NUM_LEGS> pub_steer_cmds_;
    std::array<rclcpp::Publisher<custom_msgs::msg::WriteDmMotorMITControl>::SharedPtr, NUM_LEGS> pub_ecc_cmds_;
    std::array<rclcpp::Publisher<custom_msgs::msg::WriteLkMotorSpeedControlWithTorqueLimit>::SharedPtr, NUM_LEGS> pub_wheel_cmds_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
    rclcpp::Time last_cmd_vel_time_;
    rclcpp::Time last_spin_time_;
    rclcpp::Time last_imu_time_;
    rclcpp::Time last_sbus_time_;
    rclcpp::Time last_joint_state_time_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_spin_;
    controllers::SpinDrive spin_drive_;
    controllers::TeleopSmoother teleop_smoother_;
    int spin_request_ = 0;

    // ================================================================
    // State Variables
    // ================================================================
    Eigen::Vector3d cmd_vel_;
    double nominal_drive_height_ = 0.175;
    double target_height_ = 0.175;
    double target_roll_   = 0.0;
    double target_pitch_  = 0.0;

    Eigen::Vector4d curr_steer_angles_;
    Eigen::Vector4d curr_steer_velocities_;
    Eigen::Vector4d curr_ecc_angles_;
    Eigen::Vector4d curr_ecc_velocities_;
    Eigen::Vector4d curr_ecc_efforts_;
    Eigen::Vector4d curr_wheel_efforts_;
    Eigen::Vector4d curr_wheel_velocities_;
    Eigen::Vector4d prev_target_wheel_ = Eigen::Vector4d::Zero();

    std::array<double, NUM_LEGS> raw_steer_pos_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, NUM_LEGS> raw_ecc_pos_   = {0.0, 0.0, 0.0, 0.0};
    std::array<bool, NUM_LEGS> steer_received_  = {false, false, false, false};
    std::array<bool, NUM_LEGS> ecc_received_    = {false, false, false, false};
    std::array<bool, NUM_LEGS> wheel_received_  = {false, false, false, false};
    std::array<rclcpp::Time, NUM_LEGS> last_steer_time_{};
    std::array<rclcpp::Time, NUM_LEGS> last_ecc_time_{};
    std::array<rclcpp::Time, NUM_LEGS> last_wheel_time_{};

    ImuData latest_imu_;
    Eigen::Matrix3d R_body_imu_ = Eigen::Matrix3d::Identity();
    bool imu_received_ = false;
    bool require_imu_for_homing_ = true;
    bool joints_initialized_ = false;  // true after first feedback from all 12 motors

    bool e_stop_active_ = false;
    bool ros_e_stop_active_ = false;
    bool sbus_e_stop_active_ = false;
    bool is_homing_ = true;
    double homing_timer_ = 0.0;
    const double HOMING_DURATION = 3.0;

    // SBUS RC state & configuration
    bool sbus_enabled_ = true;
    bool sbus_received_ = false;
    bool sbus_active_ = false;
    bool sbus_failsafe_estop_ = true;
    double sbus_timeout_ = 0.5;
    double sbus_ch_min_ = 172.0;
    double sbus_ch_mid_ = 992.0;
    double sbus_ch_max_ = 1811.0;
    double sbus_deadband_ = 0.05;
    int sbus_ch_vx_ = 1;
    int sbus_ch_vy_ = 0;
    int sbus_ch_wz_ = 3;
    int sbus_ch_height_ = 2;
    int sbus_ch_estop_ = 4;
    int sbus_ch_climb_mode_ = 5;
    int sbus_ch_spin_ = 6;
    int sbus_ch_posture_ = 7;
    bool sbus_inv_vx_ = false;
    bool sbus_inv_vy_ = false;
    bool sbus_inv_wz_ = false;
    bool sbus_inv_height_ = false;
    bool sbus_inv_estop_ = false;
    double sbus_height_min_ = 0.084;
    double sbus_height_max_ = 0.22;
    double sbus_norm_vx_ = 0.0;
    double sbus_norm_vy_ = 0.0;
    double sbus_norm_wz_ = 0.0;

    // Hardware control & safety parameters
    bool enable_motors_ = false;
    bool use_ecc_torque_ff_ = true;
    double motor_watchdog_timeout_ = 0.25;
    double dm_steer_kp_ = 360.0;
    double dm_steer_kd_ = 5.0;
    double dm_steer_pmax_ = M_PI;
    double dm_ecc_kp_ = 125.0;
    double dm_ecc_kd_ = 4.8;
    double dm_ecc_pmax_ = M_PI;
    double wheel_kt_ = 0.16;
    double wheel_max_torque_nm_ = 2.2;
    double wheel_mf_max_current_a_ = 16.5;

    // Polarity & Zero-Offset Mapping Arrays
    std::vector<double> steer_signs_;
    std::vector<double> steer_offsets_;
    std::vector<double> ecc_signs_;
    std::vector<double> ecc_offsets_;
    std::vector<double> wheel_signs_;

    // Auto-Strafe Outward Stance ("外八字") state
    PostureConfiguration user_posture_config_ = PostureConfiguration::OUTWARD;
    bool strafe_outward_active_ = false;
    double strafe_exit_timer_ = 0.0;
};

}  // namespace robot_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_control::MobedControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
