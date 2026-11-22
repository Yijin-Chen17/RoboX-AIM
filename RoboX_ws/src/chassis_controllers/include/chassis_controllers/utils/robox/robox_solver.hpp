#ifndef CHASSIS_CONTROLLERS_UTILS_ROBOX_ROBOX_SOLVER_HPP
#define CHASSIS_CONTROLLERS_UTILS_ROBOX_ROBOX_SOLVER_HPP

#pragma once

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <string>

#include "chassis_controllers/utils/robox/controllers/balance_controller.hpp"
#include "chassis_controllers/utils/robox/controllers/driving_controller.hpp"
#include "chassis_controllers/utils/robox/controllers/spin_drive.hpp"
#include "chassis_controllers/utils/robox/core/mobed_types.hpp"
#include "chassis_controllers/utils/robox/core/robot_params.hpp"
#include "chassis_controllers/utils/robox/estimation/climb_geometry_estimator.hpp"
#include "chassis_controllers/utils/robox/estimation/contact_detector.hpp"
#include "chassis_controllers/utils/robox/estimation/state_estimator.hpp"
#include "chassis_controllers/utils/robox/estimation/terrain_estimator.hpp"
#include "chassis_controllers/utils/robox/estimation/wheel_support.hpp"
#include "chassis_controllers/utils/robox/kinematics/mobed_kinematics.hpp"
#include "chassis_controllers/utils/robox/tasks/task_controller.hpp"

namespace robot_control {

struct RoboxSensorState {
    Eigen::Vector4d steer_angles     = Eigen::Vector4d::Zero();
    Eigen::Vector4d steer_velocities = Eigen::Vector4d::Zero();
    Eigen::Vector4d ecc_angles       = Eigen::Vector4d::Zero();
    Eigen::Vector4d ecc_velocities   = Eigen::Vector4d::Zero();
    Eigen::Vector4d ecc_efforts      = Eigen::Vector4d::Zero();
    Eigen::Vector4d wheel_velocities = Eigen::Vector4d::Zero();
    Eigen::Vector4d wheel_efforts    = Eigen::Vector4d::Zero();

    ImuData imu;
    bool imu_received    = false;
    double imu_age_sec   = 0.0;
    double joint_age_sec = 0.0;
};

struct RoboxCommandState {
    Eigen::Vector3d cmd_vel = Eigen::Vector3d::Zero();  // [x_speed, y_speed, yaw_speed]
    double target_height    = 0.175;
    double target_roll      = 0.0;
    double target_pitch     = 0.0;
    int spin_request        = 0;      // -1 = CW, 0 = off, 1 = CCW
    bool emergency_stop     = false;
};

struct RoboxSolverOutput {
    Eigen::Vector4d target_steer_angles = Eigen::Vector4d::Zero();
    Eigen::Vector4d target_ecc_angles   = Eigen::Vector4d::Zero();
    Eigen::Vector4d target_ecc_torques  = Eigen::Vector4d::Zero();
    Eigen::Vector4d target_wheel_speeds = Eigen::Vector4d::Zero();

    std::array<bool, NUM_LEGS> ecc_override_mask{false, false, false, false};
    bool task_active           = false;
    bool is_homing             = true;
    bool imu_just_calibrated   = false;
    bool homing_just_completed = false;
    bool fsm_state_changed     = false;
    std::string prev_fsm_state = "IDLE";
    std::string curr_fsm_state = "IDLE";
    bool unsafe                = false;
};

/**
 * @brief Encapsulates the full 8-phase RoboX / MobED whole-body control & locomotion pipeline.
 *
 * Combines kinematics, EKF state estimation, terrain & geometry estimation, multi-modal contact
 * detection, adaptive climbing FSM, SpinDrive, driving controller, and SRBD+QP balance control
 * in pure C++ with zero ROS 2 dependencies.
 */
class RoboxSolver {
public:
    explicit RoboxSolver(RobotModelType model_type = RobotModelType::ROBOX_V2);

    void setRobotModel(RobotModelType model_type);
    void setImuLeverArm(const Eigen::Vector3d& lever_arm);
    void setRequireImuForHoming(bool require) { require_imu_for_homing_ = require; }

    bool setPayloadMass(double mass);
    bool setPostureMode(PostureConfiguration cfg);
    bool setClimbMode(ClimbMode new_mode);
    bool setCurbHeight(double height);

    void resetOnEmergencyStop();
    void resetOnWatchdogTimeout();

    double nominalDriveHeight() const { return nominal_drive_height_; }
    Eigen::Vector3d imuAccelBias() const { return state_estimator_->getAccelBias(); }
    Eigen::Vector3d imuGyroBias() const { return state_estimator_->getGyroBias(); }

    RoboxSolverOutput update(const RoboxSensorState& in, const RoboxCommandState& cmd, double dt);

private:
    void updateAutoStrafePosture(
        const Eigen::Vector3d& active_cmd_vel,
        const Eigen::Vector4d& curr_steer_angles,
        bool spin_owns_drive,
        bool emergency_stop,
        double dt);

    RobotParams robot_params_;
    double nominal_drive_height_ = 0.175;
    bool require_imu_for_homing_ = true;

    std::shared_ptr<kinematics::MobedKinematics> kinematics_;
    std::unique_ptr<controllers::DrivingController> driving_controller_;
    std::unique_ptr<controllers::BalanceController> balance_controller_;
    std::unique_ptr<estimation::StateEstimator> state_estimator_;
    std::unique_ptr<estimation::TerrainEstimator> terrain_estimator_;
    std::unique_ptr<estimation::ContactDetector> contact_detector_;
    estimation::WheelSupportEstimator wheel_support_estimator_;
    std::unique_ptr<tasks::TaskController> task_controller_;
    std::unique_ptr<estimation::ClimbGeometryEstimator> climb_geometry_estimator_;
    controllers::SpinDrive spin_drive_;

    ClimbMode climb_mode_ = ClimbMode::PAPER_NORMAL;
    PostureConfiguration user_posture_config_ = PostureConfiguration::OUTWARD;
    double user_curb_height_  = 0.075;
    double user_payload_mass_ = 0.0;

    bool is_homing_ = true;
    double homing_timer_ = 0.0;
    Eigen::Vector4d prev_target_wheel_ = Eigen::Vector4d::Zero();
    std::string prev_fsm_state_name_   = "IDLE";
    bool strafe_outward_active_        = false;
    double strafe_exit_timer_          = 0.0;
};

}  // namespace robot_control

#endif  // CHASSIS_CONTROLLERS_UTILS_ROBOX_ROBOX_SOLVER_HPP
