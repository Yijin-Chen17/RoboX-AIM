#include "chassis_controllers/utils/robox/robox_solver.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control {

RoboxSolver::RoboxSolver(const RobotModelType model_type) {
    setRobotModel(model_type);
}

void RoboxSolver::setRobotModel(const RobotModelType model_type) {
    robot_params_.applyModel(model_type);
    nominal_drive_height_ = (model_type == RobotModelType::ROBOX_V2) ? 0.175 : 0.205;

    kinematics::MobedParameters k_params;
    k_params.length_x         = robot_params_.length_x;
    k_params.width_y          = robot_params_.width_y;
    k_params.l_ecc            = robot_params_.l_ecc;
    k_params.r_wheel          = robot_params_.r_wheel;
    k_params.posture_z_offset = robot_params_.posture_z_offset;
    k_params.max_height       = robot_params_.max_height;
    k_params.min_height       = robot_params_.min_height;
    kinematics_ = std::make_shared<kinematics::MobedKinematics>(k_params);

    driving_controller_ = std::make_unique<controllers::DrivingController>(
        controllers::DrivingControllerParams{}, kinematics_);
    driving_controller_->setWheelPiParams(wheel_kp_, wheel_ki_, wheel_max_torque_);
    balance_controller_ = std::make_unique<controllers::BalanceController>(
        controllers::BalanceControllerParams{}, kinematics_, robot_params_);
    balance_controller_->setPostureConfiguration(user_posture_config_);
    if (user_payload_mass_ > 0.0) {
        balance_controller_->setPayloadMass(user_payload_mass_);
    }

    state_estimator_ = std::make_unique<estimation::StateEstimator>(robot_params_);
    state_estimator_->startStaticCalibration(200);
    terrain_estimator_        = std::make_unique<estimation::TerrainEstimator>(robot_params_);
    contact_detector_         = std::make_unique<estimation::ContactDetector>(robot_params_);
    climb_geometry_estimator_ = std::make_unique<estimation::ClimbGeometryEstimator>(robot_params_);
    task_controller_          = std::make_unique<tasks::TaskController>();
    task_controller_->setClimbMode(climb_mode_);
    task_controller_->setCurbHeight(
        climb_mode_ == ClimbMode::STAIR_200MM ? 0.20 : user_curb_height_);

    wheel_support_estimator_.reset();
    spin_drive_.reset();
    prev_target_wheel_.setZero();
    prev_fsm_state_name_   = "IDLE";
    strafe_outward_active_ = false;
    strafe_exit_timer_     = 0.0;
    is_homing_             = true;
    homing_timer_          = 0.0;
}

void RoboxSolver::setImuLeverArm(const Eigen::Vector3d& lever_arm) {
    state_estimator_->setImuLeverArm(lever_arm);
}

void RoboxSolver::setWheelPiParams(
    const double kp_wheel, const double ki_wheel, const double max_wheel_torque) {
    wheel_kp_         = kp_wheel;
    wheel_ki_         = ki_wheel;
    wheel_max_torque_ = std::max(0.0, max_wheel_torque);
    if (driving_controller_) {
        driving_controller_->setWheelPiParams(wheel_kp_, wheel_ki_, wheel_max_torque_);
    }
}

bool RoboxSolver::setPayloadMass(const double mass) {
    const double clamped = std::max(0.0, mass);
    if (std::abs(clamped - user_payload_mass_) <= 1e-4) {
        return false;
    }
    user_payload_mass_ = clamped;
    balance_controller_->setPayloadMass(clamped);
    return true;
}

bool RoboxSolver::setPostureMode(const PostureConfiguration cfg) {
    if (cfg == user_posture_config_) {
        return false;
    }
    user_posture_config_ = cfg;
    balance_controller_->setPostureConfiguration(cfg);
    return true;
}

bool RoboxSolver::setClimbMode(const ClimbMode new_mode) {
    if (new_mode == climb_mode_) {
        return false;
    }
    if (climb_mode_ == ClimbMode::STAIR_200MM && new_mode != ClimbMode::STAIR_200MM) {
        if (task_controller_->isActive()) {
            task_controller_->abort();
        }
    }
    climb_mode_ = new_mode;
    task_controller_->setClimbMode(climb_mode_);
    if (climb_mode_ == ClimbMode::STAIR_200MM) {
        task_controller_->setCurbHeight(0.20);
    } else {
        task_controller_->setCurbHeight(user_curb_height_);
    }
    return true;
}

bool RoboxSolver::setCurbHeight(const double height) {
    const double clamped = std::clamp(height, 0.04, 0.22);
    if (std::abs(clamped - user_curb_height_) <= 1e-4 ||
        climb_mode_ == ClimbMode::STAIR_200MM ||
        task_controller_->isActive() ||
        climb_geometry_estimator_->hasLatchedHeight()) {
        return false;
    }
    user_curb_height_ = clamped;
    task_controller_->setCurbHeight(clamped);
    return true;
}

void RoboxSolver::resetOnEmergencyStop() {
    task_controller_->abort();
    task_controller_->reset();
    contact_detector_->reset();
    wheel_support_estimator_.reset();
    climb_geometry_estimator_->reset();
    terrain_estimator_->reset();
    state_estimator_->reset();
    driving_controller_->reset();
    balance_controller_->reset();
    spin_drive_.reset();
    prev_target_wheel_.setZero();
    prev_fsm_state_name_   = "IDLE";
    strafe_outward_active_ = false;
    strafe_exit_timer_     = 0.0;
}

void RoboxSolver::updateAutoStrafePosture(
    const Eigen::Vector3d& active_cmd_vel,
    const Eigen::Vector4d& curr_steer_angles,
    const bool spin_owns_drive,
    const bool emergency_stop,
    const double dt) {
    if (spin_owns_drive) {
        strafe_outward_active_ = false;
        strafe_exit_timer_     = 0.0;
        balance_controller_->setPostureConfiguration(PostureConfiguration::SPIN);
        return;
    }

    if (user_posture_config_ == PostureConfiguration::OUTWARD &&
        !task_controller_->isActive() && !is_homing_ && !emergency_stop) {
        const bool strafe_intent =
            (std::abs(active_cmd_vel.y()) > 0.05 &&
             std::abs(active_cmd_vel.y()) >= 0.8 * std::abs(active_cmd_vel.x()));
        const bool steer_aligned_strafe =
            (curr_steer_angles(0) < -0.80 &&
             curr_steer_angles(1) >  0.80 &&
             curr_steer_angles(2) >  0.80 &&
             curr_steer_angles(3) < -0.80);

        if (strafe_intent && steer_aligned_strafe) {
            strafe_outward_active_ = true;
            strafe_exit_timer_     = 0.0;
        } else if (strafe_outward_active_) {
            const bool longitudinal_intent =
                (std::abs(active_cmd_vel.x()) > 0.10 &&
                 std::abs(active_cmd_vel.x()) > std::abs(active_cmd_vel.y()));
            if (longitudinal_intent) {
                strafe_outward_active_ = false;
                strafe_exit_timer_     = 0.0;
            } else if (!strafe_intent) {
                strafe_exit_timer_ += dt;
                if (strafe_exit_timer_ > 0.25) {
                    strafe_outward_active_ = false;
                }
            } else {
                strafe_exit_timer_ = 0.0;
            }
        }

        const PostureConfiguration effective_config = strafe_outward_active_
            ? PostureConfiguration::STRAFE_OUTWARD
            : user_posture_config_;
        balance_controller_->setPostureConfiguration(effective_config);
    } else if (!task_controller_->isActive()) {
        balance_controller_->setPostureConfiguration(user_posture_config_);
    }
}

RoboxSolverOutput RoboxSolver::update(
    const RoboxSensorState& in, const RoboxCommandState& cmd, const double dt) {
    RoboxSolverOutput out;
    out.prev_fsm_state = prev_fsm_state_name_;

    // Phase 1: Homing Initialization & IMU Static Calibration
    if (is_homing_) {
        if (!require_imu_for_homing_ || in.imu_received) {
            homing_timer_ += dt;
        }
        if (in.imu_received && !state_estimator_->isCalibrated()) {
            if (state_estimator_->updateStaticCalibration(in.imu)) {
                out.imu_just_calibrated = true;
            }
        }
        const bool has_cmd_input =
            (cmd.cmd_vel.norm() > 0.02 ||
             std::abs(cmd.target_height - nominal_drive_height_) > 0.01);
        const bool imu_ready =
            !require_imu_for_homing_ ||
            (in.imu_received && (state_estimator_->isCalibrated() || homing_timer_ > 2.5));
        if (imu_ready && (homing_timer_ > 1.0 || (homing_timer_ > 0.3 && has_cmd_input))) {
            is_homing_                = false;
            out.homing_just_completed = true;
        }
    }
    out.is_homing = is_homing_;

    // Phase 2: State Estimation & Wheel Support Check
    std::array<bool, NUM_LEGS> contact_valid = task_controller_->getContactValid();
    const estimation::WheelSupport* wheel_support = nullptr;
    if (!is_homing_ && !cmd.emergency_stop && cmd.spin_request == 0 &&
        spin_drive_.phase() == controllers::SpinDrive::Phase::IDLE &&
        climb_mode_ != ClimbMode::STAIR_200MM && !task_controller_->isActive() &&
        in.imu_received && in.imu_age_sec < 0.15) {
        wheel_support = &wheel_support_estimator_.update(
            in.ecc_angles, in.ecc_velocities, in.ecc_efforts, robot_params_.l_ecc, dt);
        for (int i = 0; i < NUM_LEGS; ++i) {
            contact_valid[i] = contact_valid[i] && wheel_support->supporting[i];
        }
    } else {
        wheel_support_estimator_.reset();
    }

    if (in.imu_received) {
        state_estimator_->predict(in.imu, dt);
    }
    const Eigen::Vector3d omega_body =
        in.imu_received ? in.imu.angular_velocity : Eigen::Vector3d::Zero();
    state_estimator_->correctWithOdometry(
        in.wheel_velocities, in.steer_angles, in.ecc_angles,
        in.steer_velocities, in.ecc_velocities, omega_body, contact_valid);
    const BodyState body_state = state_estimator_->getState();

    // Phase 2.5: Foot Kinematics, Climb Geometry, SpinDrive & Terrain Estimation
    std::array<Eigen::Vector3d, NUM_LEGS> foot_pos_in_base;
    std::array<Eigen::Vector3d, NUM_LEGS> contact_points_world;
    const Eigen::Vector3d base_pos = body_state.position;
    const Eigen::Matrix3d R_base   = body_state.orientation.toRotationMatrix();

    for (int i = 0; i < NUM_LEGS; ++i) {
        foot_pos_in_base[i] = kinematics_->computeFootPositionInBase(
            i, in.steer_angles(i), in.ecc_angles(i));
        contact_points_world[i] = base_pos + R_base * foot_pos_in_base[i];
    }

    if (climb_mode_ != ClimbMode::DISABLED || task_controller_->isActive()) {
        climb_geometry_estimator_->update(
            body_state, contact_points_world,
            in.ecc_angles, in.ecc_efforts,
            in.wheel_velocities, in.wheel_efforts,
            prev_target_wheel_, omega_body,
            task_controller_->getCurbHeight(), dt, in.ecc_velocities);
    }

    if (climb_mode_ == ClimbMode::PAPER_NORMAL && cmd.spin_request != 0 &&
        task_controller_->isActive()) {
        task_controller_->abort();
    }

    const bool spin_available =
        robot_params_.model_type == RobotModelType::ROBOX_V2 &&
        !is_homing_ && !task_controller_->isActive() &&
        climb_mode_ != ClimbMode::STAIR_200MM && in.imu_received &&
        in.imu_age_sec < 0.15 && in.joint_age_sec < 0.15 &&
        in.imu.angular_velocity.allFinite() &&
        R_base(2, 2) > 0.97;

    const auto spin = spin_drive_.update(
        cmd.spin_request, spin_available, cmd.emergency_stop,
        controllers::SpinDrive::measuredHeading(in.imu.orientation, in.imu.has_orientation),
        (R_base * (omega_body - state_estimator_->getGyroBias())).z(),
        (R_base.transpose() * body_state.linear_velocity).head<2>(),
        driving_controller_->getSteeringError(),
        cmd.cmd_vel.head<2>(), in.ecc_angles, *kinematics_, dt);

    Eigen::Vector3d active_cmd_vel =
        (is_homing_ || cmd.spin_request != 0) ? Eigen::Vector3d::Zero() : cmd.cmd_vel;
    if (spin.ownsDrive()) {
        active_cmd_vel = spin.twist;
    }
    if (task_controller_->isActive()) {
        active_cmd_vel = Eigen::Vector3d(task_controller_->getForwardVelocityOverride(), 0.0, 0.0);
    }

    if (!task_controller_->isActive()) {
        const bool is_stopped = (active_cmd_vel.norm() < 1e-3);
        terrain_estimator_->update(foot_pos_in_base, R_base, contact_valid, is_stopped, dt);
    }
    const TerrainState terrain_state = terrain_estimator_->getState();

    // Phase 3: Contact Detection
    const Eigen::Vector3d v_body       = R_base.transpose() * body_state.linear_velocity;
    const double chassis_forward_vel   = v_body.x();
    const double imu_accel_x           = in.imu_received ? in.imu.linear_acceleration.x() : 0.0;
    const double imu_pitch_rate        = in.imu_received ? in.imu.angular_velocity.y() : 0.0;

    contact_detector_->update(
        in.ecc_efforts, in.wheel_efforts, in.wheel_velocities,
        chassis_forward_vel, active_cmd_vel.x(), imu_accel_x,
        climb_geometry_estimator_->getGeometry(), dt,
        active_cmd_vel.y(), active_cmd_vel.z(), imu_pitch_rate);

    // Phase 4: FSM Task Controller
    if (!is_homing_ && !cmd.emergency_stop && !spin.ownsDrive() && cmd.spin_request == 0) {
        const auto& geom = climb_geometry_estimator_->getGeometry();
        if (!task_controller_->isActive() && cmd.cmd_vel(0) < -0.05) {
            contact_detector_->reset();
        }
        task_controller_->setNormalApproachAllowed(
            user_posture_config_ == PostureConfiguration::OUTWARD &&
            std::abs(cmd.cmd_vel.y()) < 0.08 &&
            std::abs(cmd.cmd_vel.z()) < 0.25 &&
            in.steer_angles.cwiseAbs().maxCoeff() < 0.25);

        const bool was_active = task_controller_->isActive();
        task_controller_->update(
            *contact_detector_, geom, in.ecc_angles, in.ecc_efforts,
            dt, chassis_forward_vel, cmd.cmd_vel(0));
        if (was_active && !task_controller_->isActive()) {
            climb_geometry_estimator_->reset();
            contact_detector_->reset();
        }
    }

    if (climb_mode_ == ClimbMode::PAPER_NORMAL && task_controller_->isActive()) {
        active_cmd_vel = Eigen::Vector3d(task_controller_->getForwardVelocityOverride(), 0.0, 0.0);
    }

    const std::string curr_state_name = task_controller_->getStateName();
    out.curr_fsm_state = curr_state_name;
    if (curr_state_name != prev_fsm_state_name_) {
        out.fsm_state_changed = true;
        out.prev_fsm_state    = prev_fsm_state_name_;
        if (prev_fsm_state_name_ == "REAR_PLACED" && curr_state_name == "IDLE") {
            terrain_estimator_->reset();
            state_estimator_->reset();
            climb_geometry_estimator_->reset();
            contact_detector_->reset();
            balance_controller_->syncCurrentAngles(in.ecc_angles);
        }
        prev_fsm_state_name_ = curr_state_name;
    }

    // Phase 5: Driving Controller & Auto-Strafe Stance
    auto [target_steer, target_wheel] = driving_controller_->update(
        active_cmd_vel, in.steer_angles, in.ecc_angles, dt,
        cmd.emergency_stop, is_homing_, in.steer_velocities, in.wheel_velocities,
        in.ecc_velocities, contact_valid, spin.alignment_yaw_rate, spin.ownsDrive());
    prev_target_wheel_ = target_wheel;

    updateAutoStrafePosture(
        active_cmd_vel, in.steer_angles, spin.ownsDrive(), cmd.emergency_stop, dt);

    // Phase 6: Balance Controller
    const double active_height = (is_homing_ || spin.ownsDrive()) ? 0.205 : cmd.target_height;
    double active_roll         = is_homing_ ? 0.0 : cmd.target_roll;
    const double active_pitch  = is_homing_ ? 0.0 : cmd.target_pitch;
    if (!is_homing_ && !spin.ownsDrive()) {
        active_roll += driving_controller_->getBankAngle();
    }

    TerrainState active_terrain_state = terrain_state;
    if (is_homing_) {
        active_terrain_state.pitch_slope = 0.0;
        active_terrain_state.roll_slope  = 0.0;
        active_terrain_state.slope_angle = 0.0;
        active_terrain_state.normal      = Eigen::Vector3d::UnitZ();
        active_terrain_state.rotation    = Eigen::Matrix3d::Identity();
        active_terrain_state.is_valid    = true;
    } else if (task_controller_->isActive()) {
        active_terrain_state.pitch_slope = 0.0;
        active_terrain_state.normal      = Eigen::Vector3d(
            0.0, -std::sin(active_terrain_state.roll_slope), std::cos(active_terrain_state.roll_slope));
        active_terrain_state.rotation = Eigen::AngleAxisd(
            active_terrain_state.roll_slope, Eigen::Vector3d::UnitX()).toRotationMatrix();
    }

    const Eigen::Vector3d balance_twist =
        spin.ownsDrive() ? Eigen::Vector3d::Zero() : active_cmd_vel;
    const ControlOutput balance_out = balance_controller_->updateHybrid(
        active_height, active_roll, active_pitch,
        target_steer, in.ecc_angles, body_state,
        contact_points_world, contact_valid, active_terrain_state, dt,
        cmd.emergency_stop, balance_twist,
        climb_mode_ != ClimbMode::STAIR_200MM,
        (task_controller_->isActive() || spin.ownsDrive()) ? nullptr : wheel_support);

    // Phase 7: FSM Overrides & Blend-Out
    Eigen::Vector4d target_ecc                    = balance_out.ecc_angles;
    const Eigen::Vector4d ecc_overrides_vec       = task_controller_->getEccOverrides();
    const std::array<bool, NUM_LEGS> override_mask = task_controller_->getOverrideMask();
    const double blend_w                          = task_controller_->getBlendWeight();
    const bool task_active                        = task_controller_->isActive();

    if (task_active) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (override_mask[i]) {
                target_ecc(i) = ecc_overrides_vec(i);
            }
        }
    } else if (blend_w > 0.0) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (override_mask[i]) {
                const double delta = (climb_mode_ == ClimbMode::PAPER_NORMAL)
                    ? target_ecc(i) - ecc_overrides_vec(i)
                    : kinematics_->normalize_angle(target_ecc(i) - ecc_overrides_vec(i));
                target_ecc(i) = ecc_overrides_vec(i) + (1.0 - blend_w) * delta;
            }
        }
    }

    Eigen::Vector4d target_ecc_torques   = balance_out.ecc_torques;
    Eigen::Vector4d target_wheel_torques = driving_controller_->getWheelTorques();
    if (task_active) {
        const auto wheel_override_mask = task_controller_->getWheelOverrideMask();
        const auto wheel_overrides     = task_controller_->getWheelSpeedOverrides();
        const auto climb_state         = task_controller_->getState();
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (override_mask[i]) {
                target_ecc_torques(i) = 0.0;
            }
            if (wheel_override_mask[i]) {
                target_wheel(i) = wheel_overrides(i);
            }

            if (climb_state == tasks::ClimbState::FRONT_WALL_CLIMB ||
                climb_state == tasks::ClimbState::FRONT_VAULT_180) {
                target_wheel_torques(i) += 1.5;
            } else if (climb_state == tasks::ClimbState::STRADDLE_LEVELING) {
                target_wheel_torques(i) += (i == FL || i == FR) ? 2.0 : 1.5;
            } else if (climb_state == tasks::ClimbState::REAR_BELLY_REST ||
                       climb_state == tasks::ClimbState::REAR_VAULT_LOOP) {
                target_wheel_torques(i) += 2.0;
            } else if (climb_state == tasks::ClimbState::REAR_CREEP_ADVANCE) {
                target_wheel_torques(i) += 1.2;
            } else if (climb_state == tasks::ClimbState::POST_CLIMB_RESET) {
                target_wheel_torques(i) += 1.5;
            } else if (climb_state == tasks::ClimbState::CLIMBED_RUN) {
                target_wheel_torques(i) += 1.0;
            }
        }
    }
    target_wheel_torques = target_wheel_torques.cwiseMax(-wheel_max_torque_).cwiseMin(wheel_max_torque_);

    out.target_steer_angles  = target_steer;
    out.target_ecc_angles    = target_ecc;
    out.target_ecc_torques   = target_ecc_torques;
    out.target_wheel_speeds  = target_wheel;
    out.target_wheel_torques = target_wheel_torques;
    out.ecc_override_mask    = override_mask;
    out.task_active          = task_active;

    out.unsafe = !out.target_steer_angles.allFinite() ||
                 !out.target_ecc_angles.allFinite() ||
                 !out.target_ecc_torques.allFinite() ||
                 !out.target_wheel_speeds.allFinite() ||
                 !out.target_wheel_torques.allFinite();
    return out;
}

}  // namespace robot_control
