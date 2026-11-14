#include "robot_control/core/mobed_controller_core.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace robot_control {

MobedControllerCore::MobedControllerCore(const RobotParams& params)
    : robot_params_(params)
{
    setRobotModel(robot_params_.model_type);

    state_estimator_     = std::make_unique<estimation::StateEstimator>(robot_params_);
    state_estimator_->startStaticCalibration(50); // fast calibration in sim
    terrain_estimator_   = std::make_unique<estimation::TerrainEstimator>(robot_params_);
    contact_detector_    = std::make_unique<estimation::ContactDetector>(robot_params_);
    climb_geometry_estimator_ = std::make_unique<estimation::ClimbGeometryEstimator>(robot_params_);
    task_controller_     = std::make_unique<tasks::TaskController>();
    task_controller_->setClimbMode(climb_mode_);
    task_controller_->setCurbHeight(0.08); // adaptive 80mm default baseline

    reset();
}

void MobedControllerCore::setRobotModel(RobotModelType type) {
    robot_params_.applyModel(type);

    kinematics::MobedParameters k_params;
    k_params.length_x = robot_params_.length_x;
    k_params.width_y  = robot_params_.width_y;
    k_params.l_ecc    = robot_params_.l_ecc;
    k_params.r_wheel  = robot_params_.r_wheel;
    k_params.posture_z_offset = robot_params_.posture_z_offset;
    k_params.max_height = robot_params_.max_height;
    k_params.min_height = robot_params_.min_height;
    kinematics_ = std::make_shared<kinematics::MobedKinematics>(k_params);

    controllers::DrivingControllerParams drive_params;
    controllers::BalanceControllerParams balance_params;
    driving_controller_ = std::make_unique<controllers::DrivingController>(drive_params, kinematics_);
    balance_controller_ = std::make_unique<controllers::BalanceController>(balance_params, kinematics_);

    if (type == RobotModelType::ROBOX_V2) {
        steer_signs_ = {1.0, 1.0, 1.0, 1.0};
        ecc_signs_   = {1.0, -1.0, 1.0, -1.0};
        wheel_signs_ = {-1.0, 1.0, -1.0, 1.0};
        ecc_offsets_ = {M_PI, M_PI, M_PI, M_PI};
    } else {
        steer_signs_ = {1.0, 1.0, 1.0, 1.0};
        ecc_signs_   = {1.0, -1.0, -1.0, 1.0};
        wheel_signs_ = {1.0, -1.0, 1.0, -1.0};
        ecc_offsets_ = {-1.9003, -3.0526, 1.5320, -3.0473};
    }
    reset();
}

void MobedControllerCore::reset() {
    spin_drive_.reset();
    joints_initialized_ = false;
    is_homing_ = true;
    homing_timer_ = 0.0;
    strafe_outward_active_ = false;
    strafe_exit_timer_ = 0.0;
    prev_target_wheel_.setZero();
    prev_cad_ecc_.fill(0.0);

    if (state_estimator_) state_estimator_->reset();
    if (terrain_estimator_) terrain_estimator_->reset();
    if (contact_detector_) contact_detector_->reset();
    if (climb_geometry_estimator_) climb_geometry_estimator_->reset();
    if (task_controller_) task_controller_->reset();
}

void MobedControllerCore::update(const SensorInputs& in, const CommandInputs& cmd, double dt, ActuatorOutputs& out,
                                 int spin_request, bool normal_entry_allowed) {
    if (dt <= 0.0 || dt > 0.5) return;

    transition_smoother_.update(dt);

    // ================================================================
    // Phase 1: Unpack sensors & hardware polarities into math frame
    // ================================================================
    Eigen::Vector4d curr_steer_angles;
    Eigen::Vector4d curr_steer_velocities;
    Eigen::Vector4d curr_ecc_angles;
    Eigen::Vector4d curr_ecc_velocities;
    Eigen::Vector4d curr_ecc_efforts;
    Eigen::Vector4d curr_wheel_velocities;
    Eigen::Vector4d curr_wheel_efforts;

    for (int i = 0; i < 4; ++i) {
        // Steering
        curr_steer_angles(i)     = in.steer_pos[i] * steer_signs_[i];
        curr_steer_velocities(i) = in.steer_vel[i] * steer_signs_[i];

        // Eccentric posture
        double raw_q = (in.ecc_pos[i] - ecc_offsets_[i]) * ecc_signs_[i];
        while (raw_q > M_PI) raw_q -= 2.0 * M_PI;
        while (raw_q <= -M_PI) raw_q += 2.0 * M_PI;
        curr_ecc_angles(i)     = raw_q;
        curr_ecc_velocities(i) = in.ecc_vel[i] * ecc_signs_[i];
        curr_ecc_efforts(i)    = in.ecc_effort[i];

        // Wheels
        curr_wheel_velocities(i) = in.wheel_vel[i] * wheel_signs_[i];
        curr_wheel_efforts(i)    = in.wheel_effort[i] * wheel_signs_[i];
    }

    ClimbMode new_climb_mode = static_cast<ClimbMode>(cmd.climb_mode);

    if (!joints_initialized_) {
        for (int i = 0; i < 4; ++i) {
            prev_cad_ecc_[i] = in.ecc_pos[i];
        }
        joints_initialized_ = true;
        if (new_climb_mode == ClimbMode::STAIR_200MM) {
            transition_smoother_.start(curr_ecc_angles, curr_wheel_velocities, 0.0, 1.0, 0.60);
        }
    }

    // Unpack IMU
    ImuData imu;
    imu.linear_acceleration = Eigen::Vector3d(in.imu_accel[0], in.imu_accel[1], in.imu_accel[2]);
    imu.angular_velocity    = Eigen::Vector3d(in.imu_gyro[0],  in.imu_gyro[1],  in.imu_gyro[2]);
    imu.orientation         = Eigen::Quaterniond(in.imu_quat[0], in.imu_quat[1], in.imu_quat[2], in.imu_quat[3]);
    imu.has_orientation     = (in.imu_quat[0] != 0.0 || in.imu_quat[1] != 0.0 ||
                               in.imu_quat[2] != 0.0 || in.imu_quat[3] != 0.0);

    // Fast homing & preemption
    Eigen::Vector3d cmd_vel(cmd.vx, cmd.vy, cmd.wz);
    double target_height = (cmd.target_height >= 0.08 && cmd.target_height <= 0.22) ? cmd.target_height : 0.205;
    double target_roll   = std::clamp(cmd.target_roll, -0.35, 0.35);
    double target_pitch  = std::clamp(cmd.target_pitch, -0.35, 0.35);

    // Unpack user posture mode and autonomous climb mode
    user_posture_config_ = static_cast<PostureConfiguration>(cmd.posture_mode);
    if (new_climb_mode != climb_mode_) {
        // Mode transition detected
        if (climb_mode_ == ClimbMode::STAIR_200MM && new_climb_mode != ClimbMode::STAIR_200MM) {
            // Operator manually ended 200mm stair mode
            if (task_controller_->isActive()) {
                task_controller_->abort();
            }
        }
        climb_mode_ = new_climb_mode;
        task_controller_->setClimbMode(climb_mode_);
        climb_geometry_estimator_->reset();
        contact_detector_->reset();
        transition_smoother_.start(curr_ecc_angles, curr_wheel_velocities, 0.0, 1.0, 0.60);
    }

    if (is_homing_) {
        homing_timer_ += dt;
        if (!state_estimator_->isCalibrated()) {
            state_estimator_->updateStaticCalibration(imu);
        }
        bool has_teleop_input = (cmd_vel.norm() > 0.05);
        if (homing_timer_ > 0.5 || has_teleop_input || climb_mode_ == ClimbMode::STAIR_200MM) {
            is_homing_ = false;
        }
    }

    // ================================================================
    // Phase 2: State Estimation (15-State ESKF with contact gating)
    // ================================================================
    std::array<bool, NUM_LEGS> contact_valid = task_controller_->getContactValid();

    // Verify contact force: if |effort| < 1.0 Nm during motion, leg is airborne
    int supported_legs = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (contact_valid[i] && std::abs(curr_ecc_efforts(i)) >= 1.0) {
            supported_legs++;
        }
    }
    if (supported_legs >= 3) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (contact_valid[i] && std::abs(curr_ecc_efforts(i)) < 1.0) {
                contact_valid[i] = false;
            }
        }
    }

    state_estimator_->predict(imu, dt);
    state_estimator_->correctWithOdometry(
        curr_wheel_velocities, curr_steer_angles, curr_ecc_angles,
        curr_steer_velocities, curr_ecc_velocities, imu.angular_velocity, contact_valid);

    BodyState body_state = state_estimator_->getState();

    // ================================================================
    // Phase 2.5: Contact Points 3D & Terrain Estimation (Eq 4-6)
    // ================================================================
    std::array<Eigen::Vector3d, NUM_LEGS> foot_pos_in_base;
    std::array<Eigen::Vector3d, NUM_LEGS> contact_points_world;
    Eigen::Vector3d base_pos = body_state.position;
    Eigen::Matrix3d R_base = body_state.orientation.toRotationMatrix();

    for (int i = 0; i < NUM_LEGS; ++i) {
        foot_pos_in_base[i] = kinematics_->computeFootPositionInBase(
            i, curr_steer_angles(i), curr_ecc_angles(i));
        contact_points_world[i] = base_pos + R_base * foot_pos_in_base[i];
    }

    if (climb_mode_ != ClimbMode::DISABLED || task_controller_->isActive()) {
        climb_geometry_estimator_->update(
            body_state, contact_points_world, curr_ecc_angles,
            curr_ecc_efforts,
            curr_wheel_velocities, curr_wheel_efforts, prev_target_wheel_,
            imu.angular_velocity, task_controller_->getCurbHeight(), dt, curr_ecc_velocities);
    }

    const bool e_stop = cmd.e_stop != 0;
    if (e_stop && task_controller_->isActive()) {
        task_controller_->abort();
    }
    if (climb_mode_ == ClimbMode::PAPER_NORMAL && spin_request != 0 && task_controller_->isActive()) {
        task_controller_->abort();
    }

    const bool spin_available = robot_params_.model_type == RobotModelType::ROBOX_V2 &&
        !is_homing_ && !transition_smoother_.in_transition && !task_controller_->isActive() &&
        climb_mode_ != ClimbMode::STAIR_200MM && imu.angular_velocity.allFinite() &&
        R_base(2, 2) > 0.97;
    const auto spin = spin_drive_.update(
        spin_request, spin_available, e_stop,
        controllers::SpinDrive::measuredHeading(imu.orientation, imu.has_orientation),
        (R_base * (imu.angular_velocity - state_estimator_->getGyroBias())).z(),
        (R_base.transpose() * body_state.linear_velocity).head<2>(),
        driving_controller_->getSteeringError(),
        cmd_vel.head<2>(), curr_ecc_angles, *kinematics_, dt);
    Eigen::Vector3d active_cmd_vel =
        (is_homing_ || spin_request != 0) ? Eigen::Vector3d::Zero() : cmd_vel;
    if (spin.ownsDrive()) active_cmd_vel = spin.twist;
    if (task_controller_->isActive()) {
        double fsm_vx = task_controller_->getForwardVelocityOverride();
        active_cmd_vel = Eigen::Vector3d(fsm_vx, 0.0, 0.0);
    }

    bool climbing_active = task_controller_->isActive();
    if (!climbing_active) {
        bool is_stopped = (active_cmd_vel.norm() < 1e-3);
        terrain_estimator_->update(foot_pos_in_base, R_base, contact_valid, is_stopped);
    }
    TerrainState terrain_state = terrain_estimator_->getState();

    // ================================================================
    // Phase 3: Contact & Collision Detection
    // ================================================================
    Eigen::Vector3d v_body = R_base.transpose() * body_state.linear_velocity;
    double chassis_forward_vel = v_body.x();

    // Gravity-compensated body longitudinal dynamic acceleration
    // If sensor provides specific force (norm > 5.0 m/s^2), removes gravity projection
    double forward_accel_dyn = imu.linear_acceleration.x();
    if (imu.linear_acceleration.norm() > 5.0) {
        Eigen::Vector3d g_world(0.0, 0.0, -9.81);
        Eigen::Vector3d g_body = R_base.transpose() * g_world;
        forward_accel_dyn += g_body.x();
    }

    contact_detector_->update(
        curr_ecc_efforts, curr_wheel_efforts, curr_wheel_velocities,
        chassis_forward_vel, active_cmd_vel.x(), forward_accel_dyn,
        climb_geometry_estimator_->getGeometry(), dt,
        active_cmd_vel.y(), active_cmd_vel.z(),
        imu.angular_velocity.y());

    // ================================================================
    // Phase 4: Obstacle Climbing State Machine (Climb FSM)
    // ================================================================
    if (!is_homing_ && !e_stop && !spin.ownsDrive() && spin_request == 0) {
        const auto& geom = climb_geometry_estimator_->getGeometry();
        if (!task_controller_->isActive() && cmd_vel(0) < -0.05) {
            contact_detector_->reset();
        }

        task_controller_->setNormalApproachAllowed(
            (normal_entry_allowed || task_controller_->isActive()) &&
            user_posture_config_ == PostureConfiguration::OUTWARD &&
            std::abs(cmd_vel.y()) < 0.08 && std::abs(cmd_vel.z()) < 0.25 &&
            curr_steer_angles.cwiseAbs().maxCoeff() < 0.25);
        bool was_active = task_controller_->isActive();
        task_controller_->update(
            *contact_detector_, geom, curr_ecc_angles, curr_ecc_efforts, dt, chassis_forward_vel, cmd_vel(0));
        bool is_active = task_controller_->isActive();
        if (was_active != is_active) {
            // Reset geometry estimator both when a climb completes (active -> IDLE)
            // and when a new climb starts (IDLE -> active), so ground_baseline_z_
            // is freshly latched at the exact start of each step climb.
            climb_geometry_estimator_->reset();
            if (!is_active) {
                contact_detector_->reset();
            }
        }
    }

    if (climb_mode_ == ClimbMode::PAPER_NORMAL && task_controller_->isActive()) {
        active_cmd_vel = Eigen::Vector3d(task_controller_->getForwardVelocityOverride(), 0.0, 0.0);
    }

    // ================================================================
    // Phase 5: Driving Controller (Swerve IK + Coherent Scaling + Bank Angle)
    // ================================================================
    auto [target_steer, target_wheel] = driving_controller_->update(
        active_cmd_vel, curr_steer_angles, curr_ecc_angles, dt,
        e_stop, is_homing_, curr_steer_velocities, curr_wheel_velocities,
        curr_ecc_velocities, contact_valid, spin.alignment_yaw_rate, spin.ownsDrive());

    prev_target_wheel_ = target_wheel;

    // Auto-Strafe Outward Stance ("外八字") detection when pure strafing
    if (spin.ownsDrive()) {
        strafe_outward_active_ = false;
        strafe_exit_timer_ = 0.0;
        balance_controller_->setPostureConfiguration(PostureConfiguration::SPIN);
    } else if (user_posture_config_ == PostureConfiguration::OUTWARD &&
        !task_controller_->isActive() && !is_homing_ && !e_stop) {

        bool strafe_intent = (std::abs(active_cmd_vel.y()) > 0.05 &&
                              std::abs(active_cmd_vel.y()) >= 0.8 * std::abs(active_cmd_vel.x()));
        bool steer_aligned_strafe = (curr_steer_angles(0) < -0.80 &&
                                     curr_steer_angles(1) >  0.80 &&
                                     curr_steer_angles(2) >  0.80 &&
                                     curr_steer_angles(3) < -0.80);

        if (strafe_intent && steer_aligned_strafe) {
            strafe_outward_active_ = true;
            strafe_exit_timer_ = 0.0;
        } else if (strafe_outward_active_) {
            bool longitudinal_intent = (std::abs(active_cmd_vel.x()) > 0.10 &&
                                        std::abs(active_cmd_vel.x()) > std::abs(active_cmd_vel.y()));
            if (longitudinal_intent) {
                strafe_outward_active_ = false;
                strafe_exit_timer_ = 0.0;
            } else if (!strafe_intent) {
                strafe_exit_timer_ += dt;
                if (strafe_exit_timer_ > 0.25) {
                    strafe_outward_active_ = false;
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

    // ================================================================
    // Phase 6: Balance Controller (SRBD + QP + Bank Angle overlay)
    // ================================================================
    double effective_target_height = target_height;
    if (task_controller_->getClimbMode() == ClimbMode::STAIR_200MM && !task_controller_->hasClimbed()) {
        effective_target_height = 0.220; // 物理最高极限，保证 210.5mm 离地间隙 (q=0.0 rad 直立姿态)
    }
    double active_height = is_homing_ ? 0.205 : effective_target_height;
    double active_roll   = is_homing_ ? 0.0 : target_roll;
    double active_pitch  = is_homing_ ? 0.0 : target_pitch;

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
        active_terrain_state.normal = Eigen::Vector3d(0.0, -std::sin(active_terrain_state.roll_slope), std::cos(active_terrain_state.roll_slope));
        active_terrain_state.rotation = Eigen::AngleAxisd(
            active_terrain_state.roll_slope, Eigen::Vector3d::UnitX()).toRotationMatrix();
    }

    const Eigen::Vector3d balance_twist = spin.ownsDrive()
        ? Eigen::Vector3d::Zero() : active_cmd_vel;
    ControlOutput balance_out = balance_controller_->updateHybrid(
        active_height, active_roll, active_pitch,
        target_steer, curr_ecc_angles, curr_ecc_efforts, body_state,
        contact_points_world, contact_valid, active_terrain_state, dt, e_stop, balance_twist,
        climb_mode_ != ClimbMode::STAIR_200MM);

    Eigen::Vector4d target_ecc = balance_out.ecc_angles;

    // Direct 200mm Stair Mode Maximum Height Stance:
    // In STAIR_200MM mode before and between climbs: eccentric arms stand strictly vertically downwards (q = 0.0 rad, 6 o'clock)
    // to provide maximum ground clearance and full 144mm upward climbing stroke.
    if (climb_mode_ == ClimbMode::STAIR_200MM && !task_controller_->isActive()) {
        target_ecc.setZero();
    }

    // ================================================================
    // Phase 7: FSM Override (Replace legs if climbing)
    // ================================================================
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

    if (transition_smoother_.in_transition && !task_controller_->isActive()) {
        double alpha = transition_smoother_.getWeight();
        for (int i = 0; i < NUM_LEGS; ++i) {
            target_ecc(i) = (1.0 - alpha) * transition_smoother_.start_ecc_angles(i) + alpha * target_ecc(i);
        }
    }

    // ================================================================
    // Phase 8: Synthesize Actuator Outputs (Shortest-Arc CAD Unwrapping)
    // ================================================================
    for (int i = 0; i < 4; ++i) {
        // Steering
        out.steer_pos[i] = target_steer(i) * steer_signs_[i];
        out.steer_effort[i] = std::clamp(driving_controller_->getSteerTorques()(i) * steer_signs_[i], -10.0, 10.0);

        bool leg_is_overridden = task_controller_->isActive() && ecc_override_mask[i];

        // Keep the command continuous across successive climbs and revolutions.
        double target_cad = target_ecc(i) * ecc_signs_[i] + ecc_offsets_[i];
        double delta_cad = kinematics_->normalize_angle(target_cad - prev_cad_ecc_[i]);
        double continuous_cad = prev_cad_ecc_[i] + delta_cad;
        prev_cad_ecc_[i] = continuous_cad;
        out.ecc_pos[i] = continuous_cad;

        double ff_weight = transition_smoother_.getFeedforwardWeight();
        double effort_cmd = leg_is_overridden
                          ? 0.0
                          : balance_out.ecc_torques(i) * ecc_signs_[i] * ff_weight;

        if (climb_mode_ == ClimbMode::STAIR_200MM && !task_controller_->isActive()) {
            effort_cmd = 0.0;
        }

        // DM-J4340P rated torque is 12 Nm.  Its 40 Nm value is peak torque
        // and must not be used as a continuous position-control limit.
        effort_cmd = std::clamp(effort_cmd, -12.0, 12.0);
        out.ecc_effort[i] = effort_cmd;

        // Wheels
        double target_wheel_vel = target_wheel(i);
        if (task_controller_->isActive()) {
            auto wheel_override_mask = task_controller_->getWheelOverrideMask();
            if (wheel_override_mask[i]) {
                target_wheel_vel = task_controller_->getWheelSpeedOverrides()(i);
            }
        }
        if (transition_smoother_.in_transition && !task_controller_->isActive() &&
            !spin.ownsDrive() && !e_stop) {
            double alpha = transition_smoother_.getWeight();
            target_wheel_vel = (1.0 - alpha) * transition_smoother_.start_wheel_vels(i) + alpha * target_wheel_vel;
        }
        out.wheel_vel[i] = target_wheel_vel * wheel_signs_[i];

        // Traction torque injection (final wheel limit is the MS9010 2.2 Nm peak)
        double wheel_effort = driving_controller_->getWheelTorques()(i) * wheel_signs_[i];
        if (task_controller_->isActive()) {
            auto state = task_controller_->getState();
            if (state == tasks::ClimbState::FRONT_WALL_CLIMB || state == tasks::ClimbState::FRONT_VAULT_180) {
                wheel_effort += 1.5 * wheel_signs_[i];
            } else if (state == tasks::ClimbState::STRADDLE_LEVELING) {
                if (i == FL || i == FR) wheel_effort += 2.0 * wheel_signs_[i];
                else wheel_effort += 1.5 * wheel_signs_[i];
            } else if (state == tasks::ClimbState::REAR_BELLY_REST) {
                wheel_effort += 2.0 * wheel_signs_[i];
            } else if (state == tasks::ClimbState::REAR_VAULT_LOOP) {
                wheel_effort += 2.0 * wheel_signs_[i];
            } else if (state == tasks::ClimbState::REAR_CREEP_ADVANCE) {
                wheel_effort += 1.2 * wheel_signs_[i];
            } else if (state == tasks::ClimbState::POST_CLIMB_RESET) {
                wheel_effort += 1.5 * wheel_signs_[i];
            } else if (state == tasks::ClimbState::CLIMBED_RUN) {
                wheel_effort += 1.0 * wheel_signs_[i];
            }
        }
        wheel_effort = std::clamp(wheel_effort, -2.2, 2.2);
        out.wheel_effort[i] = wheel_effort;
    }

    // Telemetry & diagnostics
    out.fsm_state = static_cast<int>(task_controller_->getState());
    out.bank_angle = driving_controller_->getBankAngle();
    out.terrain_pitch = terrain_state.pitch_slope;
    out.terrain_roll = terrain_state.roll_slope;
}

} // namespace robot_control

// ----------------------------------------------------------------------------
// C-Linkage Implementations
// ----------------------------------------------------------------------------
extern "C" {

void* mobed_create() {
    return new robot_control::MobedControllerCore();
}

void mobed_destroy(void* handle) {
    if (handle) {
        delete static_cast<robot_control::MobedControllerCore*>(handle);
    }
}

void mobed_reset(void* handle) {
    if (handle) {
        static_cast<robot_control::MobedControllerCore*>(handle)->reset();
    }
}

void mobed_set_robot_model(void* handle, int model_type) {
    if (handle) {
        static_cast<robot_control::MobedControllerCore*>(handle)->setRobotModel(
            static_cast<robot_control::RobotModelType>(model_type));
    }
}

void mobed_update(void* handle,
                  const robot_control::SensorInputs* in,
                  const robot_control::CommandInputs* cmd,
                  double dt,
                  robot_control::ActuatorOutputs* out) {
    if (handle && in && cmd && out) {
        static_cast<robot_control::MobedControllerCore*>(handle)->update(*in, *cmd, dt, *out);
    }
}

int mobed_update_spin(void* handle, const robot_control::SensorInputs* in,
                      const robot_control::CommandInputs* cmd, double dt,
                      robot_control::ActuatorOutputs* out, int spin_request) {
    return mobed_update_spin_with_terrain(handle, in, cmd, dt, out, spin_request, 1);
}

int mobed_update_spin_with_terrain(void* handle, const robot_control::SensorInputs* in,
                                  const robot_control::CommandInputs* cmd, double dt,
                                  robot_control::ActuatorOutputs* out, int spin_request,
                                  int normal_entry_allowed) {
    if (!handle || !in || !cmd || !out) return 0;
    auto* core = static_cast<robot_control::MobedControllerCore*>(handle);
    core->update(*in, *cmd, dt, *out, spin_request, normal_entry_allowed != 0);
    return static_cast<int>(core->spinPhase());
}

}
