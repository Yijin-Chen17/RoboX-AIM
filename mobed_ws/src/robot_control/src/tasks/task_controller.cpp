#include "robot_control/tasks/task_controller.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace robot_control {
namespace tasks {

TaskController::TaskController(const TaskControllerParams& params)
    : params_(params)
{
    reset();
}

void TaskController::reset() {
    state_ = ClimbState::IDLE;
    ecc_overrides_ = Eigen::Vector4d::Zero();
    override_mask_.fill(false);
    wheel_speed_overrides_.setZero();
    wheel_speed_command_.setZero();
    wheel_override_mask_.fill(false);
    active_lift_mask_.fill(false);
    forward_vel_override_ = 0.0;
    state_timer_ = 0.0;
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 0.0;
    cooldown_timer_ = 0.0;
    travel_distance_ = 0.0;
    stall_jam_timer_ = 0.0;
    condition_timer_ = 0.0;
    wheel_speed_command_.setZero();
    ecc_command_initialized_ = false;
    current_front_target_ = 0.0;
    lift_start_angles_ = Eigen::Vector4d::Zero();
    front_target_angles_ = Eigen::Vector2d::Zero();
    rear_target_angles_ = Eigen::Vector2d::Zero();
    has_climbed_ = false;
    climb_fault_latched_ = false;
    coordinated_lift_active_ = false;
    landing_rear_q_ = 0.0;

    for (auto& leg : leg_fsms_) {
        leg.state = LegState::CRUISE;
        leg.timer = 0.0;
        leg.start_angle = 0.0;
        leg.target_angle = 0.0;
        leg.impact_confirmed = false;
    }
}

void TaskController::abort() {
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 1.0;
    cooldown_timer_ = params_.cooldown_duration;
    travel_distance_ = 0.0;
    stall_jam_timer_ = 0.0;
    condition_timer_ = 0.0;
    forward_vel_override_ = 0.0;
    wheel_speed_overrides_.setZero();
    wheel_speed_command_.setZero();
    ecc_command_initialized_ = false;
    wheel_override_mask_.fill(false);
    active_lift_mask_.fill(false);
    coordinated_lift_active_ = false;
    climb_fault_latched_ = true;
    for (auto& leg : leg_fsms_) {
        leg.state = LegState::CRUISE;
        leg.timer = 0.0;
    }
    transitionTo(ClimbState::IDLE);
}

void TaskController::transitionTo(ClimbState new_state) {
    state_ = new_state;
    state_timer_ = 0.0;
    travel_distance_ = 0.0;
    stall_jam_timer_ = 0.0;
    condition_timer_ = 0.0;
    if (new_state == ClimbState::IDLE) {
        coordinated_lift_active_ = false;
    }
}

bool TaskController::confirmCondition(bool condition, double dt, double required_time) {
    if (condition) {
        condition_timer_ += dt;
    } else {
        // A broken contact/geometry condition invalidates the confirmation.
        // This deliberately has no latch: every transition must be true now.
        condition_timer_ = 0.0;
    }
    return condition_timer_ >= required_time;
}

void TaskController::setWheelSpeedCommand(double linear_speed, double dt) {
    Eigen::Vector4d speeds = Eigen::Vector4d::Constant(linear_speed);
    setWheelSpeedCommand(speeds, dt);
}

void TaskController::setWheelSpeedCommand(const Eigen::Vector4d& linear_speeds, double dt) {
    const double max_delta = params_.climb_wheel_accel * std::max(0.0, dt);
    const double radius = std::max(1e-4, params_.wheel_radius);
    for (int i = 0; i < NUM_LEGS; ++i) {
        const double target = linear_speeds(i) / radius;
        const double delta = std::clamp(target - wheel_speed_command_(i),
                                        -max_delta / radius,
                                         max_delta / radius);
        wheel_speed_command_(i) += delta;
        wheel_speed_overrides_(i) = wheel_speed_command_(i);
    }
}

Eigen::Vector4d TaskController::slewEccAngles(const Eigen::Vector4d& current,
                                               const Eigen::Vector4d& target,
                                               double dt) {
    // Sensor angles are normalized to [-pi, pi] by the controller core.  The
    // stair maneuver uses one 180-degree path 0 -> -pi and then holds the
    // highest stance.  Keep the command unwrapped while the override is
    // active so the branch cut at +/-pi cannot create a reverse jump.
    Eigen::Vector4d result = ecc_command_initialized_ ? ecc_overrides_ : current;
    const double max_delta = params_.climb_ecc_rate * std::max(0.0, dt);
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!ecc_command_initialized_) {
            // Anti-branch-cut unwrapping: seed within the same 2*pi domain as target
            double diff = target(i) - result(i);
            while (diff > M_PI)  { result(i) += 2.0 * M_PI; diff -= 2.0 * M_PI; }
            while (diff < -M_PI) { result(i) -= 2.0 * M_PI; diff += 2.0 * M_PI; }
        }
        result(i) += std::clamp(target(i) - result(i), -max_delta, max_delta);
    }
    return result;
}

double TaskController::computeFrontTargetAngle(double h_curb) const {
    if (h_curb > 0.15) {
        return 0.0; // Math lowest point
    }
    double ratio = std::clamp((h_curb - 0.04) / (0.10 - 0.04), 0.0, 1.0);
    return 1.35 + ratio * (1.80 - 1.35);
}

double TaskController::computeFrontLiftAngle(double h_curb) const {
    return computeFrontTargetAngle(h_curb);
}

double TaskController::computeFrontPlateauAngle(double h_curb) const {
    return computeFrontTargetAngle(h_curb);
}

double TaskController::computeAdaptiveLiftAngle([[maybe_unused]] double q_start) const {
    return computeFrontTargetAngle(params_.target_curb_height);
}

double TaskController::computeRearLevelingAngle([[maybe_unused]] double q_front_avg, double h_curb) const {
    double ratio = std::clamp((h_curb - 0.04) / (0.20 - 0.04), 0.0, 1.0);
    return -0.55 + ratio * (-0.15 - (-0.55));
}

double TaskController::computeRearClimbTargetAngle(double h_curb) const {
    double ratio = std::clamp((h_curb - 0.04) / (0.20 - 0.04), 0.0, 1.0);
    return -0.85 - ratio * (2.10 - 0.85);
}

std::string TaskController::getStateName() const {
    switch (state_) {
        case ClimbState::IDLE:               return "IDLE";
        case ClimbState::FRONT_WALL_CLIMB:   return "FRONT_WALL_CLIMB";
        case ClimbState::STRADDLE_LEVELING:  return "STRADDLE_LEVELING";
        case ClimbState::REAR_BELLY_REST:    return "REAR_BELLY_REST";
        case ClimbState::REAR_VAULT_LOOP:    return "REAR_VAULT_LOOP";
        case ClimbState::REAR_CREEP_ADVANCE: return "REAR_CREEP_ADVANCE";
        case ClimbState::POST_CLIMB_RESET:   return "POST_CLIMB_RESET";
        case ClimbState::CLIMBED_RUN:        return "CLIMBED_RUN";
        default:                             return "UNKNOWN";
    }
}

void TaskController::update(
    const estimation::ContactDetector& contact_detector,
    const estimation::ClimbGeometry& geometry,
    const Eigen::Vector4d& current_ecc_angles,
    const Eigen::Vector4d& current_ecc_efforts,
    double dt,
    double body_vx,
    double cmd_vx)
{
    if (dt <= 0.0 || dt > 0.5) return;

    if (params_.climb_mode == ClimbMode::DISABLED) {
        if (state_ != ClimbState::IDLE) {
            abort();
        }
        handleBlendOut(dt);
        return;
    }

    if (params_.climb_mode == ClimbMode::PAPER_NORMAL) {
        updatePaperNormal(contact_detector, geometry, current_ecc_angles, current_ecc_efforts, dt, body_vx, cmd_vx);
    } else if (params_.climb_mode == ClimbMode::STAIR_200MM) {
        updateStair200mm(contact_detector, geometry, current_ecc_angles, current_ecc_efforts, dt, body_vx, cmd_vx);
    }

    handleBlendOut(dt);
}

void TaskController::updatePaperNormal(
    [[maybe_unused]] const estimation::ContactDetector& contact_detector,
    [[maybe_unused]] const estimation::ClimbGeometry& geometry,
    [[maybe_unused]] const Eigen::Vector4d& current_ecc_angles,
    [[maybe_unused]] const Eigen::Vector4d& current_ecc_efforts,
    [[maybe_unused]] double dt,
    [[maybe_unused]] double body_vx,
    [[maybe_unused]] double cmd_vx)
{
    // Mode 1: IEEE RA-L 2026 paper standard normal mode.
    // Pure paper behavior: BalanceController + DrivingController fully govern posture,
    // terrain auto-leveling, and swerve drive.
    // TaskController maintains IDLE state with no overrides.
    if (state_ != ClimbState::IDLE) {
        abort();
        return;
    }

    state_ = ClimbState::IDLE;
    override_mask_.fill(false);
    forward_vel_override_ = 0.0;
    wheel_speed_overrides_.setZero();
    wheel_override_mask_.fill(false);
    active_lift_mask_.fill(false);
    coordinated_lift_active_ = false;
}

bool TaskController::rearAxleOnPlatform(const estimation::ClimbGeometry& geometry,
                                       const Eigen::Vector4d& eccentric) const {
    const auto at_landing_angle = [&](int leg) {
        const double error = std::remainder(eccentric(leg) - params_.rear_landing_angle, 2.0 * M_PI);
        return std::abs(error) < 0.30;
    };
    // Near the vertical stance, loaded eccentric joints can carry almost no
    // torque. Require geometry and attitude to agree before accepting landing.
    // Support both 200mm and 150mm steps starting from a cleared IDLE state,
    // as well as drift-free relative elevation (max_climbed_z + delta_height).
    const double minimum_height = std::min(getCurbHeight() - 0.05, 0.115);
    const bool height_confirmed =
        std::min(geometry.front_height, geometry.rear_height) >= minimum_height ||
        (geometry.max_climbed_z >= 0.120 && std::abs(geometry.delta_height) < 0.055);
    return at_landing_angle(RL) && at_landing_angle(RR) &&
           height_confirmed &&
           std::abs(geometry.delta_height) < 0.060 &&
           std::abs(geometry.front_z_vel) < 0.15 &&
           std::abs(geometry.body_pitch) < 0.18 &&
           std::abs(geometry.body_roll) < 0.18 &&
           std::abs(geometry.nose_up_pitch_rate) < 0.35;
}

void TaskController::updateStair200mm(
    const estimation::ContactDetector& contact_detector,
    const estimation::ClimbGeometry& geometry,
    const Eigen::Vector4d& current_ecc_angles,
    const Eigen::Vector4d& current_ecc_efforts,
    double dt,
    double body_vx,
    double cmd_vx)
{
    (void)current_ecc_efforts;
    state_timer_ += dt;

    // A climb command is never allowed to continue through a clearly unsafe
    // body attitude or persistent rear-wheel spin.  The belly-rest phase is
    // intentionally allowed to have small clearance; only a large negative
    // margin is considered a fault here.
    const bool unsafe_attitude = std::abs(geometry.body_pitch) > params_.climb_max_pitch ||
                                 std::abs(geometry.body_roll) > params_.climb_max_roll;
    if (state_ != ClimbState::IDLE && unsafe_attitude) {
        std::cout << "[ABORT line 253] unsafe_attitude=" << unsafe_attitude
                  << " (pitch=" << geometry.body_pitch << ", roll=" << geometry.body_roll << ")\n";
        abort();
        return;
    }

    if (state_ == ClimbState::IDLE) {
        ecc_command_initialized_ = false;
    } else if (!ecc_command_initialized_) {
        // Seed the unwrapped command cleanly centered at 0.0
        ecc_overrides_ = current_ecc_angles;
        for (int i = 0; i < NUM_LEGS; ++i) {
            while (ecc_overrides_(i) > M_PI)  ecc_overrides_(i) -= 2.0 * M_PI;
            while (ecc_overrides_(i) <= -M_PI) ecc_overrides_(i) += 2.0 * M_PI;
        }
        ecc_command_initialized_ = true;
    }

    switch (state_) {
        // ============================================================
        // 0. IDLE: High-stance approach, 100% manual user control
        // ============================================================
        case ClimbState::IDLE: {
            override_mask_.fill(false);
            wheel_override_mask_.fill(false);
            forward_vel_override_ = 0.0;
            wheel_speed_overrides_.setZero();

            if (cooldown_timer_ > 0.0) {
                cooldown_timer_ = std::max(0.0, cooldown_timer_ - dt);
                if (cooldown_timer_ <= 0.0) {
                    has_climbed_ = false;
                }
                break;
            }

            // Do not repeatedly re-trigger a failed climb while the operator
            // is still commanding forward motion.  A deliberate stop clears
            // the fault and arms a new attempt.
            if (climb_fault_latched_) {
                if (cmd_vx < 0.05 && std::abs(body_vx) < 0.05) {
                    climb_fault_latched_ = false;
                }
                break;
            }

            // Head-on collision detection: confirmed by blocked slip, GMO external torque & IMU deceleration
            bool front_impact = contact_detector.frontImpactDetected();
            bool front_stance_ready = (std::abs(current_ecc_angles(FL)) < 0.60 &&
                                       std::abs(current_ecc_angles(FR)) < 0.60);
            const bool approach_ready = front_impact && front_stance_ready &&
                (cmd_vx >= 0.10 || body_vx >= 0.10);
            const bool new_edge_confirmed = approach_ready &&
                (!has_climbed_ || (cmd_vx >= 0.10 && body_vx >= -0.05));
            if (new_edge_confirmed) {
                has_climbed_ = false;
                lift_start_angles_ = current_ecc_angles;
                travel_distance_ = 0.0;
                transitionTo(ClimbState::FRONT_WALL_CLIMB);
            }
            break;
        }

        // ============================================================
        // 1. FRONT_WALL_CLIMB: Front legs flip up to highest point (0 -> -pi rad)
        //    Front wheels bite curb top, belly drops onto bottom slide rails on 200mm curb corner
        // ============================================================
        case ClimbState::FRONT_WALL_CLIMB: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            double duration = params_.front_climb_max_time;
            Eigen::Vector4d target = Eigen::Vector4d::Zero();
            target(FL) = params_.front_landing_angle;
            target(FR) = params_.front_landing_angle;

            // Once the front eccentric arms swing past -2.5 rad into the step
            // lip under physical load, tilt the rear eccentric arms to +0.28 rad
            // so the 12 Nm rear posture joints push the chassis forward and pitch
            // the nose up over the 200mm step corner.
            const bool front_past_mid = (ecc_overrides_(FL) <= -2.5 || ecc_overrides_(FR) <= -2.5);
            const bool front_loaded = current_ecc_efforts.head<2>().cwiseAbs().maxCoeff() > 0.5;
            const double rear_assist = (front_past_mid && front_loaded)
                ? params_.rear_climb_assist_angle
                : 0.0;
            target(RL) = rear_assist;
            target(RR) = rear_assist;
            ecc_overrides_ = slewEccAngles(current_ecc_angles, target, dt);

            // Staged wheel linear speeds: moderate rear push during upward swing,
            // then full traction on both axles once the front tires engage the lip.
            const double v_front = front_past_mid ? 2.00 : 1.80;
            const double v_rear  = front_past_mid ? 1.50 : 1.00;
            Eigen::Vector4d front_climb_drive;
            front_climb_drive << v_front, v_front, v_rear, v_rear;
            setWheelSpeedCommand(front_climb_drive, dt);
            forward_vel_override_ = v_front;

            const double front_angle_err = current_ecc_angles.head<2>().unaryExpr([&](double angle) {
                return std::abs(std::remainder(angle - params_.front_landing_angle, 2.0 * M_PI));
            }).maxCoeff();
            const bool front_swing_reached = front_angle_err < 0.30;
            const bool front_near_top = front_angle_err < 0.55;

            // Keep the landing angle until the front axle carries load on the
            // upper tread and the chassis advances. On a 150mm step starting
            // from a cleared IDLE state, the step height (0.150m) is close to
            // the 0.144m eccentric stroke, so forward traction (body_vx >= 0.05)
            // is established at delta_height >= 0.125m as soon as the arms pass
            // near -pi (front_near_top).
            const double min_tread_delta_h = std::min(getCurbHeight() - 0.035, 0.120);
            const double front_tread_vx = body_vx +
                0.5 * (geometry.eccentric_rolling_velocity(FL) + geometry.eccentric_rolling_velocity(FR));
            const bool front_advancing = (body_vx >= 0.05) ||
                (front_near_top && geometry.nose_up_pitch < 0.035 && front_tread_vx >= 0.12);
            const bool front_rolling_on_tread = (front_swing_reached || front_near_top) &&
                geometry.front_platform_contact &&
                geometry.delta_height >= min_tread_delta_h &&
                front_advancing;
            if (state_timer_ >= params_.front_climb_min_time &&
                confirmCondition(front_rolling_on_tread, dt, 0.04)) {
                transitionTo(ClimbState::STRADDLE_LEVELING);
            } else if (state_timer_ >= duration) {
                std::cout << "[STAIR] Front axle did not establish traction on the tread\n";
                abort();
            }
            break;
        }

        // ============================================================
        // 2. STRADDLE_LEVELING: Slide Forward on Rails (通过滑轨一直往前滑)
        //    Front legs hold highest joint stance (-pi rad, "保持偏心关节最高")
        //    Chassis slides forward along rails while rear wheels push on ground
        // ============================================================
        case ClimbState::STRADDLE_LEVELING: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            // Front support was confirmed at the handoff. Return the front
            // arms toward vertical while the rear axle keeps pushing.
            Eigen::Vector4d target = Eigen::Vector4d::Zero();
            target.head<2>().setConstant(-M_PI);
            ecc_overrides_ = slewEccAngles(current_ecc_angles, target, dt);

            // Forward slide pull.  The rear axle remains at q=0 during this
            // phase and must keep pushing while the front axle is supported;
            // 0.60 m/s was insufficient to move the chassis to the required
            // pre-lift position on the 200 mm curb.
            // Manual test: use the maximum allowed wheel linear speed while
            // the front axle is on the plateau and the rear axle is not yet
            // lifted.
            double v_pull = 1.50;
            // Keep both rear wheels positively driven during the forward
            // slide.  The rear axle is still on the lower floor here and
            // must not be allowed to roll backward under the front-axle
            // support reaction.
            Eigen::Vector4d straddle_drive;
            straddle_drive << v_pull, v_pull, 1.00, 1.00;
            setWheelSpeedCommand(straddle_drive, dt);
            forward_vel_override_ = v_pull;

            // The front-landing handoff zeros this accumulator. Use signed
            // estimated body velocity so rollback subtracts from the advance.
            travel_distance_ += body_vx * dt;
            // A vertical arm can carry load with little measured effort.
            const double min_elevated_delta_h = std::min(getCurbHeight() - 0.05, 0.120);
            const bool front_still_elevated = geometry.delta_height >= min_elevated_delta_h;
            const bool is_lower_step = (geometry.max_climbed_z > 0.110 && geometry.max_climbed_z < 0.165);
            const double req_advance = is_lower_step ? 0.005 : params_.rear_lift_min_advance;
            if (front_still_elevated && body_vx >= 0.05 &&
                travel_distance_ >= req_advance) {
                transitionTo(ClimbState::REAR_BELLY_REST);
            } else if (state_timer_ >= params_.straddle_max_time) {
                std::cout << "[STAIR] Front support or forward progress was not sufficient for rear lift\n";
                abort();
            }
            break;
        }

        // ============================================================
        // 3. REAR_BELLY_REST: Lift the rear axle with the existing slew limit.
        // ============================================================
        case ClimbState::REAR_BELLY_REST: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            double duration = params_.rear_lift_max_time;
            Eigen::Vector4d target = Eigen::Vector4d::Constant(-M_PI);
            target(RR) = params_.rear_landing_angle;
            target(RL) = params_.rear_landing_angle;
            ecc_overrides_ = slewEccAngles(current_ecc_angles, target, dt);

            // On a 200mm step, belly-rail contact naturally regulates forward
            // speed (~0.42 m/s) while the rear legs fold up. On a 150mm step
            // (max_climbed_z < 0.165m), the belly rails are 50mm above the step
            // with zero drag; pacing the forward pull while the rear legs fold
            // past -2.4 rad prevents the rear wheels from hitting the step wall
            // before they are lifted above the corner, then bursts at 1.80 m/s.
            const bool is_lower_step = (geometry.max_climbed_z > 0.110 && geometry.max_climbed_z < 0.165);
            const bool rear_past_corner = (ecc_overrides_(RL) <= -2.4 && ecc_overrides_(RR) <= -2.4);
            const double v_rear_lift = is_lower_step
                ? (rear_past_corner ? 1.80 : 0.55)
                : 1.50;
            setWheelSpeedCommand(v_rear_lift, dt);
            forward_vel_override_ = v_rear_lift;

            const auto at_rear_landing = [&](double q) {
                return std::abs(std::atan2(
                    std::sin(q - params_.rear_landing_angle),
                    std::cos(q - params_.rear_landing_angle))) < 0.28;
            };
            const bool rear_lifted = at_rear_landing(current_ecc_angles(RL)) &&
                                     at_rear_landing(current_ecc_angles(RR));
            if (state_timer_ >= params_.rear_lift_min_time &&
                confirmCondition(rear_lifted, dt, params_.climb_confirm_time)) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_VAULT_LOOP);
            } else if (state_timer_ >= duration) {
                if (rear_lifted) {
                    travel_distance_ = 0.0;
                    transitionTo(ClimbState::REAR_VAULT_LOOP);
                } else {
                    abort();
                }
            }
            break;
        }

        // ============================================================
        // 4. REAR_VAULT_LOOP: rear eccentric arms have flipped.  The rear
        // wheels must now roll onto the plateau; leaving them at zero speed
        // leaves the axle hanging on the step edge (the failure mode seen in
        // the 200 mm climb).
        // ============================================================
        case ClimbState::REAR_VAULT_LOOP: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            double duration = params_.rear_vault_max_time;

            Eigen::Vector4d target = Eigen::Vector4d::Constant(-M_PI);
            target(RL) = params_.rear_landing_angle;
            target(RR) = params_.rear_landing_angle;
            ecc_overrides_ = slewEccAngles(current_ecc_angles, target, dt);

            // Keep maximum forward speed on the front axle and give the
            // rear axle the same speed command while it clears the curb.
            setWheelSpeedCommand(1.50, dt);
            forward_vel_override_ = 1.50;

            if (state_timer_ >= params_.rear_vault_min_time &&
                confirmCondition(rearAxleOnPlatform(geometry, current_ecc_angles), dt, 0.04)) {
                has_climbed_ = true;
                transitionTo(ClimbState::CLIMBED_RUN);
            } else if (state_timer_ >= duration) {
                std::cout << "[STAIR_200MM] Rear axle landing was not confirmed\n";
                abort();
            }
            break;
        }

        // Platform stance recovery begins as soon as the rear axle lands.
        case ClimbState::REAR_CREEP_ADVANCE:
        case ClimbState::POST_CLIMB_RESET:
        case ClimbState::CLIMBED_RUN: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            // From -pi, the front arms sweep forward through -3pi/2 and
            // the rear arms sweep backward through -pi/2. Both end vertical.
            Eigen::Vector4d target;
            target << -2.0 * M_PI, -2.0 * M_PI, 0.0, 0.0;
            ecc_overrides_ = slewEccAngles(current_ecc_angles, target, dt);

            // Keep a modest forward bias while canceling each arm's measured
            // rolling motion. Use the existing wheel speed slew limiter.
            // Keep the rear axle clear of the edge until stance recovery ends,
            // including when the operator releases the forward command.
            constexpr double v_recovery = 0.35;
            Eigen::Vector4d wheel_target = geometry.eccentric_rolling_velocity;
            wheel_target.array() += v_recovery;
            wheel_target = wheel_target.cwiseMax(-0.60).cwiseMin(0.60);
            setWheelSpeedCommand(wheel_target, dt);
            forward_vel_override_ = v_recovery;

            // Check if all 4 legs have recovered the upright standby posture
            const auto at_standby = [&](double q) {
                return std::abs(std::atan2(std::sin(q), std::cos(q))) < 0.35;
            };
            const bool all_legs_stood = at_standby(current_ecc_angles(FL)) &&
                                        at_standby(current_ecc_angles(FR)) &&
                                        at_standby(current_ecc_angles(RL)) &&
                                        at_standby(current_ecc_angles(RR));

            if (all_legs_stood && ecc_overrides_.isApprox(target)) {
                blend_out_timer_ = 0.0;
                blend_out_weight_ = 0.0;
                cooldown_timer_ = 0.0;
                travel_distance_ = 0.0;
                override_mask_.fill(false);
                wheel_override_mask_.fill(false);
                wheel_speed_command_.setZero();
                transitionTo(ClimbState::IDLE);
            } else if (state_timer_ >= params_.stand_up_timeout) {
                has_climbed_ = false;
                std::cout << "[STAIR_200MM] Stand-up did not complete\n";
                abort();
            }
            break;
        }
    }
}

void TaskController::handleBlendOut(double dt) {
    if (state_ == ClimbState::IDLE && blend_out_weight_ > 0.0) {
        blend_out_timer_ += dt;
        double progress = blend_out_timer_ / params_.blend_out_duration;
        blend_out_weight_ = std::max(0.0, 1.0 - progress);

        if (blend_out_weight_ > 0.0) {
            override_mask_.fill(true);
        } else {
            override_mask_.fill(false);
            active_lift_mask_.fill(false);
        }
    }
}

std::array<bool, NUM_LEGS> TaskController::getContactValid() const {
    std::array<bool, NUM_LEGS> valid = {true, true, true, true};
    if (state_ == ClimbState::FRONT_WALL_CLIMB) {
        valid[FL] = false;
        valid[FR] = false;
    } else if (state_ == ClimbState::REAR_BELLY_REST || state_ == ClimbState::REAR_VAULT_LOOP) {
        valid[RL] = false;
        valid[RR] = false;
    }
    return valid;
}

}  // namespace tasks
}  // namespace robot_control
