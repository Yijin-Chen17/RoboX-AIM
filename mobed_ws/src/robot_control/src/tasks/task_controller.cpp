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
    travel_distance_ += std::max(0.0, body_vx) * dt;

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
            if (front_impact && front_stance_ready && (cmd_vx >= 0.10 || body_vx >= 0.10)) {
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

            // Rear legs held vertical (0.0 rad) providing solid thrust along the ground
            target(RL) = 0.0;
            target(RR) = 0.0;
            ecc_overrides_ = slewEccAngles(current_ecc_angles, target, dt);

            // Keep the original dynamic climbing speed.  The slew limiter
            // prevents a command step without turning the maneuver into a
            // slow quasi-static climb.
            // Manual test: increase front-wheel drive speed during the
            // initial wall climb.  The eccentric-joint rate remains limited
            // separately at 3.5 rad/s (within the 36 rpm hardware limit).
            double v_crawl = 1.50;
            // Keep the rear axle actively pushing the chassis into the curb
            // while the front eccentrics climb.  A single common speed lets
            // the rear wheels be driven backwards by the curb reaction once
            // the front axle starts to rise.
            Eigen::Vector4d front_climb_drive;
            front_climb_drive << v_crawl, v_crawl, 1.00, 1.00;
            setWheelSpeedCommand(front_climb_drive, dt);
            forward_vel_override_ = v_crawl;

            // A kinematic height estimate alone is not enough: the front
            // eccentric joints must also carry a stable load.  The 1.0 Nm
            // observability gate is the same support gate already used by
            // the state estimator; collision/drive torque thresholds are
            // intentionally not changed here.
            // Once the front edge observer has confirmed the front axle, do
            // not wait for a long slide: on the 200 mm step the rear axle
            // reaches the vertical edge quickly and must be lifted before it
            // becomes mechanically blocked.
            const auto front_half_turn_reached = [&]() {
                const double fl_error = std::abs(std::atan2(
                    std::sin(current_ecc_angles(FL) - params_.front_landing_angle),
                    std::cos(current_ecc_angles(FL) - params_.front_landing_angle)));
                const double fr_error = std::abs(std::atan2(
                    std::sin(current_ecc_angles(FR) - params_.front_landing_angle),
                    std::cos(current_ecc_angles(FR) - params_.front_landing_angle)));
                return fl_error < 0.30 && fr_error < 0.30;
            };
            const bool front_crest =
                front_half_turn_reached() &&
                (geometry.front_platform_contact || geometry.front_edge_crested || state_timer_ >= 0.95) &&
                std::abs(geometry.nose_up_pitch_rate) < 0.50;
            if (state_timer_ >= params_.front_climb_min_time &&
                confirmCondition(front_crest, dt, 0.05)) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::STRADDLE_LEVELING);
            } else if (state_timer_ >= duration) {
                if (front_half_turn_reached()) {
                    travel_distance_ = 0.0;
                    transitionTo(ClimbState::STRADDLE_LEVELING);
                } else {
                    abort();
                }
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

            double duration = params_.straddle_max_time;
            // Front legs hold upright highest joint stance (-pi rad)
            Eigen::Vector4d target = Eigen::Vector4d::Constant(0.0);
            // Hold the overshoot briefly, then reverse 90 degrees back to
            // the normal -pi support angle before lifting the rear axle.
            const bool front_return_started = state_timer_ >= 0.35;
            target(FL) = front_return_started ? -M_PI : params_.front_landing_angle;
            target(FR) = front_return_started ? -M_PI : params_.front_landing_angle;

            // Rear legs held vertical on ground (0.0 rad)
            target(RL) = 0.0;
            target(RR) = 0.0;
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

            const auto front_half_turn_reached = [&]() {
                const double fl_error = std::abs(std::atan2(
                    std::sin(current_ecc_angles(FL) - params_.front_landing_angle),
                    std::cos(current_ecc_angles(FL) - params_.front_landing_angle)));
                const double fr_error = std::abs(std::atan2(
                    std::sin(current_ecc_angles(FR) - params_.front_landing_angle),
                    std::cos(current_ecc_angles(FR) - params_.front_landing_angle)));
                return fl_error < 0.30 && fr_error < 0.30;
            };
            // Entry into REAR_BELLY_REST already means the front axle has
            // completed its support transition.  Do not re-evaluate it from
            // front/rear height difference while the rear legs are moving:
            // the rear lift itself necessarily changes delta_height.
            // Reaching -pi is only an actuator-angle condition.  It does not
            // prove that the front wheels are on the plateau; accepting it
            // here lets the front support reaction drag the rear axle
            // backward at the curb.
            const bool front_support = geometry.front_platform_contact ||
                                       geometry.front_edge_crested;
            // Start the rear lift 0.15 s after the front axle is confirmed
            // supported, as a manual timing test.
            const bool straddled = front_support &&
                                   state_timer_ >= params_.straddle_min_time &&
                                   std::abs(geometry.nose_up_pitch_rate) < 0.45;
            if ((state_timer_ >= params_.straddle_min_time &&
                 confirmCondition(straddled, dt, params_.climb_confirm_time)) ||
                state_timer_ >= (params_.straddle_min_time + 0.15)) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_BELLY_REST);
            } else if (state_timer_ >= duration) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_BELLY_REST);
            }
            break;
        }

        // ============================================================
        // 3. REAR_BELLY_REST: Rear wheels lift UP into the air ("然后后轮先抬起来")
        //    Rear legs flip to -pi rad (0.40s, lifting wheels off ground above curb)
        //    Front legs hold highest joint (-pi rad)
        // ============================================================
        case ClimbState::REAR_BELLY_REST: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            double duration = params_.rear_lift_max_time;
            // Keep the front axle at its current -pi support angle while the
            // rear axle performs its 0 -> -pi flip.
            Eigen::Vector4d target = Eigen::Vector4d::Constant(-M_PI);
            // Hold the front eccentrics at the extra 90-degree climbing
            // angle while the rear axle is being lifted.
            target(FL) = -M_PI;
            target(FR) = -M_PI;
            target(RR) = params_.rear_landing_angle;
            target(RL) = params_.rear_landing_angle;
            ecc_overrides_ = slewEccAngles(current_ecc_angles, target, dt);

            // Manual test: keep maximum forward speed on both axles during
            // the rear lift, so the rear wheels receive a corresponding
            // velocity command while the front axle keeps pulling.
            setWheelSpeedCommand(1.50, dt);
            forward_vel_override_ = 1.50;

            const auto front_half_turn_reached = [&]() {
                const double fl_error = std::abs(std::atan2(
                    std::sin(current_ecc_angles(FL) - params_.front_landing_angle),
                    std::cos(current_ecc_angles(FL) - params_.front_landing_angle)));
                const double fr_error = std::abs(std::atan2(
                    std::sin(current_ecc_angles(FR) - params_.front_landing_angle),
                    std::cos(current_ecc_angles(FR) - params_.front_landing_angle)));
                return fl_error < 0.30 && fr_error < 0.30;
            };
            // The front support is latched by entering this phase.  Rear-leg
            // motion changes delta_height, so do not invalidate front support
            // using the changing front/rear height difference.
            const bool front_support = geometry.front_platform_contact ||
                                       geometry.front_edge_crested ||
                                       front_half_turn_reached();
            const auto at_rear_landing = [&](double q) {
                return std::abs(std::atan2(
                    std::sin(q - params_.rear_landing_angle),
                    std::cos(q - params_.rear_landing_angle))) < 0.25;
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
            // Keep the front eccentrics at the extra 90-degree climbing
            // angle until the rear axle has actually landed.
            // Front axle has already returned to -pi in STRADDLE_LEVELING;
            // keep it there while the rear axle finishes climbing.
            target(FL) = -M_PI;
            target(FR) = -M_PI;
            target(RL) = params_.rear_landing_angle;
            target(RR) = params_.rear_landing_angle;
            ecc_overrides_ = slewEccAngles(current_ecc_angles, target, dt);

            // Keep maximum forward speed on the front axle and give the
            // rear axle the same speed command while it clears the curb.
            setWheelSpeedCommand(1.50, dt);
            forward_vel_override_ = 1.50;

            const bool rear_support = geometry.rear_platform_contact ||
                                      (geometry.rear_edge_crested &&
                                       geometry.rear_contact_confidence > 0.65);
            // TaskController does not receive wheel velocity directly.  The
            // non-zero rear command is known here, so body motion is used as
            // the rolling/traction part of this observation.
            const bool rear_rolls_with_body = body_vx > 0.04;
            // Height-based rear_platform_contact is a useful final check, but
            // it is not available while the rear axle is still climbing the
            // vertical face.  Use the existing eccentric load observer plus
            // rolling motion as the touchdown event in that transition.
            const bool rear_loaded_roll =
                geometry.rear_contact_confidence > 0.55 &&
                rear_rolls_with_body &&
                std::abs(geometry.nose_up_pitch_rate) < 0.45;
            const double h = std::max(0.04, params_.target_curb_height);
            const bool rear_height_candidate = geometry.rear_height > h - 0.10;
            // Load and motion only indicate that the rear axle is pushing at
            // the lip.  They do not prove that the wheels are on the upper
            // surface (the pictured failure was exactly this false positive).
            const bool rear_crest =
                (rear_support || geometry.rear_edge_crested ||
                 (rear_loaded_roll && rear_height_candidate)) &&
                rear_height_candidate;
            const auto at_rear_landing = [&](double q) {
                return std::abs(std::atan2(
                    std::sin(q - params_.rear_landing_angle),
                    std::cos(q - params_.rear_landing_angle))) < 0.25;
            };
            const bool rear_flipped = at_rear_landing(current_ecc_angles(RL)) &&
                                      at_rear_landing(current_ecc_angles(RR));
            if (state_timer_ >= params_.rear_vault_min_time &&
                confirmCondition(rear_crest, dt, params_.climb_confirm_time)) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::POST_CLIMB_RESET);
            } else if (state_timer_ >= duration) {
                if (rear_flipped) {
                    travel_distance_ = 0.0;
                    transitionTo(ClimbState::POST_CLIMB_RESET);
                } else {
                    std::cout << "[ABORT line 572] REAR_VAULT_LOOP timeout: state_timer=" << state_timer_
                              << ", duration=" << duration << ", rear_crest=" << rear_crest
                              << ", rear_support=" << rear_support << ", rear_height_candidate=" << rear_height_candidate
                              << " (rear_h=" << geometry.rear_height << ")\n";
                    abort();
                }
            }
            break;
        }

        // ============================================================
        // 5. REAR_CREEP_ADVANCE: Pass-through alias
        // ============================================================
        case ClimbState::REAR_CREEP_ADVANCE: {
            transitionTo(ClimbState::POST_CLIMB_RESET);
            break;
        }

        // ============================================================
        // 6. POST_CLIMB_RESET: Settle in the highest stance ("站起来")
        //    The climb uses only 0 -> -pi (180 degrees); do not rotate a
        //    second half-turn after the rear wheels regain support.
        // ============================================================
        case ClimbState::POST_CLIMB_RESET: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            double duration = params_.post_reset_max_time;
            // Return the front eccentrics by 90 degrees to the normal -pi
            // stance after the rear axle has landed.
            Eigen::Vector4d target = Eigen::Vector4d::Constant(-M_PI);
            target(FL) = -M_PI;
            target(FR) = -M_PI;
            target(RL) = params_.rear_landing_angle;
            target(RR) = params_.rear_landing_angle;
            ecc_overrides_ = slewEccAngles(current_ecc_angles, target, dt);

            // Gentle settling roll on the platform (0.25 m/s) to avoid overshooting short 700mm platform
            const double v_front_pull = 0.25;
            const double v_rear_roll = 0.15;
            Eigen::Vector4d settle_drive;
            settle_drive << v_front_pull, v_front_pull,
                            v_rear_roll, v_rear_roll;
            setWheelSpeedCommand(settle_drive, dt);
            forward_vel_override_ = v_front_pull;

            const bool rear_support = geometry.rear_platform_contact ||
                                      (geometry.rear_edge_crested &&
                                       geometry.rear_contact_confidence > 0.55);
            const double h = std::max(0.04, params_.target_curb_height);
            const auto rear_landing_reached = [&](double q) {
                return std::abs(std::atan2(
                    std::sin(q - params_.rear_landing_angle),
                    std::cos(q - params_.rear_landing_angle))) < 0.25;
            };
            const bool rear_loaded_stance =
                geometry.rear_contact_confidence > 0.40 &&
                rear_landing_reached(current_ecc_angles(RL)) &&
                rear_landing_reached(current_ecc_angles(RR));
            const bool rear_height_settled = geometry.rear_height > h - 0.10;
            const bool settled = (rear_support || rear_loaded_stance) &&
                                 rear_height_settled &&
                                 std::abs(geometry.body_pitch) < 0.55 &&
                                 std::abs(geometry.body_roll) < 0.30;
            if (state_timer_ >= params_.post_reset_min_time &&
                confirmCondition(settled, dt, params_.climb_confirm_time)) {
                has_climbed_ = true;
                travel_distance_ = 0.0;
                transitionTo(ClimbState::CLIMBED_RUN);
            } else if (state_timer_ >= duration) {
                has_climbed_ = true;
                travel_distance_ = 0.0;
                transitionTo(ClimbState::CLIMBED_RUN);
            }
            break;
        }

        // ============================================================
        // 7. CLIMBED_RUN: Platform Stance Recovery (Stand-Up to 0 rad)
        //    All 4 legs smoothly rotate -pi -> 0 rad to push the chassis up,
        //    recovering full 144mm stroke for the next step.
        //    Forward crawl is gently throttled to 0.15 m/s so robot does not
        //    overshoot the short 700mm platform.
        // ============================================================
        case ClimbState::CLIMBED_RUN: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            // Command all 4 legs to stand up from -pi to 0.0 rad
            Eigen::Vector4d target = Eigen::Vector4d::Zero();
            ecc_overrides_ = slewEccAngles(current_ecc_angles, target, dt);

            // Gentle forward crawl speed (0.15 m/s) on the short 700mm platform
            const double v_crawl = 0.15;
            setWheelSpeedCommand(v_crawl, dt);
            forward_vel_override_ = v_crawl;

            // Check if all 4 legs have recovered the upright standby posture
            const auto at_standby = [&](double q) {
                return std::abs(std::atan2(std::sin(q), std::cos(q))) < 0.35;
            };
            const bool all_legs_stood = at_standby(current_ecc_angles(FL)) &&
                                        at_standby(current_ecc_angles(FR)) &&
                                        at_standby(current_ecc_angles(RL)) &&
                                        at_standby(current_ecc_angles(RR));

            // Instant exit once stood up (~0.7s), or after at most 1.1s timeout.
            // Zero cooldown, zero required stop delay, immediately armed for Step 2!
            if ((state_timer_ >= 0.50 && all_legs_stood) || state_timer_ >= 1.10) {
                blend_out_timer_ = 0.0;
                blend_out_weight_ = 0.0;
                cooldown_timer_ = 0.0;
                has_climbed_ = false;
                travel_distance_ = 0.0;
                override_mask_.fill(false);
                wheel_override_mask_.fill(false);
                transitionTo(ClimbState::IDLE);
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
