#include "robot_control/tasks/task_controller.hpp"
#include <cmath>
#include <algorithm>

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
    wheel_override_mask_.fill(false);
    active_lift_mask_.fill(false);
    forward_vel_override_ = 0.0;
    state_timer_ = 0.0;
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 0.0;
    cooldown_timer_ = 0.0;
    travel_distance_ = 0.0;
    stall_jam_timer_ = 0.0;
    current_front_target_ = 0.0;
    lift_start_angles_ = Eigen::Vector4d::Zero();
    front_target_angles_ = Eigen::Vector2d::Zero();
    rear_target_angles_ = Eigen::Vector2d::Zero();
    has_climbed_ = false;
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
    forward_vel_override_ = 0.0;
    wheel_speed_overrides_.setZero();
    wheel_override_mask_.fill(false);
    active_lift_mask_.fill(false);
    coordinated_lift_active_ = false;
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
    if (new_state == ClimbState::IDLE) {
        coordinated_lift_active_ = false;
    }
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

            // Head-on collision detection: confirmed by blocked slip, GMO external torque & IMU deceleration
            bool front_impact = contact_detector.frontImpactDetected();
            if (front_impact && (cmd_vx > 0.05 || body_vx > 0.05)) {
                lift_start_angles_ = current_ecc_angles;
                travel_distance_ = 0.0;
                transitionTo(ClimbState::FRONT_WALL_CLIMB);
            }
            break;
        }

        // ============================================================
        // 1. FRONT_WALL_CLIMB: 180-degree backward rotation to highest point (0 -> -pi rad, 1.05s, <= 30 RPM)
        // ============================================================
        // 1. FRONT_WALL_CLIMB: Front legs swing up to land on curb plateau (0 -> -2.10 rad, 0.70s, ~28.6 RPM)
        // ============================================================
        case ClimbState::FRONT_WALL_CLIMB: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            double duration = 0.70;
            double prog = std::clamp(state_timer_ / duration, 0.0, 1.0);
            double q_front = -prog * 2.10;

            ecc_overrides_(FL) = q_front;
            ecc_overrides_(FR) = q_front;

            // Rear legs held vertical (0.0 rad) to provide solid thrust
            ecc_overrides_(RL) = 0.0;
            ecc_overrides_(RR) = 0.0;

            // Forward momentum drive: rear wheels push ski ramp up onto curb
            wheel_speed_overrides_(FL) = 0.50 / params_.wheel_radius;
            wheel_speed_overrides_(FR) = 0.50 / params_.wheel_radius;
            wheel_speed_overrides_(RL) = 0.50 / params_.wheel_radius;
            wheel_speed_overrides_(RR) = 0.50 / params_.wheel_radius;
            forward_vel_override_ = 0.50;

            if (state_timer_ >= duration) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::STRADDLE_LEVELING);
            }
            break;
        }

        // ============================================================
        // 2. STRADDLE_LEVELING: Front wheels touch down on curb plateau and pull forward (0.40s)
        // ============================================================
        case ClimbState::STRADDLE_LEVELING: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            // Front wheels firmly on curb plateau at -2.10 rad
            ecc_overrides_(FL) = -2.10;
            ecc_overrides_(FR) = -2.10;

            // Rear legs held vertical
            ecc_overrides_(RL) = 0.0;
            ecc_overrides_(RR) = 0.0;

            // Front wheels pull forward, rear wheels push
            wheel_speed_overrides_.setConstant(0.50 / params_.wheel_radius);
            forward_vel_override_ = 0.50;

            if (state_timer_ >= 0.40) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_VAULT_LOOP);
            }
            break;
        }

        // ============================================================
        // 3. REAR_BELLY_REST: Pass-through alias
        // ============================================================
        case ClimbState::REAR_BELLY_REST: {
            transitionTo(ClimbState::REAR_VAULT_LOOP);
            break;
        }

        // ============================================================
        // 4. REAR_VAULT_LOOP: Rear legs rotate 180 degrees backward to highest point (0 -> -pi rad, 1.20s, <= 30 RPM)
        //    Front legs finish swinging from -2.10 rad to -pi rad (180 degrees highest point)
        // ============================================================
        case ClimbState::REAR_VAULT_LOOP: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            double duration = 1.20;
            double prog = std::clamp(state_timer_ / duration, 0.0, 1.0);

            // Front legs finish reaching highest point: -2.10 -> -pi rad
            double q_front = (1.0 - prog) * (-2.10) + prog * (-M_PI);
            ecc_overrides_(FL) = q_front;
            ecc_overrides_(FR) = q_front;

            // Rear legs rotate 180 degrees backward: 0 -> -pi rad
            double q_rear = -prog * M_PI;
            ecc_overrides_(RL) = q_rear;
            ecc_overrides_(RR) = q_rear;

            wheel_speed_overrides_.setConstant(0.50 / params_.wheel_radius);
            forward_vel_override_ = 0.50;

            if (state_timer_ >= duration) {
                has_climbed_ = true;
                travel_distance_ = 0.0;
                transitionTo(ClimbState::CLIMBED_RUN);
            }
            break;
        }

        // ============================================================
        // 5 & 6. REAR_CREEP_ADVANCE & POST_CLIMB_RESET: Pass-through aliases
        // ============================================================
        case ClimbState::REAR_CREEP_ADVANCE:
        case ClimbState::POST_CLIMB_RESET: {
            has_climbed_ = true;
            transitionTo(ClimbState::CLIMBED_RUN);
            break;
        }

        // ============================================================
        // 7. CLIMBED_RUN: All 4 legs hold highest point (-pi rad) on 200mm plateau
        // ============================================================
        case ClimbState::CLIMBED_RUN: {
            // All 4 legs locked at 180-deg highest point stance (-pi rad)
            override_mask_.fill(true);
            ecc_overrides_(FL) = -M_PI;
            ecc_overrides_(FR) = -M_PI;
            ecc_overrides_(RL) = -M_PI;
            ecc_overrides_(RR) = -M_PI;

            // Sustain forward drive until robot has cleared the plateau edge completely
            if (state_timer_ < 1.0 || travel_distance_ < 0.40) {
                wheel_override_mask_.fill(true);
                wheel_speed_overrides_.setConstant(0.50 / params_.wheel_radius);
                forward_vel_override_ = 0.50;
            } else {
                // Operator direct manual driving control
                wheel_override_mask_.fill(false);
                forward_vel_override_ = 0.0;
            }

            // Exit back to IDLE only if operator stops (cmd_vx < 0.05 for >= 1.0s)
            if (cmd_vx < 0.05 && state_timer_ >= 1.0) {
                blend_out_timer_ = 0.0;
                blend_out_weight_ = 1.0;
                cooldown_timer_ = params_.cooldown_duration;
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
