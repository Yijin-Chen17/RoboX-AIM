#include "robot_control/tasks/normal_climb_controller.hpp"
#include <algorithm>
#include <cmath>

namespace robot_control::tasks {

void NormalClimbController::reset() {
    *this = NormalClimbController{};
}

void NormalClimbController::enter(State state) {
    state_ = state;
    timer_ = confirmation_ = distance_ = 0.0;
}

bool NormalClimbController::confirm(bool condition, double dt) {
    confirmation_ = condition ? confirmation_ + dt : 0.0;
    return confirmation_ >= params_.confirmation_time;
}

void NormalClimbController::abort() {
    if (active()) {
        blend_weight_ = 1.0;
        blend_duration_ = std::max(params_.blend_duration,
            (cruise_ - command_).cwiseAbs().maxCoeff() / params_.recovery_rate);
    }
    speed_ = 0.0;
    fault_ = true;
    completed_ = false;
    cooldown_ = 0.5;
    enter(State::IDLE);
}

std::string NormalClimbController::stateName() const {
    switch (state_) {
        case State::IDLE: return "IDLE";
        case State::FRONT_LIFT: return "NORMAL_FRONT_LIFT";
        case State::ADVANCE: return "NORMAL_ADVANCE";
        case State::REAR_LIFT: return "NORMAL_REAR_LIFT";
        case State::CLEAR_EDGE: return "NORMAL_CLEAR_EDGE";
        case State::RECOVER: return "NORMAL_RECOVER";
    }
    return "UNKNOWN";
}

std::array<bool, NUM_LEGS> NormalClimbController::contactValid() const {
    std::array<bool, NUM_LEGS> valid{true, true, true, true};
    if (state_ == State::FRONT_LIFT) valid[FL] = valid[FR] = false;
    if (state_ == State::REAR_LIFT) valid[RL] = valid[RR] = false;
    return valid;
}

void NormalClimbController::update(const estimation::ContactDetector& contact,
                                  const estimation::ClimbGeometry& g,
                                  const Eigen::Vector4d& angles,
                                  double dt, double body_vx, double cmd_vx) {
    if (dt <= 0.0 || dt > 0.5) return;
    cooldown_ = std::max(0.0, cooldown_ - dt);
    height_difference_rate_ += std::clamp(dt / (0.05 + dt), 0.0, 1.0) *
        ((g.delta_height - previous_height_difference_) / dt - height_difference_rate_);
    previous_height_difference_ = g.delta_height;
    if (!active()) {
        speed_ = 0.0;
        blend_weight_ = std::max(0.0, blend_weight_ - dt / blend_duration_);
        if (blend_weight_ == 0.0) mask_.fill(false);
        if (cmd_vx < 0.05 && std::abs(body_vx) < 0.05) fault_ = false;
        obstacle_evidence_timer_ = std::max(0.0, obstacle_evidence_timer_ - dt);
        if (contact.frontImpactDetected() || (g.delta_height > 0.008 &&
            contact.getWheelExtTorques().head<2>().minCoeff() > 1.3))
            obstacle_evidence_timer_ = 1.5;
        // Ordinary rolling/balancing handles flat ground and small traversable
        // bumps. Only a confirmed obstruction starts the paper-style sequence.
        const bool outward = angles.head<2>().minCoeff() > 0.15 &&
                             angles.tail<2>().maxCoeff() < -0.15;
        if (outward && std::abs(g.delta_height) < 0.004 &&
            std::abs(g.body_pitch) < 0.04 &&
            contact.getWheelExtTorques().maxCoeff() < 0.8) {
            cruise_ = angles;
            cruise_valid_ = true;
        }
        if (!approach_allowed_ || fault_ || cooldown_ > 0.0 || blend_weight_ > 0.0 ||
            cmd_vx < 0.10 || !outward) { confirmation_ = 0.0; return; }
        // Restore the early stable controller's already-on-curb branch. The
        // relative height cancels odometry vertical drift; rolling and low
        // wheel resistance distinguish a landing from hanging over the edge.
        const bool front_placed = confirm(obstacle_evidence_timer_ > 0.0 && g.delta_height > 0.012 &&
            std::abs(height_difference_rate_) < 0.02 && body_vx > 0.10 &&
            contact.getWheelExtTorques().head<2>().maxCoeff() < 0.8, dt);
        if (!front_placed && !contact.frontImpactDetected()) return;
        if (std::abs(g.body_pitch) > 0.25 || std::abs(g.body_roll) > 0.20 ||
            g.delta_height > params_.max_step_height + 0.015) return;
        // Recover to the last level-ground stance, not the asymmetric
        // straddling stance observed after the front axle rolled onto a curb.
        if (!cruise_valid_) {
            const double nominal = std::clamp(angles.cwiseAbs().mean(), 0.4, 1.2);
            cruise_ << nominal, nominal, -nominal, -nominal;
        }
        command_ = angles;
        mask_.fill(true);
        height_ = front_placed ? g.delta_height : 0.0;
        support_ready_ = 0.0;
        previous_height_difference_ = g.delta_height;
        height_difference_rate_ = 0.0;
        completed_ = false;
        enter(front_placed ? State::ADVANCE : State::FRONT_LIFT);
    }

    timer_ += dt;
    distance_ = std::max(0.0, distance_ + body_vx * dt);
    if (!approach_allowed_ || cmd_vx < 0.05 || std::abs(g.body_pitch) > 0.35 ||
        std::abs(g.body_roll) > 0.30 || timer_ > (state_ == State::RECOVER ? params_.recovery_timeout : params_.phase_timeout) ||
        g.delta_height > params_.max_step_height + 0.015) {
        abort();
        return;
    }
    speed_ = params_.drive_speed;
    const Eigen::Vector4d previous = command_;
    switch (state_) {
        case State::FRONT_LIFT: {
            // Extend rear support while swinging the front pair toward the
            // curb. Height alone cannot confirm a wheel landing.
            command_.tail<2>().setZero();
            const bool landed = g.delta_height >= params_.min_step_height &&
                std::abs(height_difference_rate_) < 0.025 && body_vx > 0.20 && distance_ > 0.05 &&
                contact.getWheelExtTorques().head<2>().maxCoeff() < 1.0;
            if (!landed) {
                if (support_ready_ < 0.2) {
                    command_.head<2>().setZero();
                    if (angles.head<2>().cwiseAbs().maxCoeff() < 0.08)
                        support_ready_ += dt;
                } else {
                    command_.head<2>().array() -= params_.lift_rate * dt;
                }
            }
            if (confirm(landed, dt)) {
                height_ = std::clamp(g.delta_height, params_.min_step_height, params_.max_step_height);
                enter(State::ADVANCE);
            }
            break;
        }
        case State::ADVANCE: {
            // Retain front landing and rear support geometry while correcting
            // residual pitch until rear contact is actually observed.
            command_.head<2>().array() -= 1.5 * g.nose_up_pitch * dt;
            if (distance_ > 0.15 && std::abs(g.delta_height) < 0.008 &&
                        std::abs(height_difference_rate_) < 0.02 && body_vx > 0.10 && contact.getWheelExtTorques().tail<2>().maxCoeff() < 0.8) {
                enter(State::CLEAR_EDGE);
                break;
            }
            const bool rear_load_change =
                contact.getWheelExtTorques().tail<2>().maxCoeff() > 1.3 ||
                contact.getEccTorqueRates().tail<2>().cwiseAbs().maxCoeff() > 12.0;
            const bool rear_arrived = distance_ > 0.20 &&
                ((contact.rearImpactDetected() && body_vx < 0.12) ||
                 (rear_load_change && g.delta_height < height_ * 0.65 && body_vx > 0.15));
            if (confirm(rear_arrived, dt)) {
                command_.tail<2>() = angles.tail<2>();
                mask_.fill(true);
                enter(State::REAR_LIFT);
            }
            break;
        }
        case State::REAR_LIFT: {
            const bool landed = std::abs(g.delta_height) < 0.015 &&
                std::abs(height_difference_rate_) < 0.025 && body_vx > 0.15 && distance_ > 0.025 &&
                contact.getWheelExtTorques().tail<2>().maxCoeff() < 1.0;
            if (!landed) command_.tail<2>().array() -= params_.lift_rate * dt;
            command_.head<2>().array() -= 1.5 * g.nose_up_pitch * dt;
            if (confirm(landed, dt)) enter(State::CLEAR_EDGE);
            break;
        }
        case State::CLEAR_EDGE:
            if (confirm(std::abs(g.delta_height) < 0.015 && distance_ >= 0.10, dt))
                enter(State::RECOVER);
            break;
        case State::RECOVER:
            speed_ = 0.0; // finish posture recovery before resuming operator travel
            for (int i = 0; i < NUM_LEGS; ++i)
                command_(i) += std::clamp(cruise_(i) - command_(i),
                    -params_.recovery_rate * dt, params_.recovery_rate * dt);
            if (confirm((angles - cruise_).cwiseAbs().maxCoeff() < 0.08 &&
                        std::abs(g.delta_height) < 0.015 && std::abs(g.body_pitch) < 0.10, dt)) {
                completed_ = true;
                blend_weight_ = 1.0;
                blend_duration_ = params_.blend_duration;
                cooldown_ = 0.5;
                speed_ = 0.0;
                enter(State::IDLE);
            }
            break;
        case State::IDLE: break;
    }
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!mask_[i]) { command_(i) = angles(i); continue; }
        command_(i) = std::clamp(command_(i), -params_.max_ecc_angle, params_.max_ecc_angle);
        command_(i) = previous(i) + std::clamp(command_(i) - previous(i),
            -params_.lift_rate * dt, params_.lift_rate * dt);
    }
}
}  // namespace robot_control::tasks
