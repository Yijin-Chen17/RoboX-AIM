#pragma once

#include <array>
#include <string>
#include "robot_control/estimation/contact_detector.hpp"

namespace robot_control::tasks {

// Separate state IDs, parameters, and memory from the 200 mm vault controller.
class NormalClimbController {
public:
    enum class State : int { IDLE = 0, FRONT_LIFT = 20, ADVANCE = 21,
                             REAR_LIFT = 22, CLEAR_EDGE = 23, RECOVER = 24 };
    struct Params {
        double max_step_height = 0.080;
        double min_step_height = 0.008;
        double drive_speed = 0.45;
        double lift_rate = 3.50;
        double recovery_rate = 0.50;
        double max_ecc_angle = 2.85; // limited folding, never the stair mode's full revolution
        double confirmation_time = 0.10;
        double phase_timeout = 6.0;
        double recovery_timeout = 10.0;
        double blend_duration = 0.40;
    };

    void update(const estimation::ContactDetector& contact,
                const estimation::ClimbGeometry& geometry,
                const Eigen::Vector4d& angles, double dt, double body_vx, double cmd_vx);
    void reset();
    void abort();
    void setApproachAllowed(bool allowed) { approach_allowed_ = allowed; }
    bool active() const { return state_ != State::IDLE; }
    bool hasClimbed() const { return completed_; }
    State state() const { return state_; }
    std::string stateName() const;
    const Eigen::Vector4d& angles() const { return command_; }
    std::array<bool, NUM_LEGS> overrideMask() const { return mask_; }
    std::array<bool, NUM_LEGS> contactValid() const;
    double speed() const { return speed_; }
    double blendWeight() const { return blend_weight_; }

private:
    void enter(State state);
    bool confirm(bool condition, double dt);
    Params params_;
    State state_ = State::IDLE;
    Eigen::Vector4d command_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d cruise_ = Eigen::Vector4d::Zero();
    std::array<bool, NUM_LEGS> mask_ = {false, false, false, false};
    double timer_ = 0.0;
    double confirmation_ = 0.0;
    double distance_ = 0.0;
    double height_ = 0.0;
    double previous_height_difference_ = 0.0;
    double height_difference_rate_ = 0.0;
    double speed_ = 0.0;
    double blend_weight_ = 0.0;
    double blend_duration_ = 0.40;
    double cooldown_ = 0.0;
    bool approach_allowed_ = true;
    bool fault_ = false;
    bool completed_ = false;
    bool cruise_valid_ = false;
    double support_ready_ = 0.0;
    double obstacle_evidence_timer_ = 0.0;
};
}  // namespace robot_control::tasks
