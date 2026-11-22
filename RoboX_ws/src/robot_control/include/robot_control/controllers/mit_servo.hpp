#pragma once

#include <algorithm>
#include <cmath>

namespace robot_control {
namespace controllers {

// All values refer to the joint output shaft, in radians, rad/s and N·m.
struct MitCommand {
    double position;
    double velocity;
    double kp;
    double kd;
    double torque_feedforward;
};

// Damiao DM-J4310P-2EC / DM-J4340P-2EC V1.1 hardware MIT protocol bounds:
// Kp in [0, 500], Kd in [0, 5.0] Nm·s/rad.
inline MitCommand steeringMitCommand(double target, double measured_position,
                                     double target_velocity, double kp, double kd,
                                     double pmax = 3.14159265358979323846) {
    constexpr double revolution = 6.2831853071795864769;
    // Choose the equivalent target closest to the measured encoder position.
    const double error = std::remainder(target - measured_position, revolution);
    const double desired_pos = std::clamp(measured_position + error, -pmax, pmax);
    return {
        desired_pos,
        target_velocity,
        std::clamp(kp, 0.0, 500.0),
        std::clamp(kd, 0.0, 5.0),
        0.0
    };
}

inline MitCommand steeringMitCommand(double target, double measured_position,
                                     double target_velocity = 0.0) {
    return steeringMitCommand(target, measured_position, target_velocity, 360.0, 5.0, 12.5);
}

inline MitCommand eccentricMitCommand(double target, double measured_position,
                                      double kp, double kd,
                                      double torque_feedforward = 0.0,
                                      double pmax = 3.14159265358979323846) {
    constexpr double revolution = 6.2831853071795864769;
    const double error = std::remainder(target - measured_position, revolution);
    const double desired_pos = std::clamp(measured_position + error, -pmax, pmax);
    return {
        desired_pos,
        0.0,
        std::clamp(kp, 0.0, 500.0),
        std::clamp(kd, 0.0, 5.0),
        std::clamp(torque_feedforward, -12.0, 12.0)
    };
}

inline MitCommand eccentricMitCommand(double target) {
    // The posture controller already supplies a continuous angle.
    return {target, 0.0, 125.0, 4.8, 0.0};
}

inline void limitMitFeedforward(MitCommand& command, double position,
                                double velocity, double torque_limit) {
    const double feedback = command.kp * (command.position - position)
                          + command.kd * (command.velocity - velocity);
    const double headroom = std::max(0.0, torque_limit - std::abs(feedback));
    command.torque_feedforward = std::clamp(command.torque_feedforward, -headroom, headroom);
}

// Drive-side model for simulation. On hardware, send the MitCommand fields
// to the motor's MIT interface so the drive closes the loop on its encoder.
inline double mitServoTorque(const MitCommand& command, double position,
                             double velocity, double torque_limit) {
    const double torque = command.kp * (command.position - position)
                        + command.kd * (command.velocity - velocity)
                        + command.torque_feedforward;
    return std::clamp(torque, -torque_limit, torque_limit);
}

}  // namespace controllers
}  // namespace robot_control
