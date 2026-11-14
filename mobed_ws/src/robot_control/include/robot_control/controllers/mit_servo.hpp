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
                                     double target_velocity = 0.0) {
    constexpr double revolution = 6.2831853071795864769;
    // Choose the equivalent target closest to the measured encoder position.
    const double error = std::remainder(target - measured_position, revolution);
    return {measured_position + error, target_velocity, 360.0, 5.0, 0.0};
}

inline MitCommand eccentricMitCommand(double target) {
    // The posture controller already supplies a continuous angle.
    return {target, 0.0, 125.0, 4.8, 0.0};
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
