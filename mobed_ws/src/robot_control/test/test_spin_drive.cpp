#include <cmath>
#include <cstdlib>
#include <iostream>

#include "robot_control/controllers/spin_drive.hpp"

using robot_control::controllers::SpinDrive;
using robot_control::kinematics::MobedKinematics;

int main() {
    const MobedKinematics kinematics;
    const Eigen::Vector4d eccentric(0.8, -0.8, 0.8, -0.8);
    const Eigen::Vector2d forward(0.2, 0.0);
    const Eigen::Vector2d stopped = Eigen::Vector2d::Zero();
    for (int direction : {-1, 1}) {
        SpinDrive spin;
        const auto step = [&](int request, double heading, double rate, double error,
                              const Eigen::Vector2d& command, bool emergency = false) {
            return spin.update(request, true, emergency, heading, rate, stopped, error,
                               command, eccentric, kinematics, 0.01);
        };

        for (int i = 0; i < 40; ++i) {
            const auto output = step(direction, 3.0, 0.0, 0.3, forward);
            if (output.phase != SpinDrive::Phase::ALIGNING ||
                !output.twist.isZero() || direction * output.alignment_yaw_rate <= 0.0) {
                std::cerr << "Spin applied chassis traction before steering alignment\n";
                return EXIT_FAILURE;
            }
        }

        for (int i = 0; i < 720; ++i) {
            const double angle = direction * i * SpinDrive::kYawRate * 0.01;
            const double heading = std::remainder(3.0 + angle, 2.0 * M_PI);
            const double rate = i == 0 ? 0.0 : direction * SpinDrive::kYawRate;
            const auto output = step(direction, heading, rate, 0.0, forward);
            if (i < 150) continue;
            const Eigen::Vector2d field = Eigen::Rotation2Dd(angle) * output.twist.head<2>();
            if (output.phase != SpinDrive::Phase::SPINNING || field.x() < 0.10 ||
                std::abs(field.y()) > 1e-8 ||
                std::abs(output.twist.z() - direction * SpinDrive::kYawRate) > 1e-8) {
                std::cerr << "Spin lost the entry heading or steady yaw rate\n";
                return EXIT_FAILURE;
            }
            const auto targets = kinematics.computeSteeringTargets(
                output.twist, Eigen::Vector4d::Zero(), eccentric);
            for (int wheel = 0; wheel < robot_control::NUM_LEGS; ++wheel) {
                if (kinematics.isSteerInForbiddenSector(wheel, targets.direct(wheel)) &&
                    kinematics.isSteerInForbiddenSector(wheel, targets.reversed(wheel))) {
                    std::cerr << "Spin requested an infeasible rolling heading\n";
                    return EXIT_FAILURE;
                }
            }
        }

        const auto braking = step(0, 3.0, direction * SpinDrive::kYawRate, 0.0, stopped);
        const auto emergency = step(direction, 3.0, direction * SpinDrive::kYawRate, 0.0, stopped, true);
        const auto held = step(direction, 3.0, 0.0, 0.0, stopped);
        if (braking.phase != SpinDrive::Phase::STOPPING ||
            std::abs(braking.twist.z()) >= SpinDrive::kYawRate || !emergency.twist.isZero() || held.ownsDrive()) {
            std::cerr << "Spin exit or emergency-stop rearm failed\n";
            return EXIT_FAILURE;
        }
    }
}
