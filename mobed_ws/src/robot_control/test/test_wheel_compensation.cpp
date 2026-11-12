#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "robot_control/controllers/driving_controller.hpp"

namespace {
using namespace robot_control;
using robot_control::controllers::DrivingController;
using robot_control::controllers::DrivingControllerParams;
using robot_control::kinematics::MobedKinematics;

void expectNear(double actual, double expected, const char* message) {
    if (!(std::abs(actual - expected) < 1e-8)) {
        throw std::runtime_error(message);
    }
}

void checkRollingGeometry(const MobedKinematics& kinematics) {
    const Eigen::Vector3d twist(0.3, 0.15, 1.8);
    const Eigen::Vector4d eccentric(0.4, -0.5, 0.6, -0.7);
    const auto targets = kinematics.computeSteeringTargets(
        twist, Eigen::Vector4d::Zero(), eccentric);
    const auto& params = kinematics.getParams();

    for (int i = 0; i < NUM_LEGS; ++i) {
        const double offset = (i == FL || i == RL) ? 0.05345 : -0.05345;
        for (double heading : {targets.direct(i), targets.reversed(i)}) {
            const Eigen::Vector2d rolling_axis(std::cos(heading), std::sin(heading));
            const Eigen::Vector2d lateral_axis(-rolling_axis.y(), rolling_axis.x());
            const Eigen::Vector2d wheel_position =
                kinematics.computeFootPositionInBase(i, heading, eccentric(i)).head<2>() +
                offset * lateral_axis;
            const Eigen::Vector2d velocity = twist.head<2>() +
                twist.z() * Eigen::Vector2d(-wheel_position.y(), wheel_position.x());
            expectNear(lateral_axis.dot(velocity), 0.0, "steering heading leaves lateral slip");

            const auto speeds = kinematics.computeWheelRollingSpeeds(
                i, twist, heading, eccentric(i), 0.5, -0.3);
            expectNear(speeds.chassis, rolling_axis.dot(velocity) / params.r_wheel,
                       "traction must include the signed wheel offset");
            expectNear(speeds.joints,
                       (-offset * 0.5 - params.l_ecc * std::cos(eccentric(i)) * 0.3) /
                           params.r_wheel,
                       "joint compensation must include steer and eccentric velocity");
        }
    }
}

void checkCompensationUsesFeedback(const std::shared_ptr<MobedKinematics>& kinematics) {
    DrivingController controller(DrivingControllerParams{}, kinematics);
    const Eigen::Vector4d steer = Eigen::Vector4d::Constant(0.5);
    const Eigen::Vector4d zero = Eigen::Vector4d::Zero();
    const Eigen::Vector3d stopped = Eigen::Vector3d::Zero();
    const std::array<bool, NUM_LEGS> contacts{true, false, true, true};

    auto [steer_target, wheel_speed] = controller.update(stopped, steer, zero, 0.02);
    if (steer_target.isApprox(steer)) {
        throw std::runtime_error("test must command steering with the wheel stationary");
    }
    expectNear(wheel_speed.norm(), 0.0, "stalled steer must not generate compensation");

    std::tie(steer_target, wheel_speed) = controller.update(
        stopped, steer, zero, 0.02, false, false,
        Eigen::Vector4d::Constant(0.5), zero, zero, contacts);
    expectNear(wheel_speed(FL), -0.26725, "left steer compensation has the wrong sign");
    expectNear(wheel_speed(FR), 0.0, "lifted wheel must not receive compensation");
    expectNear(wheel_speed(RR), 0.26725, "right steer compensation has the wrong sign");

    std::tie(steer_target, wheel_speed) = controller.update(
        stopped, steer, zero, 0.02, true, false, Eigen::Vector4d::Constant(0.5));
    expectNear(wheel_speed.norm(), 0.0, "emergency stop must clear wheel speed immediately");
}

void checkSharedWheelSpeedLimit(const std::shared_ptr<MobedKinematics>& kinematics) {
    DrivingControllerParams params;
    params.enable_coherent_drive_scaling = false;
    params.max_wheel_speed = 6.0;
    DrivingController controller(params, kinematics);
    const Eigen::Vector4d zero = Eigen::Vector4d::Zero();
    const Eigen::Vector4d eccentric_rate = Eigen::Vector4d::Constant(0.5);
    const Eigen::Vector3d twist(0.8, 0.0, 1.0);
    Eigen::Vector4d speed;
    for (int i = 0; i < 40; ++i) {
        speed = std::get<1>(controller.update(
            twist, zero, zero, 0.01, false, false, zero, zero, eccentric_rate));
    }

    expectNear(speed.cwiseAbs().maxCoeff(), params.max_wheel_speed,
               "wheel speed allocator must use the available headroom");
    double common_scale = 0.0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        const auto rolling = kinematics->computeWheelRollingSpeeds(i, twist, 0.0, 0.0, 0.0, 0.5);
        const double scale = (speed(i) - rolling.joints) / rolling.chassis;
        if (i == 0) {
            common_scale = scale;
        }
        expectNear(scale, common_scale, "traction must be scaled uniformly after compensation");
    }
}
}  // namespace

int main() {
    try {
        const auto kinematics = std::make_shared<MobedKinematics>();
        checkRollingGeometry(*kinematics);
        checkCompensationUsesFeedback(kinematics);
        checkSharedWheelSpeedLimit(kinematics);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
