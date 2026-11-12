#include <cmath>
#include <cstdlib>
#include <iostream>

#include "robot_control/tasks/task_controller.hpp"

using robot_control::estimation::ClimbGeometry;
using robot_control::estimation::ContactDetector;
using robot_control::tasks::ClimbState;
using robot_control::tasks::TaskController;
using robot_control::tasks::TaskControllerParams;

namespace {

class FrontImpactDetector final : public ContactDetector {
public:
    bool frontImpactDetected() const override { return true; }
    bool rearImpactDetected() const override { return rear_impact; }
    bool rear_impact = false;
};

bool checkRearLiftAfterAdvance(bool unsafe_attitude, bool front_stalled = false, bool support_lost = false) {
    constexpr double dt = 0.01;
    constexpr double forward_speed = 0.25;
    constexpr int rollback_ticks = 10;
    TaskControllerParams params;
    params.climb_mode = robot_control::ClimbMode::STAIR_200MM;
    TaskController controller(params);
    FrontImpactDetector contact_detector;
    ClimbGeometry geometry;
    Eigen::Vector4d angles = Eigen::Vector4d::Zero();

    const auto tick = [&](double forward_speed) {
        controller.updateStair200mm(contact_detector, geometry, angles,
                                   Eigen::Vector4d::Zero(), dt, forward_speed, 0.35);
        angles = controller.getEccOverrides();
    };
    // A blocked wheel can register load and reach the commanded arm angle
    // without advancing. Keep driving and wait at the landing angle.
    geometry.front_platform_contact = true;
    geometry.delta_height = 0.20;
    tick(0.0);
    for (int i = 0; i < 130; ++i) {
        tick(0.0);
        if (controller.getState() != ClimbState::FRONT_WALL_CLIMB ||
            !angles.tail<2>().isZero() ||
            controller.getWheelSpeedOverrides().minCoeff() <= 0.0) return false;
    }
    if (!angles.head<2>().isApprox(Eigen::Vector2d::Constant(params.front_landing_angle))) return false;
    if (front_stalled) {
        for (int i = 0; i < 100 && controller.isActive(); ++i) {
            tick(0.0);
            if (!angles.tail<2>().isZero()) return false;
        }
        return controller.getState() == ClimbState::IDLE;
    }

    for (int i = 0; i < 10 && controller.getState() == ClimbState::FRONT_WALL_CLIMB; ++i) {
        tick(0.25);
    }
    if (controller.getState() != ClimbState::STRADDLE_LEVELING) return false;
    const double landing_angle = angles(0);
    for (int i = 0; i < rollback_ticks; ++i) {
        tick(-forward_speed);
        if (controller.getState() != ClimbState::STRADDLE_LEVELING ||
            !angles.tail<2>().isZero()) return false;
    }
    if (angles(0) <= landing_angle) return false;
    const int advance_ticks = static_cast<int>(
        std::ceil(params.rear_lift_min_advance / (forward_speed * dt)));
    for (int i = 1; i < advance_ticks + rollback_ticks; ++i) {
        tick(forward_speed);
        if (controller.getState() != ClimbState::STRADDLE_LEVELING ||
            !angles.tail<2>().isZero()) return false;
    }
    if (unsafe_attitude) geometry.body_pitch = params.climb_max_pitch + 0.01;
    if (support_lost) geometry.delta_height = 0.10;
    const int max_ticks = support_lost ? static_cast<int>(params.straddle_max_time / dt) + 1 : 2;
    for (int i = 0; i < max_ticks && controller.getState() == ClimbState::STRADDLE_LEVELING; ++i) {
        tick(forward_speed);
    }
    if (unsafe_attitude || support_lost) {
        return controller.getState() == ClimbState::IDLE && angles.tail<2>().isZero();
    }
    if (controller.getState() != ClimbState::REAR_BELLY_REST) return false;
    tick(0.25);
    return angles.tail<2>().maxCoeff() < 0.0;
}

bool checkOutwardStandUp() {
    constexpr double dt = 0.01;
    TaskControllerParams params;
    params.climb_mode = robot_control::ClimbMode::STAIR_200MM;
    TaskController controller(params);
    FrontImpactDetector contact_detector;
    ClimbGeometry geometry;
    geometry.front_platform_contact = true;
    geometry.front_height = 0.20;
    geometry.delta_height = 0.20;

    const auto tick = [&] {
        const Eigen::Vector4d measured_angles = controller.getEccOverrides().unaryExpr(
            [](double angle) { return std::remainder(angle, 2.0 * M_PI); });
        controller.updateStair200mm(contact_detector, geometry, measured_angles,
                                   Eigen::Vector4d::Zero(), dt, 0.35, 0.35);
    };
    for (int i = 0; i < 400 && controller.getState() != ClimbState::REAR_VAULT_LOOP; ++i) {
        tick();
    }
    for (int i = 0; i < 20; ++i) tick();
    if (controller.getState() != ClimbState::REAR_VAULT_LOOP) return false;

    geometry.rear_height = 0.20;
    geometry.delta_height = 0.0;
    for (int i = 0; i < 10 && controller.getState() == ClimbState::REAR_VAULT_LOOP; ++i) {
        tick();
    }
    if (controller.getState() != ClimbState::CLIMBED_RUN) return false;

    for (int i = 0; i < 95 && controller.isActive(); ++i) {
        const Eigen::Vector4d previous = controller.getEccOverrides();
        tick();
        const Eigen::Vector4d delta = controller.getEccOverrides() - previous;
        if (delta.head<2>().maxCoeff() > 1e-9 || delta.tail<2>().minCoeff() < -1e-9 ||
            delta.cwiseAbs().maxCoeff() > params.climb_ecc_rate * dt + 1e-9) {
            return false;
        }
    }

    const Eigen::Vector4d standing(-2.0 * M_PI, -2.0 * M_PI, 0.0, 0.0);
    return controller.getState() == ClimbState::IDLE && controller.hasClimbed() &&
           controller.getEccOverrides().isApprox(standing);
}

}  // namespace

int main() {
    if (!checkRearLiftAfterAdvance(false) || !checkRearLiftAfterAdvance(true) ||
        !checkRearLiftAfterAdvance(false, true) || !checkRearLiftAfterAdvance(false, false, true)) {
        std::cerr << "Front traction handoff or rear lift guard failed\n";
        return EXIT_FAILURE;
    }
    if (!checkOutwardStandUp()) {
        std::cerr << "Outward stand-up did not complete\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
