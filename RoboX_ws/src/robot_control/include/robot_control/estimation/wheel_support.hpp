#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <Eigen/Core>

namespace robot_control::estimation {

struct WheelSupport {
    // A negative load denotes a wheel whose load cannot currently be observed.
    Eigen::Vector4d normal_load = Eigen::Vector4d::Constant(-1.0);
    std::array<bool, 4> supporting = {true, true, true, true};
};

class WheelSupportEstimator {
public:
    void reset() { *this = WheelSupportEstimator{}; }

    const WheelSupport& update(const Eigen::Vector4d& eccentric_angle,
                               const Eigen::Vector4d& eccentric_velocity,
                               const Eigen::Vector4d& motor_torque,
                               double arm_length, double dt,
                               const double* measured_load = nullptr) {
        for (int i = 0; i < 4; ++i) {
            const double lever = arm_length * std::sin(eccentric_angle(i));
            double load = measured_load ? measured_load[i] : -1.0;
            if (!measured_load && std::abs(lever) >= 0.022 &&
                std::abs(eccentric_velocity(i)) <= 2.5 && std::isfinite(motor_torque(i))) {
                // The motor resists the ground torque. This quasi-static estimate is
                // deliberately unavailable near a vertical arm or during fast arm motion.
                load = std::max(0.0, -motor_torque(i) / lever);
            }
            if (!std::isfinite(load) || load < 0.0) {
                pending_time_[i] = 0.0;
                support_.normal_load(i) = -1.0;
                continue;
            }
            const double filtered = support_.normal_load(i);
            support_.normal_load(i) = filtered < 0.0 ? load
                : filtered + dt / (0.025 + dt) * (load - filtered);
            const bool was_supporting = support_.supporting[i];
            const bool transition = was_supporting ? load < 10.0 : load > 20.0;
            pending_time_[i] = transition ? pending_time_[i] + dt : 0.0;
            if (pending_time_[i] >= (measured_load ? 0.020 : 0.045)) {
                support_.supporting[i] = !was_supporting;
                pending_time_[i] = 0.0;
            }
        }
        return support_;
    }

private:
    WheelSupport support_;
    std::array<double, 4> pending_time_{};
};

}  // namespace robot_control::estimation
