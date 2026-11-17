#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot_control/kinematics/mobed_kinematics.hpp"

namespace robot_control::controllers {

class SpinDrive {
public:
    static constexpr double kYawRate = 8.0;
    enum class Phase { IDLE, ALIGNING, SPINNING, STOPPING };

    struct Output {
        Eigen::Vector3d twist = Eigen::Vector3d::Zero();
        double alignment_yaw_rate = 0.0;
        Phase phase = Phase::IDLE;
        bool ownsDrive() const { return phase != Phase::IDLE; }
    };

    void reset() { *this = SpinDrive{}; }
    Phase phase() const { return phase_; }

    static double measuredHeading(const Eigen::Quaterniond& orientation, bool valid) {
        if (!valid || !orientation.coeffs().allFinite() || orientation.squaredNorm() < 1e-6) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const Eigen::Vector3d forward = orientation.normalized() * Eigen::Vector3d::UnitX();
        return std::atan2(forward.y(), forward.x());
    }

    Output update(int request, bool available, bool emergency_stop, double heading,
                  double yaw_rate, const Eigen::Vector2d& body_velocity, double steer_error,
                  const Eigen::Vector2d& translation, const Eigen::Vector4d& eccentric,
                  const kinematics::MobedKinematics& kinematics, double dt) {
        if (request == 0) require_release_ = false;
        if (emergency_stop) {
            reset();
            require_release_ = request != 0;
            return {};
        }

        const double speed = body_velocity.norm();
        const int direction = (request == 1 || request == -1) ? request : 0;
        const bool ready = available && std::isfinite(yaw_rate)
            && body_velocity.allFinite() && translation.allFinite() && eccentric.allFinite();
        if (phase_ == Phase::IDLE) {
            if (!ready || direction == 0 || require_release_) return {};
            direction_ = direction;
            relative_heading_ = 0.0;
            previous_heading_ = heading;
            phase_ = Phase::ALIGNING;
            elapsed_ = 0.0;
        } else if (ready) {
            const double heading_step = std::isfinite(heading) && std::isfinite(previous_heading_)
                ? std::remainder(heading - previous_heading_, 2.0 * M_PI) : yaw_rate * dt;
            relative_heading_ = std::remainder(relative_heading_ + heading_step, 2.0 * M_PI);
            previous_heading_ = heading;
        }

        if (phase_ != Phase::STOPPING && (!ready || direction != direction_)) {
            beginStop(direction != 0);
        }

        elapsed_ += dt;
        const double spin_target = direction_ * kYawRate;
        if (phase_ == Phase::ALIGNING) {
            if (elapsed_ >= 0.15 && steer_error < 0.10 && speed < 0.10 &&
                std::abs(yaw_rate) < 0.15) {
                phase_ = Phase::SPINNING;
            } else if (elapsed_ >= 3.0) {
                beginStop(true);
            }
        }

        if (phase_ == Phase::STOPPING && yaw_command_ == 0.0 &&
            ((std::abs(yaw_rate) < 0.12 && speed < 0.12) || elapsed_ >= 2.0)) {
            phase_ = Phase::IDLE;
            translation_.setZero();
            lateral_trim_.setZero();
            return {Eigen::Vector3d::Zero(), spin_target, Phase::STOPPING};
        }

        const double target_yaw = phase_ == Phase::SPINNING ? spin_target : 0.0;
        yaw_command_ += std::clamp(target_yaw - yaw_command_, -kYawAccel * dt, kYawAccel * dt);
        Eigen::Vector2d target_translation = Eigen::Vector2d::Zero();
        if (phase_ == Phase::SPINNING) {
            target_translation = translation *
                std::min(1.0, kMaxTranslation / std::max(translation.norm(), 1e-9));
        }
        const Eigen::Vector2d delta = target_translation - translation_;
        translation_ += delta * std::min(1.0, 0.8 * dt / std::max(delta.norm(), 1e-9));

        Eigen::Vector2d body_translation = Eigen::Vector2d::Zero();
        if (ready && yaw_command_ != 0.0) {
            const double limit = translationLimit(kinematics, eccentric, yaw_command_);
            const Eigen::Rotation2Dd to_body(-relative_heading_);
            const double target_speed = translation_.norm();
            if (target_speed > 0.05 && target_translation.norm() > 0.05) {
                // Trim cross-track velocity while preserving the bounded speed request.
                const Eigen::Vector2d direction_ref = translation_ / target_speed;
                const Eigen::Vector2d lateral(-direction_ref.y(), direction_ref.x());
                const double measured_lateral = lateral.dot(to_body.inverse() * body_velocity);
                const double trim = std::clamp(
                    lateral.dot(lateral_trim_) - 1.5 * dt * measured_lateral,
                    -0.4 * target_speed, 0.4 * target_speed);
                lateral_trim_ = trim * lateral;
            } else {
                lateral_trim_.setZero();
            }
            body_translation = to_body * (translation_ + lateral_trim_);
            body_translation *= std::min(limit, target_speed) /
                std::max(body_translation.norm(), 1e-9);
        }

        const double alignment = body_translation.isZero() ? spin_target : 0.0;
        return {Eigen::Vector3d(body_translation.x(), body_translation.y(), yaw_command_),
                alignment, phase_};
    }

private:
    // At either sector edge, lateral velocity from translation is bounded by
    // its magnitude. Opposite signs at the edges bracket a rolling solution
    // for every direction of translation up to the smaller edge magnitude.
    static double translationLimit(const kinematics::MobedKinematics& kinematics,
                                   const Eigen::Vector4d& eccentric, double yaw_rate) {
        const auto axes = kinematics.getSteerPositions();
        double limit = kMaxTranslation;
        for (int i = 0; i < NUM_LEGS; ++i) {
            const auto sector = kinematics.getLegSafeSectorConfig(i);
            const double half_width = sector.half_width - 0.04;
            const double reach = kinematics.getParams().l_ecc * std::sin(eccentric(i));
            const auto lateral = [&](double angle) {
                return yaw_rate * (std::cos(angle) * axes(i, 0) +
                                   std::sin(angle) * axes(i, 1) + reach);
            };
            const double left = lateral(sector.center_angle + half_width);
            const double right = lateral(sector.center_angle - half_width);
            if (left * right >= 0.0) return 0.0;
            limit = std::min({limit, std::abs(left), std::abs(right)});
        }
        return limit;
    }

    void beginStop(bool require_release) {
        phase_ = Phase::STOPPING;
        elapsed_ = 0.0;
        require_release_ = require_release;
    }

    static constexpr double kYawAccel = 8.0;
    static constexpr double kMaxTranslation = 0.20;
    Phase phase_ = Phase::IDLE;
    int direction_ = 1;
    bool require_release_ = false;
    double previous_heading_ = 0.0;
    double relative_heading_ = 0.0;
    double yaw_command_ = 0.0;
    double elapsed_ = 0.0;
    Eigen::Vector2d lateral_trim_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d translation_ = Eigen::Vector2d::Zero();
};

}  // namespace robot_control::controllers
