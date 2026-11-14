#pragma once

#include <Eigen/Dense>
#include <array>
#include <string>
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"
#include "robot_control/estimation/contact_detector.hpp"
#include "robot_control/estimation/climb_geometry_estimator.hpp"
#include "robot_control/tasks/trajectory_generator.hpp"
#include "robot_control/tasks/normal_climb_controller.hpp"

namespace robot_control {
namespace tasks {

/**
 * @brief Climbing FSM states.
 *
 * Implements high-speed dynamic momentum vault climbing (0.46s continuous traversal):
 *   IDLE -> VAULT_FRONT (0.15s) -> STRADDLE_RUN (0.16s) -> VAULT_REAR (0.15s) -> CLIMBED_RUN
 */
enum class ClimbState : int {
    IDLE                  = 0, // 待命冲刺: C++原生 0.22m 极限最高姿态 (q=0), 100% 手动操控
    FRONT_WALL_CLIMB      = 1, // 前探咬合翻越: 0 -> +1.25 rad (0.28s)
    STRADDLE_LEVELING     = 2, // 跨步前驱: 前轮在台阶顶面强拉 0.45m/s (+1.85 rad)
    REAR_BELLY_REST       = 3, // 后底盘搁置: 前轮制动, 后偏心折叠至 -1.20rad, 贴靠台阶角 (0.35s)
    REAR_VAULT_LOOP       = 4, // 后轮空中甩翻: 前轮拖车 0.38m/s, 后轮空中轻甩 -1.20 -> -4.65rad (0.30s)
    REAR_CREEP_ADVANCE    = 5, // 四轮爬行: 前行 50mm 完全脱离台阶边缘 (0.30s)
    POST_CLIMB_RESET      = 6, // Legacy entry into platform stance recovery
    CLIMBED_RUN           = 7, // Extend the eccentric arms before returning control
    // Backward-compatible aliases
    FRONT_VAULT_180       = 1,
    VAULT_FRONT           = 1,
    STRADDLE_BELLY_SLIDE  = 3,
    STRADDLE_RUN          = 2,
    VAULT_REAR            = 4,
    REAR_WALL_CLIMB       = 3,
    REAR_POST_CLIMB_RESET = 6,
    REAR_PLACED           = 7,
    REAR_TOUCHDOWN_SETTLE = 7,
    REAR_CREST_FOLD       = 7,
    STRADDLE_HOLD         = 0
};

/**
 * @brief Independent Single-Leg State Machine.
 */
enum class LegState : int {
    CRUISE          = 0, // High-stance cruise / approach, monitoring wheel contact
    VAULT_SWING     = 1, // Smooth S-curve swing
    PLANTED_SUPPORT = 2, // Support point, full traction pull
    COMPLETED       = 3  // Finished, continuous cruising
};

struct SingleLegFSM {
    LegState state = LegState::CRUISE;
    double timer = 0.0;
    double plateau_distance = 0.0;
    double start_angle = 0.0;
    double target_angle = 0.0;
    bool impact_confirmed = false;
};

/**
 * @brief Parameters for the climbing task controller.
 */
struct TaskControllerParams {
    double target_curb_height = 0.20;    // m (200mm high step challenge)
    double curb_margin = 0.015;           // m
    double wheelbase = 0.300;             // m
    double wheel_radius = 0.100;          // m

    // Mode Profile
    ClimbMode climb_mode = ClimbMode::PAPER_NORMAL;

    // Front-Vault 180 & Rear Belly-Rest Hybrid Parameters
    double front_vault_duration      = 0.24;  // s (front 180-deg vault swing)
    double front_vault_180_duration  = 0.24;  // alias
    double rear_belly_rest_duration  = 0.35;  // s (rear belly rest descent)
    double straddle_slide_duration   = 0.35;  // alias
    double rear_vault_duration       = 0.30;  // s (rear airborne whip over lip)
    double post_climb_reset_duration = 0.50;  // s (unified plateau lift/reset)
    double pull_through_vx           = 0.45;  // m/s (front wheel forward pull)
    double max_vault_torque          = 20.0;  // Nm (safe de-rated torque <= 20.0 Nm)
    double vault_traction_speed      = 1.00;  // m/s (cruising speed on plateau)
    double wheel_impact_torque_thresh = 1.8;  // Nm

    // Backward-compatible fields
    double vault_duration = 0.16;
    double front_straddle_duration = 0.25;
    double forward_drive_speed = 0.35;    // m/s
    double settling_distance = 0.25;      // m
    double settling_duration = 1.50;      // s
    double blend_out_duration = 0.30;     // s
    double cooldown_duration = 0.50;      // s
    double leveling_kp_pitch = 2.0;
    double leveling_kd_pitch = 0.3;
    double max_straddle_time_200mm = 8.0;
    double front_lift_duration = 0.40;
    double front_lift_margin = 0.020;
    double rear_proactive_prop_angle = -0.65;
    double front_roll_speed = 2.5;
    double rear_roll_speed = 2.5;
    bool active_pitch_counter_balance = true;

    // Event-driven stair FSM guards. Timers are safety bounds, not the
    // normal transition mechanism.
    double front_climb_min_time = 0.80;
    double front_climb_max_time = 2.20;
    double straddle_min_time = 0.80;
    double straddle_max_time = 2.00;
    double rear_lift_min_advance = 0.02; // m of net advance after front landing (early rear lift at full speed)
    double rear_lift_min_time = 0.80;
    double rear_lift_max_time = 2.00;
    double rear_lift_trigger_distance = 0.12;

    // 180-deg stair climb: eccentric arms rotate 0 -> -pi rad
    double rear_landing_angle = -3.141592653589793;  // -pi rad
    double front_landing_angle = -3.141592653589793 - 0.48; // -pi - 27.5 deg forward lean onto plateau (deeper lip reach & 12 Nm lever lift)
    double rear_climb_assist_angle = 0.28; // rad (+16 deg rear eccentric forward push & nose-up pitch assist during front lip climb)
    double rear_vault_min_time = 0.02;
    double rear_vault_max_time = 2.00;
    double stand_up_timeout = 1.10;

    double climb_confirm_time = 0.04;
    double climb_max_pitch = 0.75;       // rad, hard safety limit
    double climb_max_roll = 0.55;        // rad, hard safety limit
    double climb_max_slip_ratio = 0.97;
    // Posture (eccentric) joint rated speed is 36 rpm = 3.77 rad/s.  The FSM
    // command slew must stay below that, with margin for tracking error.
    double climb_ecc_rate = 3.5;         // rad/s = 33.4 rpm, FSM position slew limit
    double climb_wheel_accel = 8.0;      // m/s^2, wheel linear speed slew limit
};

class TaskController {
public:
    static constexpr double MIN_SINGULARITY_INCLINATION = 0.12;

    explicit TaskController(const TaskControllerParams& params = TaskControllerParams());

    void update(const estimation::ContactDetector& contact_detector,
                const estimation::ClimbGeometry& geometry,
                const Eigen::Vector4d& current_ecc_angles,
                const Eigen::Vector4d& current_ecc_efforts,
                double dt,
                double body_vx = 0.0,
                double cmd_vx = 0.20);

    void updatePaperNormal(const estimation::ContactDetector& contact_detector,
                           const estimation::ClimbGeometry& geometry,
                           const Eigen::Vector4d& current_ecc_angles,
                           const Eigen::Vector4d& current_ecc_efforts,
                           double dt,
                           double body_vx,
                           double cmd_vx);

    void updateStair200mm(const estimation::ContactDetector& contact_detector,
                          const estimation::ClimbGeometry& geometry,
                          const Eigen::Vector4d& current_ecc_angles,
                          const Eigen::Vector4d& current_ecc_efforts,
                          double dt,
                          double body_vx,
                          double cmd_vx);

    void update(const estimation::ContactDetector& contact_detector,
                const Eigen::Vector4d& current_ecc_angles,
                const Eigen::Vector4d& current_ecc_efforts,
                double dt,
                double body_vx = 0.0) {
        estimation::ClimbGeometry dummy;
        update(contact_detector, dummy, current_ecc_angles, current_ecc_efforts, dt, body_vx, 0.20);
    }

    void update(const estimation::ContactDetector& contact_detector,
                const Eigen::Vector4d& current_ecc_angles,
                double dt) {
        estimation::ClimbGeometry dummy;
        update(contact_detector, dummy, current_ecc_angles, Eigen::Vector4d::Zero(), dt, 0.0);
    }

    void setCurbHeight(double height) {
        params_.target_curb_height = std::clamp(height, 0.04, 0.22);
    }
    double getCurbHeight() const {
        return params_.climb_mode == ClimbMode::STAIR_200MM
            ? kStairHeight : params_.target_curb_height;
    }

    void setClimbMode(ClimbMode mode) {
        if (params_.climb_mode != mode) {
            reset(); // modes never inherit each other's commands, timers, or blend-out
        }
        params_.climb_mode = mode;
    }
    ClimbMode getClimbMode() const { return params_.climb_mode; }

    double computeFrontLiftAngle(double h_curb) const;
    double computeFrontPlateauAngle(double h_curb) const;
    double computeFrontTargetAngle(double h_curb) const;
    double computeAdaptiveLiftAngle(double q_start = 0.0) const;
    double computeRearLevelingAngle(double q_front_avg, double h_curb) const;
    double computeRearClimbTargetAngle(double h_curb) const;

    void setNormalApproachAllowed(bool allowed) { normal_climb_.setApproachAllowed(allowed); }

    void clearClimbedLock() { has_climbed_ = false; }
    bool isActive() const { return normalMode() ? normal_climb_.active() : state_ != ClimbState::IDLE; }
    bool hasClimbed() const { return normalMode() ? normal_climb_.hasClimbed() : has_climbed_; }
    ClimbState getState() const { return normalMode() ? static_cast<ClimbState>(normal_climb_.state()) : state_; }
    std::string getStateName() const;

    Eigen::Vector4d getEccOverrides() const { return normalMode() ? normal_climb_.angles() : ecc_overrides_; }
    std::array<bool, NUM_LEGS> getOverrideMask() const { return normalMode() ? normal_climb_.overrideMask() : override_mask_; }
    Eigen::Vector4d getWheelSpeedOverrides() const { return normalMode() ? Eigen::Vector4d::Constant(normal_climb_.speed() / params_.wheel_radius) : wheel_speed_overrides_; }
    std::array<bool, NUM_LEGS> getWheelOverrideMask() const { return normalMode() ? std::array<bool, NUM_LEGS>{false, false, false, false} : wheel_override_mask_; }
    double getForwardVelocityOverride() const { return normalMode() ? normal_climb_.speed() : forward_vel_override_; }
    double getBlendWeight() const { return normalMode() ? normal_climb_.blendWeight() : blend_out_weight_; }
    std::array<bool, NUM_LEGS> getContactValid() const;

    void abort();
    void reset();

    bool isCoordinatedLiftActive() const { return coordinated_lift_active_; }
    const std::array<SingleLegFSM, NUM_LEGS>& getLegFSMs() const { return leg_fsms_; }

private:
    bool normalMode() const { return params_.climb_mode == ClimbMode::PAPER_NORMAL; }
    NormalClimbController normal_climb_;
    static constexpr double kStairHeight = 0.20;
    bool rearAxleOnPlatform(const estimation::ClimbGeometry& geometry,
                           const Eigen::Vector4d& eccentric) const;

    TaskControllerParams params_;
    ClimbState state_ = ClimbState::IDLE;

    // Per-leg independent state machines
    std::array<SingleLegFSM, NUM_LEGS> leg_fsms_;

    // Per-leg override commands and mask
    Eigen::Vector4d ecc_overrides_;
    std::array<bool, NUM_LEGS> override_mask_;
    Eigen::Vector4d wheel_speed_overrides_ = Eigen::Vector4d::Zero();
    std::array<bool, NUM_LEGS> wheel_override_mask_ = {false, false, false, false};
    std::array<bool, NUM_LEGS> active_lift_mask_ = {false, false, false, false};
    double forward_vel_override_ = 0.0;

    double current_front_target_ = 0.0;

    // Timing
    double state_timer_ = 0.0;
    double blend_out_timer_ = 0.0;
    double cooldown_timer_ = 0.0;
    double travel_distance_ = 0.0;
    double stall_jam_timer_ = 0.0;
    double condition_timer_ = 0.0;
    Eigen::Vector4d wheel_speed_command_ = Eigen::Vector4d::Zero();
    bool ecc_command_initialized_ = false;

    double blend_out_weight_ = 0.0;
    bool has_climbed_ = false;
    bool climb_fault_latched_ = false;
    bool coordinated_lift_active_ = false;
    double landing_rear_q_ = 0.0;

    Eigen::Vector4d lift_start_angles_ = Eigen::Vector4d::Zero();
    Eigen::Vector2d front_target_angles_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d rear_target_angles_ = Eigen::Vector2d::Zero();

    void transitionTo(ClimbState new_state);
    void handleBlendOut(double dt);
    bool confirmCondition(bool condition, double dt, double required_time);
    void setWheelSpeedCommand(double linear_speed, double dt);
    void setWheelSpeedCommand(const Eigen::Vector4d& linear_speeds, double dt);
    Eigen::Vector4d slewEccAngles(const Eigen::Vector4d& current,
                                  const Eigen::Vector4d& target,
                                  double dt);
};

}  // namespace tasks
}  // namespace robot_control
