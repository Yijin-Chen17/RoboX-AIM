# DM/LK RoboX Wheel-Leg Chassis Controller

This node is for the 12-motor RoboX / MobED wheel-leg swerve chassis equipped with:
- **4× DM-J4310P-2EC** steering motors (MIT mode)
- **4× DM-J4340P-2EC V1.1** eccentric arm motors (MIT mode + feedforward torque)
- **4× LK MF9010 V2 (17T)** wheel hub motors (speed control with torque limit)
- **1× IMU** (`sensor_msgs/msg/Imu`)

Could be controlled by receiving `chassis_controllers/msg/RoboxChassisControl` message.

When using this controller, set the 8 DM motors to MIT control mode and the 4 LK wheel motors to speed control with torque limit mode in the EtherCAT (`EcatV2_Master`) configuration.

> **Safety Note:** `enable_motors` defaults to `false`. During initial bringup and zero-offset calibration, keep `enable_motors: false` (read-only feedback, publishes `enable=0` disable packets) until all 12 motor polarities, zero offsets, and IMU orientation are verified.

## ROS2 Parameters

|         Parameter         |     type     |                       default                       |                             description                             |
|:-------------------------:|:------------:|:---------------------------------------------------:|:-------------------------------------------------------------------:|
|       `robot_model`       |    string    |                    `"robox_v2"`                     |           Robot geometry preset (`"robox_v2"` / `"legacy"`)         |
|      `enable_motors`      |     bool     |                       `false`                       |      Master motor output switch (`false` = publish `enable=0`)      |
|    `use_ecc_torque_ff`    |     bool     |                       `true`                        |       Enable GRF feedforward torque on eccentric DM actuators       |
|  `motor_watchdog_timeout` |    double    |                       `0.25`                        |  Motor offline/fault or feedback timeout (s); disables all 12 motors if triggered |
|       `cmd_timeout`       |    double    |                       `0.5`                         |    Command timeout (s); zeros velocity and `spin_mode` if exceeded  |
|       `steer_signs`       |  double[4]   |               `[1.0, 1.0, 1.0, 1.0]`                |        Steering polarity signs `[LF, RF, LB, RB]`                   |
|      `steer_offsets`      |  double[4]   |               `[0.0, 0.0, 0.0, 0.0]`                |        Steering zero offsets in rad `[LF, RF, LB, RB]`              |
|        `ecc_signs`        |  double[4]   |              `[1.0, -1.0, 1.0, -1.0]`               |        Eccentric arm polarity signs `[LF, RF, LB, RB]`              |
|       `ecc_offsets`       |  double[4]   |           `[pi, pi, pi, pi]` (`robox_v2`)           |        Eccentric arm zero offsets in rad `[LF, RF, LB, RB]`         |
|       `wheel_signs`       |  double[4]   |             `[-1.0, 1.0, -1.0, 1.0]`                |        Wheel drive polarity signs `[LF, RF, LB, RB]`                |
|       `dm_steer_kp`       |    double    |                       `360.0`                       |                 Steering DM MIT position gain `kp`                  |
|       `dm_steer_kd`       |    double    |                        `5.0`                        |                 Steering DM MIT damping gain `kd`                   |
|      `dm_steer_pmax`      |    double    |                        `pi`                         |        Steering DM `PMax` configured in DMTools (rad)               |
|        `dm_ecc_kp`        |    double    |                       `125.0`                       |                Eccentric DM MIT position gain `kp`                  |
|        `dm_ecc_kd`        |    double    |                        `4.8`                        |                Eccentric DM MIT damping gain `kd`                   |
|       `dm_ecc_pmax`       |    double    |                        `pi`                         |        Eccentric DM `PMax` configured in DMTools (rad)              |
|        `wheel_kt`         |    double    |                       `0.16`                        |                  LK wheel torque constant (Nm/A)                    |
|   `wheel_max_torque_nm`   |    double    |                        `2.2`                        |                    LK wheel torque limit (Nm)                       |
|  `wheel_mf_max_current_a` |    double    |                       `16.5`                        |         LK MF full-scale current corresponding to 2048 LSB (A)      |
|  `chassis_cmd_read_topic` |    string    |              `"robox_chassis_command"`              |             Command topic of `RoboxChassisControl`                  |
|     `imu_read_topic`      |    string    |                       `"imu"`                       |                 IMU feedback topic (`sensor_msgs/Imu`)              |
| `steer_motor_read_topics` |  string[4]   | `["steer_lf_read", ..., "steer_rb_read"]`           |          Steering DM feedback topics `[LF, RF, LB, RB]`             |
| `steer_motor_write_topics`|  string[4]   | `["steer_lf_write", ..., "steer_rb_write"]`         |           Steering DM command topics `[LF, RF, LB, RB]`             |
|  `ecc_motor_read_topics`  |  string[4]   | `["ecc_lf_read", ..., "ecc_rb_read"]`               |          Eccentric DM feedback topics `[LF, RF, LB, RB]`            |
| `ecc_motor_write_topics`  |  string[4]   | `["ecc_lf_write", ..., "ecc_rb_write"]`             |           Eccentric DM command topics `[LF, RF, LB, RB]`            |
| `wheel_motor_read_topics` |  string[4]   | `["wheel_lf_read", ..., "wheel_rb_read"]`           |           Wheel LK feedback topics `[LF, RF, LB, RB]`               |
| `wheel_motor_write_topics`|  string[4]   | `["wheel_lf_write", ..., "wheel_rb_write"]`         |            Wheel LK command topics `[LF, RF, LB, RB]`               |
|  `require_imu_for_homing` |     bool     |                       `true`                        |       Wait for valid IMU data & bias calibration during homing      |
|    `imu_mount_rpy_deg`    |  double[3]   |                  `[0.0, 0.0, 0.0]`                  |        IMU-to-body FLU rotation `[roll, pitch, yaw]` (deg)          |
|      `imu_lever_arm`      |  double[3]   |                  `[0.0, 0.0, 0.0]`                  |         IMU offset relative to chassis center `[dx, dy, dz]` (m)    |
|  `default_payload_mass`   |    double    |                        `0.0`                        |                 Initial extra payload mass (kg)                     |
|  `default_posture_mode`   |     int      |                         `0`                         |    Initial posture (`0=OUTWARD, 1=INWARD, 2=SPIN, 3=STRAFE_OUTWARD)`|
|   `default_climb_mode`    |     int      |                         `1`                         |     Initial climb mode (`0=DISABLED, 1=PAPER_NORMAL, 2=STAIR_200MM`)|
|   `default_curb_height`   |    double    |                       `0.075`                       |            Initial obstacle curb height baseline (m)                |

## Control Message Def (`chassis_controllers/msg/RoboxChassisControl`)

```bash
float32 x_speed             # unit: m/s (FLU +X forward)
float32 y_speed             # unit: m/s (FLU +Y left)
float32 yaw_speed           # unit: rad/s (CCW positive)

float32 body_height         # unit: m (< 0.08 keeps current/default height)
float32 body_roll           # unit: rad
float32 body_pitch          # unit: rad

uint8 emergency_stop        # 0/1
int8 spin_mode              # 0=off, 1=CCW, -1=CW (autonomous SpinDrive)
uint8 posture_mode          # 0=OUTWARD, 1=INWARD, 2=SPIN, 3=STRAFE_OUTWARD
uint8 climb_mode            # 0=DISABLED, 1=PAPER_NORMAL, 2=STAIR_200MM

float32 target_curb_height  # unit: m (<= 0 keeps current; > 0 clamped to [0.04, 0.22])
float32 payload_mass        # unit: kg (< 0 keeps current/default; >= 0 updates mass)
```
