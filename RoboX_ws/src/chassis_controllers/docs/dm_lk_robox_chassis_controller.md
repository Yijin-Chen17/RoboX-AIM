# DM/LK RoboX Wheel-Leg Chassis Controller

This node is for the 12-motor RoboX / MobED wheel-leg swerve chassis equipped with:
- **4× DM-J4310P-2EC** steering motors (`Position Control With Speed Limit` mode)
- **4× DM-J4340P-2EC V1.1** eccentric arm motors (`Position Control With Speed Limit` mode)
- **4× LK MF9010 V2 (17T)** wheel hub motors (`Broadcast Current` mode, 4-in-1 on one CAN bus with IDs 1..4)
- **1× IMU** (`sensor_msgs/msg/Imu`)

Could be controlled by receiving `chassis_controllers/msg/RoboxChassisControl` message.

When using this controller, set the 8 DM motors to **Position Control With Speed Limit** mode (`WriteDmMotorPositionControlWithSpeedLimit`) and the 4 LK wheel motors to a single **Broadcast Current** task (`ReadLkMotorMulti` / `WriteLkMotorBroadcastCurrentControl`) in the EtherCAT (`EcatV2_Master`) configuration.

## Emergency Stop Behavior

When `emergency_stop` (`1`) is triggered manually in `RoboxChassisControl` (or if the solver outputs non-finite values):
1. **4× DM Steering & 4× DM Eccentric Motors**: Immediately receive `enable = 0, position = 0.0, speed = 0.0` to remove stiffness and prevent sudden leg/steering snap.
2. **4× LK Wheel Hub Motors**: Actively brake using proportional current (`clamp(-wheel_speed_kp * wheel_vel, -wheel_max_torque_nm, wheel_max_torque_nm)`) converted to broadcast current commands while any wheel speed exceeds `0.5 rad/s`; once all 4 wheels stop (`<= 0.5 rad/s`), publish `motor1..4_cmd = 0`.
3. **Anti-Jerk Recovery (`1 -> 0`)**: While emergency stop is active, the solver continuously resets its steering/balance states so that releasing emergency stop tracks smoothly from the current measured joint angles (zero steering jump, 3.0s cubic soft-start on eccentric arms), further bounded by `dm_steer_speed_limit` and `dm_ecc_speed_limit`.

## ROS2 Parameters

|        Parameter         |   type    |                       default                       |                             description                             |
|:------------------------:|:---------:|:---------------------------------------------------:|:-------------------------------------------------------------------:|
|      `robot_model`       |  string   |                    `"robox_v2"`                     |           Robot geometry preset (`"robox_v2"` / `"legacy"`)         |
|      `steer_signs`       | double[4] |               `[1.0, 1.0, 1.0, 1.0]`                |        Steering polarity signs `[LF, RF, LB, RB]`                   |
|     `steer_offsets`      | double[4] |               `[0.0, 0.0, 0.0, 0.0]`                |        Steering zero offsets in rad `[LF, RF, LB, RB]`              |
|       `ecc_signs`        | double[4] |              `[1.0, -1.0, 1.0, -1.0]`               |        Eccentric arm polarity signs `[LF, RF, LB, RB]`              |
|      `ecc_offsets`       | double[4] |           `[pi, pi, pi, pi]` (`robox_v2`)           |        Eccentric arm zero offsets in rad `[LF, RF, LB, RB]`         |
|      `wheel_signs`       | double[4] |             `[-1.0, 1.0, -1.0, 1.0]`                |        Wheel drive polarity signs `[LF, RF, LB, RB]`                |
|  `dm_steer_speed_limit`  |  double   |                       `12.0`                        |      Steering DM position-mode speed limit (rad/s)                  |
|   `dm_ecc_speed_limit`   |  double   |                        `3.5`                        |     Eccentric arm DM position-mode speed limit (rad/s)              |
|     `wheel_speed_kp`     |  double   |                       `15.0`                        |       Wheel speed PI proportional gain `Nm / (rad/s)`               |
|     `wheel_speed_ki`     |  double   |                        `0.5`                        |         Wheel speed PI integral gain `Nm / rad`                     |
|        `wheel_kt`        |  double   |                       `0.16`                        |                 LK wheel torque constant (Nm/A)                     |
|  `wheel_max_torque_nm`   |  double   |                        `2.2`                        |                  LK wheel torque limit (Nm)                         |
| `wheel_mf_max_current_a` |  double   |                       `16.5`                        |    LK MF full-scale current mapped to +/-2000 broadcast cmd (A)     |
| `chassis_cmd_read_topic` |  string   |              `"robox_chassis_command"`              |             Command topic of `RoboxChassisControl`                  |
|     `imu_read_topic`     |  string   |                       `"imu"`                       |                 IMU feedback topic (`sensor_msgs/Imu`)              |
|`steer_motor_read_topics` | string[4] | `["steer_lf_read", ..., "steer_rb_read"]`           |          Steering DM feedback topics `[LF, RF, LB, RB]`             |
|`steer_motor_write_topics`| string[4] | `["steer_lf_write", ..., "steer_rb_write"]`         |           Steering DM command topics `[LF, RF, LB, RB]`             |
| `ecc_motor_read_topics`  | string[4] | `["ecc_lf_read", ..., "ecc_rb_read"]`               |          Eccentric DM feedback topics `[LF, RF, LB, RB]`            |
| `ecc_motor_write_topics` | string[4] | `["ecc_lf_write", ..., "ecc_rb_write"]`             |           Eccentric DM command topics `[LF, RF, LB, RB]`            |
| `wheel_motor_read_topic` |  string   |                `"wheel_motor_read"`                 |      4-in-1 LK broadcast feedback topic (`ReadLkMotorMulti`)        |
|`wheel_motor_write_topic` |  string   |                `"wheel_motor_write"`                | 4-in-1 LK broadcast command topic (`WriteLkMotorBroadcastCurrentControl`) |
| `require_imu_for_homing` |   bool    |                       `true`                        |       Wait for valid IMU data & bias calibration during homing      |
|   `imu_mount_rpy_deg`    | double[3] |                  `[0.0, 0.0, 0.0]`                  |        IMU-to-body FLU rotation `[roll, pitch, yaw]` (deg)          |
|     `imu_lever_arm`      | double[3] |                  `[0.0, 0.0, 0.0]`                  |         IMU offset relative to chassis center `[dx, dy, dz]` (m)    |
|  `default_payload_mass`  |  double   |                        `0.0`                        |                 Initial extra payload mass (kg)                     |
|  `default_posture_mode`  |    int    |                         `0`                         |    Initial posture (`0=OUTWARD, 1=INWARD, 2=SPIN, 3=STRAFE_OUTWARD)`|
|   `default_climb_mode`   |    int    |                         `1`                         |     Initial climb mode (`0=DISABLED, 1=PAPER_NORMAL, 2=STAIR_200MM`)|
|  `default_curb_height`   |  double   |                       `0.075`                       |            Initial obstacle curb height baseline (m)                |

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
