# RoboX Hub Controller

Upper-level hub controller node for RoboX. Decodes hardware SBUS RC receiver frames (`custom_msgs/msg/ReadSBUSRC`), applies asymmetric slew-rate and centrifugal rollover smoothing (`TeleopSmoother`), maps 3-position switches (`emergency_stop`, `climb_mode`, `spin_mode`, `posture_mode`), and publishes unified `chassis_controllers/msg/RoboxChassisControl` commands to `dm_lk_robox_chassis_controller`.

When SBUS RC is disabled or has not been received, it forwards external `chassis_controllers/msg/RoboxChassisControl` commands received on `external_cmd_read_topic` with timeout protection.

## ROS2 Parameters

|        Parameter        |  type  |         default         |                             description                              |
|:-----------------------:|:------:|:-----------------------:|:--------------------------------------------------------------------:|
|     `sbus_enabled`      |  bool  |         `true`          |               Enable hardware SBUS RC input decoding                 |
|    `sbus_read_topic`    | string |       `"sbus_rc"`       |         SBUS RC input topic (`custom_msgs/msg/ReadSBUSRC`)           |
|`external_cmd_read_topic`| string |`"external_robox_command"`|     External fallback `RoboxChassisControl` command input topic     |
|`chassis_cmd_write_topic`| string |`"robox_chassis_command"`|   Output `RoboxChassisControl` topic consumed by chassis controller  |
|  `sbus_failsafe_estop`  |  bool  |         `true`          |     Assert `emergency_stop=1` when SBUS goes offline or failsafe     |
|     `sbus_timeout`      | double |          `0.5`          |          SBUS RC watchdog timeout before braking/failsafe (s)        |
| `external_cmd_timeout`  | double |          `0.5`          |     External command timeout before zeroing velocity/spin (s)        |
|      `sbus_ch_min`      | double |         `172.0`         |                     Raw minimum SBUS channel value                   |
|      `sbus_ch_mid`      | double |         `992.0`         |                     Raw neutral SBUS channel value                   |
|      `sbus_ch_max`      | double |        `1811.0`         |                     Raw maximum SBUS channel value                   |
|     `sbus_deadband`     | double |         `0.05`          |                  Normalized stick deadband `[0, 1]`                  |
| `sbus_ch_vx`..`posture` |  int   | `1,0,3,2,4,5,6,7`       |   Zero-based SBUS channel indices (`-1` disables channel mapping)    |
|`sbus_inv_vx`..`estop`   |  bool  |         `false`         |                      Channel inversion flags                         |
|    `sbus_height_min`    | double |         `0.084`         |                 Minimum body height from height dial (m)             |
|    `sbus_height_max`    | double |         `0.22`          |                 Maximum body height from height dial (m)             |
|  `default_body_height`  | double |         `0.175`         |                   Default body height target (m)                     |
| `default_posture_mode`  |  int   |           `0`           |             Default posture (`0=OUTWARD, 1=INWARD, 3=STRAFE`)       |
|  `default_climb_mode`   |  int   |           `1`           |            Default climb mode (`0=DISABLED, 1=NORMAL, 2=STAIR`)      |
|  `default_curb_height`  | double |         `0.075`         |                Default curb height baseline (m)                      |
| `default_payload_mass`  | double |          `0.0`          |                  Default payload mass (kg)                           |
|    `teleop_max_vx`      | double |          `1.2`          |               Maximum smoothed forward/reverse speed (m/s)           |
|    `teleop_max_vy`      | double |          `1.0`          |                 Maximum smoothed lateral speed (m/s)                 |
|    `teleop_max_wz`      | double |          `2.0`          |                Maximum smoothed yaw angular rate (rad/s)             |
| `teleop_accel_lin_fwd`  | double |          `2.6`          |              Longitudinal acceleration limit (m/s^2)                 |
|`teleop_accel_lin_brake` | double |          `2.8`          |              Longitudinal deceleration limit (m/s^2)                 |
| `teleop_accel_strafe`   | double |          `2.2`          |                 Lateral acceleration limit (m/s^2)                   |
| `teleop_accel_angular`  | double |         `12.0`          |                 Angular acceleration limit (rad/s^2)                 |
|`teleop_max_lateral_acc` | double |          `3.0`          |            Centrifugal envelope `|vx * wz|` ceiling (m/s^2)         |
