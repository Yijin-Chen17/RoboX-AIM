# 车体速度指令 (FLU 坐标系)
float32 x_speed             # 前后线速度 (m/s)
float32 y_speed             # 左右横移线速度 (m/s)
float32 yaw_speed           # 普通转向角速度 (rad/s)

# 车体姿态指令
float32 body_height         # 目标离地高度 (m); < 0.08 表示保持当前/默认高度
float32 body_roll           # 目标横滚角 (rad)
float32 body_pitch          # 目标俯仰角 (rad)

# 安全与运动模式
uint8 emergency_stop        # 0 = 正常, 1 = 急停卸力
int8 spin_mode              # 小陀螺: 0 = 关闭, 1 = 逆时针, -1 = 顺时针
uint8 posture_mode          # 0 = OUTWARD, 1 = INWARD, 2 = SPIN, 3 = STRAFE_OUTWARD
uint8 climb_mode            # 0 = DISABLED, 1 = PAPER_NORMAL(0~80mm), 2 = STAIR_200MM

# 可选运行时参数覆盖
float32 target_curb_height  # 台阶高度基线 (m); <= 0 保持当前, > 0 时限幅 [0.04, 0.22]
float32 payload_mass        # 额外负载质量 (kg); < 0 保持当前/默认参数, >= 0 时更新
