# RoboX-AIM / MobED 12-DOF 全地形全向移动底盘仿真与控制系统

<div align="center">

[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue.svg?logo=python&logoColor=white)](https://www.python.org/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C.svg?logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![MuJoCo 3.11+](https://img.shields.io/badge/MuJoCo-3.11%2B-orange.svg)](https://mujoco.org/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Platform macOS | Linux](https://img.shields.io/badge/Platform-macOS%20%7C%20Linux-green.svg)](https://github.com/)

**面向现代移动机器人（Hyundai MobED / ROBOX V2）的超高性能 120 FPS 原生 MuJoCo 动力学仿真、高鲁棒运动学/动力学控制算法库与 ROS 2 架构**

[技术特性](#-核心特性与技术亮点) • [快速上手](#-快速上手指南) • [控制键位](#-键盘遥控与交互操作) • [仿真场景](#-仿真场景体系-models_standard) • [目录结构](#-项目架构与目录结构) • [ROS 2 支持](#-ros-2-工作空间与部署)

</div>

---

## 📖 项目简介

**RoboX-AIM (MobED / ROBOX V2)** 是一套针对 **12 自由度独立转向、偏心姿态调整与全驱轮组底盘** 的完整科研与竞技机器人解决方案。

本项目参考现代智能底盘架构与前沿学术理论（IEEE RA-L 2026），完整实现了：
1. **12 自由度全解耦驱动机构**：
   - **4 个独立转向机构**（4WS / Swerve Drive，搭载达妙 DM-J4310P 减速比 10:1）
   - **4 个独立偏心臂姿态机构**（Active Eccentric Posture Mechanism，搭载达妙 DM-J4340P 减速比 40:1）
   - **4 个独立轮毂直驱轮**（4WD，搭载 MS9010 v3 直驱电机 1:1）
2. **高保真 120 FPS 原生物理仿真引擎**：基于 MuJoCo 与原生 GLFW 硬件加速渲染，彻底杜绝输入冲突与渲染延迟。
3. **C++ 原生算法内核与跨平台动态链接库**：通过零拷贝内存直连（`ctypes`），提供小于 0.1ms 的极速闭环控制；同时提供完整 ROS 2 Humble 工作空间节点与接口。
4. **RoboMaster 2027 赛场与复杂地形基准库**：精确还原 RoboMaster 机甲大师高校锦标赛 2027 赛场（RMUC 2027）地形与 5cm/10cm/20cm 台阶、斜坡、非平整路面。

---

## ✨ 核心特性与技术亮点

### 1. 原生 120 FPS 纯净视口仿真器 (`mobed_sim_native.py`)
- **硬件级多键并发检测**：基于 `glfw.get_key()` 轮询，完美支持 `W + A` 或 `W + D` 复合键同时按下，实现带侧向倾斜的摩托车式敏捷过弯（Lean Banking）。
- **零按键冲突与纯净 HUD**：彻底绕过 MuJoCo 自带的 GUI 快捷键陷阱，`Space` 表现为平滑刹车而不冻结物理引擎，`A/E/J` 键不弹出干扰面板。
- **12 电机功耗与发热实时测算**：集成达妙/大疆电机的铜损与反电动势电学模型（$P_{\text{elec}} = I \cdot (K_e |\omega_m| + I R)$），画面实时叠加功耗浮动看板。
- **三维视角平滑追踪**：支持左键围绕机器人轨道旋转（Orbit）、右键平移（Pan）、滚轮缩放（Zoom）以及底盘自动追踪相机。

### 2. 高鲁棒遥控规划器 (`teleop_commander.py`)
- **非对称二阶斜坡滤波器（Asymmetric 2nd-Order Slew-Rate Filter）**：加速响应充满推背感且轮胎不打滑，制动减速迅捷停稳且杜绝冲击俯冲。
- **连续速度场动态混合（Continuous Velocity Blending）**：
  $$\text{weight}_{\text{steer}} = 3\left(\frac{|v_x|}{v_{\text{th}}}\right)^2 - 2\left(\frac{|v_x|}{v_{\text{th}}}\right)^3$$
  彻底消除行进间转向与原地横移（Crab Strafe）切换时的舵轮 90° 暴震现象。
- **向心加速度安全包线限制**：实时监控过弯横向加速度 $|v_x \cdot \omega_z| \le 3.0\text{ m/s}^2$，动态限速，杜绝高速急弯翻车。

### 3. 先进 C++ 控制算法内核 (`mobed_ws/src/robot_control`)
- **运动学模型 (Kinematics)**：全向 4 轮独立转向 ICR（Instantaneous Center of Rotation）连续几何解算与偏心机构高度/姿态正逆运动学。
- **动力学与平衡控制 (Dynamics & Balance)**：单刚体动力学（SRBD）模型结合地面反力二次规划优化（GRF QP Optimizer），实现自适应车体水平姿态调平与外八字宽轮距稳定抗倾翻。
- **越障状态机 (Obstacle Climbing FSM)**：自主实现前轮高抬翻越（Front Vault 180°）、跨骑调平（Straddle Leveling）、底盘拖底滑行保护（Rear Belly Rest）与后轮翻越超调重置（Rear 45° Overshoot Traction Sequence），稳定越过高达 200mm 的垂直台阶。
- **状态估计 (State Estimation)**：结合 IMU 与 12 轴编码器的扩展卡尔曼滤波（EKF）、地形坡度估计器以及越障接触冲击探测器。

### 4. 严谨标定的标准仿真场景 (`models_standard`)
- **RMUC 2027 赛场**：经多重射线与三角面片分析重构，彻底修复 CAD 导入常见的气墙与地面破面问题（通过 6/6 全项几何与动力学验证）。
- **多地形障碍台架 (Playground)**：包含 5cm、10cm、20cm 路沿台阶、坡度测试区与平整广场。
- **ROBOX V2 标准模型**：30.00 kg 整机质量参数精确标定，具有真实的碰撞外壳与底盘防护滑块几何。

---

## 📁 项目架构与目录结构

```text
RoboX-AIM/
├── mobed_sim_native.py            # [核心] 原生 120 FPS GLFW + MuJoCo 仿真执行入口
├── teleop_commander.py            # [核心] 高鲁棒滤波与连续转向遥控规划器
├── requirements.txt               # Python 依赖清单
├── Dockerfile                     # ROS 2 Humble 完整开发容器配置
│
├── build_native/                  # 原生 C++ 动态链接库输出目录
│   └── libmobed_control.dylib     # C++ 算法编译生成的高性能动态库
│
├── models_standard/               # [标定模型库] 标准仿真场景与模型资产
│   ├── robox_v2/                  # ROBOX V2 机器人标准平地模型 (MJCF & OBJ)
│   ├── rmuc2027/                  # RoboMaster 2027 赛场 (Lite 120 FPS / HD 版)
│   └── playground/                # 综合障碍与越障测试台架 (5cm/10cm/20cm)
│
├── mobed_ws/                      # [ROS 2 工作空间] ROS 2 Humble 核心代码
│   └── src/
│       ├── robot_interfaces/      # 自定义 ROS 2 消息 (MobEDCommand.msg)
│       ├── robot_control/         # C++ 核心控制算法实现与 ROS 2 节点
│       │   ├── include/robot_control/  # 动力学、运动学、估计算法头文件
│       │   └── src/                    # 算法源文件与 mobed_control_node
│       └── robot_bringup/         # 启动文件与系统配置 (mobed_core.launch.py)
│
├── scripts/                       # 辅助工具与构建脚本
│   ├── build_native_dylib.sh      # 一键编译 C++ 控制算法动态库
│   ├── start-dev.sh               # 快速启动 Docker 开发环境
│   ├── verify_arena_collisions.py # 赛场碰撞连续性与几何验证套件 (6项测试)
│   ├── reconstruct_rmuc2027_collisions.py # 赛场碰撞体解析重构工具
│   ├── calibrate_arena_collisions.py      # 赛场 STL 几何校准脚本
│   └── decimate_arena_meshes.py          # 赛场网格轻量化体素抽稀工具
│
└── sources/                       # 原始 CAD、URDF 与 STP 参考资料
```

---

## 🚀 快速上手指南

### 1. 环境准备

推荐运行环境：
- **操作系统**：macOS (Apple Silicon / Intel) 或 Linux (Ubuntu 20.04 / 22.04)
- **Python**：Python 3.10+ (推荐 3.10 ~ 3.14)
- **编译工具**：`clang++` (macOS 预装) 或 `g++` (支持 C++17)，以及 `Eigen3`

#### 安装依赖库 (macOS)：
```bash
# 安装基础编译依赖与 Eigen
brew install cmake eigen glfw
```

#### 安装依赖库 (Ubuntu / Debian)：
```bash
sudo apt update && sudo apt install -y build-essential cmake libglfw3-dev libeigen3-dev
```

### 2. 配置 Python 虚拟环境并安装依赖

```bash
# 克隆或进入项目目录
cd RoboX-AIM

# 创建虚拟环境
python3 -m venv .venv
source .venv/bin/activate

# 安装 Python 运行依赖
pip install -r requirements.txt
```

### 3. 一键编译 C++ 算法动态库

运行脚本快速构建 `libmobed_control.dylib` (或 Linux 对应的 `.so`)：

```bash
./scripts/build_native_dylib.sh
```
*编译完成后将在 `build_native/` 目录下生成 `libmobed_control.dylib`。*

### 4. 启动原生 120 FPS 仿真

#### 运行默认场景（RoboMaster 2027 赛场 Lite 版）：
```bash
python3 mobed_sim_native.py
```

#### 运行多障碍综合测试场（Playground）：
```bash
python3 mobed_sim_native.py --arena playground
```

#### 运行平地基准标定场景：
```bash
python3 mobed_sim_native.py --arena flat
```

#### 常用启动参数说明：
```bash
python3 mobed_sim_native.py \
  --arena rmuc2027 \        # 可选: rmuc2027 (默认), rmuc2027_hd, playground, flat/robox
  --climb normal \          # 越障模式: normal (模式1: 论文标准自适应), stair (模式2: 200mm 台阶专用), off
  --keyframe home_blue      # 预设出生点: home_red, home_blue, test_center, test_platform_top 等
```

---

## 🎮 键盘遥控与交互操作

在 3D 仿真视口中直接点击并激活窗口，支持完整硬件级同时多键并发输入：

| 按键类别 | 按键 | 功能说明 | 性能指标 |
| :--- | :--- | :--- | :--- |
| **纵向行驶** | `W` / `S` | 前进 / 后退 | 额定巡航速度 1.0 m/s |
| **敏捷转弯** | `W + A` / `W + D` | 前进转弯 + 动态摩托车式压弯 (Lean) | 动态向心倾斜角补偿 |
| **原地横移** | `A` / `D` (静止时) | 纯侧向平移（全向蟹行） | 外八字宽轮距撑开，0.8 m/s |
| **原地自转** | `Q` / `E` | 逆时针 / 顺时针原地自转 | 切向最小阻力解算，1.8 rad/s |
| **平滑制动** | `Space` / `X` | 柔和渐进式制动（松开按键滑行停止） | 减速度 4.5 m/s²，防颠簸 |
| **碰撞线框** | `H` | 开关碰撞体几何层（Group 3 Collision） | 观察碰撞几何与接触判定 |
| **最低姿态** | `L` | 车身降至最低离地间隙（偏心臂向上指天） | 偏心姿态重置基线 |
| **电机功耗测试** | `T` / `G` | 偏心臂向外 / 向内 180° 转动功耗测试 | 测试 DM-J4340P 动态峰值功耗 |
| **越障模式切换** | `C` | 循环切换模式（模式 1: 论文标准 ↔ 模式 2: 200mm 台阶） | FSM 状态机切换 |
| **越障模式选择** | `1` / `2` / `0` | 直接选择: `[1]` 论文标准 \| `[2]` 200mm 台阶 \| `[0]` 关闭越障 | 越障控制状态切换 |
| **视角控制** | 鼠标左键拖拽 | 围绕底盘中心 360° 旋转观察 | 实时底盘追踪 |
| **视角控制** | 鼠标右键拖拽 | 视口平移（Pan） | 自由定位观察点 |
| **视角控制** | 鼠标滚轮 | 视口缩放（Zoom In / Out） | 细节观察 |

---

## 🏟️ 仿真场景体系 (`models_standard`)

| 场景目录 | 核心 XML 模型文件 | 场景说明与用途 | 关键特性 |
| :--- | :--- | :--- | :--- |
| **`rmuc2027/`** | `robot_rmuc2027_lite.xml` | **RoboMaster 2027 高校锦标赛竞技场 (推荐)** | 120 FPS 满帧流程度；全几何无缝碰撞体；包含环形路、中央天桥、飞坡与飞镖站 |
| | `robot_rmuc2027.xml` | **RMUC 2027 完整高精度光影版** | 包含完整高精度 CAD 网格与高保真材质光影 |
| **`playground/`** | `robot_playground.xml` | **全地形障碍与阶梯测试场** | 包含 5cm、10cm、20cm 连续路沿、台阶、坡道与断差测试 |
| **`robox_v2/`** | `robox_v2.xml` | **ROBOX V2 标准平地模型** | 30.00 kg 实测标定模型，底层滑块阻抗与 12 轴执行器参数标定 |

### 赛场几何与碰撞自动化验证
本项目自带自动化无损碰撞验证体系，可验证赛场连续通过性、地基无死角拼接与无幽灵碰撞墙：
```bash
python3 scripts/verify_arena_collisions.py
```
*执行 6 项综合测试（高架桥连续性、环形路无重叠、门洞中空性、资源岛开放性、飞镖站无缝过渡、多出生点动力学静止稳定性），全部绿标通过。*

---

## 🤖 ROS 2 工作空间与部署

本项目在 `mobed_ws` 中内置完整的 ROS 2 Humble 工程结构，与原生 C++ 核心保持 100% 算法代码共享。

### 1. 本地 ROS 2 构建 (Ubuntu 22.04 + ROS 2 Humble)
```bash
cd mobed_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 启动核心控制节点与硬件/仿真桥接
ros2 launch robot_bringup mobed_core.launch.py
```

### 2. ROS 2 话题与接口
- `/mobed/command` (`robot_interfaces/msg/MobEDCommand`)：接收包含速度指令 `Twist` 与目标机体高度/姿态角（`body_height`, `body_roll`, `body_pitch`）的综合控制包。
- `/joint_states` (`sensor_msgs/msg/JointState`)：底盘 12 关节角度、角速度与实测力矩反馈。
- `/imu` (`sensor_msgs/msg/Imu`)：高频六轴 IMU 加速度、角速度与四元数。
- `/mobed/posture_mode` (`std_msgs/msg/Int32`)：动态切换姿态构型（`0`: OUTWARD, `1`: INWARD, `2`: SPIN, `3`: STRAFE_OUTWARD）。
- `/mobed/payload_mass` (`std_msgs/msg/Float64`)：动态设定搭载负载质量（0 ~ 56.0 kg），控制器自动修正 SRBD 惯量矩阵与重心前馈。

### 3. Docker 容器化环境运行
若在 macOS 或未配置 ROS 2 的机器上开发 ROS 节点，可使用项目提供的 Dockerfile：
```bash
# 构建镜像
docker build -t ros2-dev .

# 启动开发容器并挂载当前工程
./scripts/start-dev.sh
```

---

## ⚙️ 机器人核心物理规格表 (ROBOX V2)

| 参数项 | 数值 | 备注 |
| :--- | :--- | :--- |
| **底盘总质量 (Total Mass)** | **30.00 kg** | 车体 16.6 kg + 4x转向 6.1 kg + 4x偏心 4.2 kg + 4x车轮 3.0 kg |
| **最大有效载荷 (Payload)** | **56.00 kg** | 算法支持 0 ~ 56kg 动态在线自适应质心与惯量重计算 |
| **底盘半长 / 半宽** | **0.2226 m / 0.2226 m** | 中心至转向回转轴标准 CAD 距离（对称正方形布局） |
| **偏心摇臂长度 ($l_{\text{ecc}}$)** | **0.0720 m** | 偏心轴到车轮轴中心距 |
| **车轮半径 ($r_{\text{wheel}}$)** | **0.1000 m** | 聚氨酯高抓地力橡胶轮胎轮毂 |
| **垂直离地间隙范围** | **0.055 m ~ 0.228 m** | 偏心机构在上下止点间的连续调节高度域 |
| **最大巡航速度 / 自转角速度** | **1.0 m/s / 1.8 rad/s** | 软件安全限幅下，支持超调响应 |
| **越障能力** | **高达 200 mm 垂直阶梯** | 经模式 2 (STAIR_200MM) 自动化翻越状态机通过 |
