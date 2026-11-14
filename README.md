# Environment Setup & Usage Guide

---

## 🛠️ 环境配置

### 1. 系统依赖

- **操作系统**：macOS 或 Linux (Ubuntu 20.04 / 22.04+)
- **Python**：Python 3.10+
- **编译工具**：支持 C++17 的编译器（`clang++` 或 `g++`）、`cmake`、`Eigen3`、`GLFW`

#### macOS 安装基础依赖：
```bash
brew install cmake eigen glfw
```

#### Ubuntu / Debian 安装基础依赖：
```bash
sudo apt update && sudo apt install -y build-essential cmake libglfw3-dev libeigen3-dev
```

### 2. Python 虚拟环境与依赖安装

```bash
# 创建并激活虚拟环境
python3 -m venv .venv
source .venv/bin/activate

# 安装依赖
pip install -r requirements.txt
```

### 3. 编译动态库

在项目根目录下执行编译脚本，生成所需的动态库：

```bash
./scripts/build_native_dylib.sh
```

---

## 🚀 使用方法

### 1. 运行程序

启动主程序：

```bash
python3 mobed_sim_native.py
```

转向和偏心臂的 MIT 指令定义及仿真力矩计算在
`mobed_ws/src/robot_control/include/robot_control/controllers/mit_servo.hpp`。
更新后先运行 `./scripts/build_native_dylib.sh`。原生仿真每个物理步调用 C++ 计算力矩。

接实车时，先把控制器的舵角和偏心臂目标角转换到各电机的输出轴坐标，校准方向和零位。
用对应的 `steeringMitCommand` 或 `eccentricMitCommand` 生成五个 MIT 指令字段，再按电机配置编码成 CAN 帧。
实车驱动器根据编码器反馈闭合 MIT 环。头文件里的增益是仿真初值，实车需分别带载标定。

#### 常用运行参数：

- **场景选项**（`--arena`）：
  ```bash
  python3 mobed_sim_native.py --arena rmuc2027          # 默认：RMUC 2027 完整高精度场地 + ROBOX V2
  python3 mobed_sim_native.py --arena rmuc2027_physics  # RMUC 2027 纯碰撞加速模式
  python3 mobed_sim_native.py --arena playground        # 障碍台阶测试台
  python3 mobed_sim_native.py --arena flat              # 平地
  ```
- **模式选项**（`--climb`）：
  ```bash
  python3 mobed_sim_native.py --climb normal   # 默认：0–80 mm 小台阶
  python3 mobed_sim_native.py --climb stair    # 独立的 200 mm 攀爬
  python3 mobed_sim_native.py --climb off      # 关闭攀爬
  ```
  仿真中按 `1` 进入 normal，按 `2` 进入 200 mm，按 `0` 关闭攀爬。切换会清除上一模式的攀爬进度。测试小台阶时，在 normal 下用外展姿态直行接近台阶。
  原生仿真在启动 normal 攀爬前会测量前方碰撞地形的高度，高于 80 mm 时保持待机。
- **初始位置/状态**（`--keyframe`）：
  ```bash
  python3 mobed_sim_native.py --keyframe home_red           # 红方基地出生点（朝向场地中心）
  python3 mobed_sim_native.py --keyframe home_blue          # 蓝方基地出生点（朝向场地中心）
  python3 mobed_sim_native.py --keyframe central_passage    # 中央隧道入口
  python3 mobed_sim_native.py --keyframe road_red           # 红方起伏路
  python3 mobed_sim_native.py --keyframe ramp_red           # 红方飞坡
  python3 mobed_sim_native.py --keyframe test_platform_top  # 中央高地平台
  ```

---

### 2. 交互控制说明

在图形窗口激活时支持以下键盘与鼠标控制：

| 按键 | 功能 |
| :--- | :--- |
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左 / 右 |
| `W + A` / `W + D` | 前进并左偏 / 前进并右偏 |
| `Q` / `E` | 原地逆时针 / 顺时针旋转 |
| `V` / `B` | 开关逆时针 / 顺时针小陀螺。启动后，方向键按进入时的车头方向平移 |
| `Space` / `X` | 减速 / 停止 |
| `C` | 模式循环切换 |
| `1` / `2` / `0` | 直接指定模式（1 / 2 / 0） |
| `H` | 开关碰撞层线框显示（Group 3） |
| `L` | 姿态复位 |
| 鼠标左键拖拽 | 视角旋转 |
| 鼠标右键拖拽 | 视角平移 |
| 鼠标滚轮 | 视角缩放 |

小陀螺适用于 ROBOX V2 的平坦场地。进入后先对舵，再逐步升至目标角速度 1.8 rad/s。松开方向键后继续自转。再次按模式键、按 `Space` / `X` 或窗口失焦会减速退出。切换旋转方向时，先退出并停稳，再按另一方向的模式键。

ROS 2 通过 `/mobed/spin`（`std_msgs/msg/Int8`）请求小陀螺。发送 `1` 为逆时针、`-1` 为顺时针、`0` 为退出。启用期间持续发送模式请求和 `/mobed/command`，两者的超时均为 0.5 秒。此时命令的线速度相对于进入模式时的车头方向。零线速度请求表示原地持续自转。

---

### 3. ROS 2 方式（可选）

如需通过 ROS 2 运行：

```bash
cd mobed_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_bringup mobed_core.launch.py
```

或使用 Docker 容器环境：

```bash
docker build -t ros2-dev .
./scripts/start-dev.sh
```
