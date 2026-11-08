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

#### 常用运行参数：

- **场景选项**（`--arena`）：
  ```bash
  python3 mobed_sim_native.py --arena playground
  python3 mobed_sim_native.py --arena flat
  ```
- **模式选项**（`--climb`）：
  ```bash
  python3 mobed_sim_native.py --climb normal   # 默认
  python3 mobed_sim_native.py --climb stair
  python3 mobed_sim_native.py --climb off
  ```
- **初始位置/状态**（`--keyframe`）：
  ```bash
  python3 mobed_sim_native.py --keyframe home_blue
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
| `Space` / `X` | 减速 / 停止 |
| `C` | 模式循环切换 |
| `1` / `2` / `0` | 直接指定模式（1 / 2 / 0） |
| `H` | 开关碰撞层线框显示（Group 3） |
| `L` | 姿态复位 |
| 鼠标左键拖拽 | 视角旋转 |
| 鼠标右键拖拽 | 视角平移 |
| 鼠标滚轮 | 视角缩放 |

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
