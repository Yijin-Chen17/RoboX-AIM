# ROBOX V2 — MuJoCo 导入版

推荐打开包根目录下的 `robox_v2.xml`。也可以直接加载 `urdf/robox_v2.urdf`。
请先完整解压压缩包，保留文件夹结构；不要只复制 XML 或 URDF。

## 使用

Python 环境安装 MuJoCo 后，在解压目录运行：

```powershell
python -m pip install mujoco
python view_model.py
```

在自己的 Python 程序中加载：

```python
from pathlib import Path
import mujoco

folder = Path(r"D:\robox_v2_mujoco")  # 改为实际解压路径
model = mujoco.MjModel.from_xml_path(str(folder / "robox_v2.xml"))
# 或者：
# model = mujoco.MjModel.from_xml_path(str(folder / "urdf/robox_v2.urdf"))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
```

也可以在 MuJoCo simulate 程序中打开 `robox_v2.xml`。

## 修改内容

- 实体名称改为 `part_01` 至 `part_13`，编号与原 CAD 实体编号一致。
- 活动关节名称改为 `joint_01` 至 `joint_12`，编号与原“旋转”编号一致。
- 辅助配合坐标系改为 `mate_link_01` 等，对应固定关节改为 `mate_joint_01` 等。
- 同步更新所有父子连杆引用，原名称对照见 `name_mapping.csv`。
- 13 个 glTF 网格转换为 ASCII 文件名的 OBJ；保留顶点坐标、三角形及场景节点变换，没有缩放、居中或旋转模型。
- 网格路径改为相对路径，脱离 ROS 的 `package://` 路径解析。
- URDF 添加 MuJoCo compiler 设置：启用 visual 导入，并融合固定关节。
- 从已加载的 URDF 导出原生 MJCF `robox_v2.xml`，修正其网格相对路径。
- 保留原关节类型、位置、轴向、质量、质心与惯性参数。

运行所需的 URDF、MJCF、OBJ 文件内容及文件名均为 ASCII。此中文说明和原名称对照表不参与模型编译。

## 当前模型范围

- 原文件是固定基座，修改版保持固定基座；底盘不会自由移动。
- 原文件没有 collision、地面或 actuator；修改版保持原设置。网格仅用于显示，不会与地面产生接触。行走/行驶仿真需要另行配置这些内容。
- 12 个活动关节原本都是 continuous，没有位置限位；未推测或添加限位、控制器、力矩上限。
- 原 URDF 所有质量之和约为 **0.0123068 kg（12.31 g）**，数据原样保留。正式动力学仿真前应核对 CAD 材料、密度及导出单位。
- `name_mapping.csv` 保留了原中文名称，以便回查 CAD；它不是运行依赖。
