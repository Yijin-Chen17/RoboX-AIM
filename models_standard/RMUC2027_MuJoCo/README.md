# RMUC 2027 MuJoCo 场景

按已审核的 v3 外形生成，面向轮式机器人行驶、越坡和避障。保留真实台阶、坡度、中央通道、公路圆滑起伏及回收开槽；已删除两侧基地、前哨站、资源站／兑换站、工作台、专用底座及配套能量单元。风机、堡垒和中央机关保留粗外形。场地为静态碰撞模型，机关没有运动机构。

没有图片贴图。碰撞采用 **13 个盒体 + 1,352 个凸网格，共 1,365 个碰撞体、34,152 个三角面**。可选的 42 个纯色显示网格仅改善外观，不参与接触；训练入口会丢弃这些显示网格。没有将凹地形整体当作一个凸壳。

## 选择入口

| 文件 | 用途 |
| --- | --- |
| `robot_rmuc2027.xml` | **ROBOX V2 + 完整场地仿真**，集成 12 关节电驱与标定 Keyframe |
| `robot_rmuc2027_physics.xml` | **ROBOX V2 + 纯碰撞场地**，`discardvisual="true"`，加速物理仿真与训练 |
| `scene.xml` | 完整场地，带纯色显示外表面，可直接加载 |
| `scene_physics.xml` | 纯碰撞场地；`discardvisual="true"`，训练优先使用 |
| `field.xml` | 可被自己的机器人场景 include 的静态场地定义 |
| `rover_test.xml` | 带通用四轮测试车的演示；不是你的实际机器人 |
| `launch_robox.sh` / `.cmd` | 启动 ROBOX V2 完整仿真交互控制（macOS / Linux / Windows） |
| `launch_scene.sh` / `.cmd` | 查看 RMUC 2027 场地 |
| `launch_rover.sh` / `.cmd` | 演示测试车通过公路起伏 |
| `run_tests.cmd` | Windows 双击重新运行接触、路线及速度测试 |
| `测试报告.md` | 本机测试结果、条件及边界问题说明 |

请保留整个文件夹的相对结构；场景需要 `field.xml` 和 `assets/visual/`，单独复制入口 XML 不够。无需原始 653 MB STEP 或 CAD 软件。

## 在当前电脑查看

直接双击 `launch_scene.cmd` 或 `launch_rover.cmd`。启动文件会优先使用本场景的 `.venv`，其次使用这次转换所建的本地 Python 环境，再尝试系统 Python。

查看器左键拖动旋转，右键拖动平移，滚轮缩放；空格暂停／继续，R 重置。测试车视角会跟随车体。关闭窗口即可退出。

## 换一台电脑

推荐 Python 3.12；已验证的版本是 Python 3.12.15、MuJoCo 4.0.0。进入本文件夹运行：

```powershell
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe view_scene.py
```

Linux 下使用 `.venv/bin/python`。无窗口的接触和路线测试不需要渲染；查看器和 `--render` 需要可用的图形环境。

```powershell
# 以纯碰撞方式显示测试车通过中央通道
python view_scene.py --route central_passage --physics

# 完整复测并导出实际 MuJoCo 图片
python validate_scene.py --render

# 无窗口地运行一条演示路线
python view_scene.py --route road_red --physics --headless --duration 15
```

请在已安装依赖的 Python 环境下执行。路线名称可选 `flat`、`central_passage`、`ramp_red`、`ramp_blue`、`road_red`、`road_blue`。测试结果写入 `test_results.json`，轨迹写入 `test_trajectories.json`；复测会更新这两个文件。

## 接入自己的机器人

在自己的主 MJCF 中加入 `<include file="field.xml"/>`，保留场地资源文件的相对位置；机器人仍放在自己的 `worldbody` 中。可直接以 `scene_physics.xml` 为起点添加机器人。

```xml
<mujoco model="my_robot_on_rmuc2027">
  <compiler angle="radian" autolimits="true" fusestatic="true" discardvisual="true"/>
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>
  <include file="field.xml"/>
  <worldbody>
    <!-- 在这里放入自己的机器人 body -->
  </worldbody>
</mujoco>
```

场地碰撞体 `contype=1, conaffinity=0`；机器人碰撞几何的 `conaffinity` 必须包含位 1，常见的默认 `1/1` 可以接触。示例测试车使用 `contype=2, conaffinity=1`。不要给真实机器人的碰撞体设置 `conaffinity=0`。

场地碰撞体属于显示组 **3**，显示外表面属于组 **1**。组开关只影响可见性，不影响碰撞。直接用通用查看器打开 `scene_physics.xml` 时，需要打开组 3 才能看到场地；本包的 `--physics` 查看命令会自动处理。

`discardvisual` 会删除整个模型中 `contype=0, conaffinity=0` 的纯显示几何，也包括机器人自己的显示外壳。训练时可以使用；需要相机图像时使用 `scene.xml` 并保留显示网格。

## 坐标与接触参数

- 单位为米、千克、秒，Z 向上，重力为 −9.81 m/s²。
- 场地 X 为长向，Y 为短向，XY 原点在场地中部附近。包含外围结构的碰撞范围约为 X ±14.134 m、Y ±8.002 m、Z −0.194～3.718 m。
- 地面保留 CAD 约 1.5° 的双坡，地表高度随位置变化；不要再叠加一个大平面。将机器人放在局部地面上方留少量落下余量。
- 对原 STEP 的世界坐标：先把毫米换成米，再平移 `[0, -1.6243436765, 1.8509]` m；各实例装配姿态已包含在几何中。
- 默认步长 2 ms、`implicitfast`、Newton。场地摩擦 `0.9 0.005 0.0001`，`solref=0.006 1`。摩擦为可调假定，未测定实物材料；真实机器人的轮胎和控制器需要单独调试。

## 测试边界

测试使用 20 kg 通用四轮车、半径 0.10 m 的车轮、无悬挂。六条路线和高台阶阻挡测试不等于实际参赛机器人的整场可通行性；本车把较高台阶作为障碍物，不承诺能够攀爬。

台阶精确拼缝处存在微米以下的浮点坐标偏移，零宽度射线可能命中邻阶或下方地面。两侧内部采样及有限半径接触已另行核对，原始射线差异保留在 `export_validation.json`。若用射线建立高度图，应避开恰好落在竖直接缝上的单点，或取微小邻域的多点结果。

运行速度数据只表示本机单 CPU、单辆测试车、无渲染的物理步进速度，包含完整 1,365 个碰撞体；不代表 GPU 并行训练吞吐，也没有与原始 STEP 场景进行速度对比。

## 来源

源文件 `RMUC2027_V1.0.0.stp`，外形已按官方 2027 V1.0.0 规则手册相关地形图核对；几何近似误差针对 CAD 缓存离散面，不是对实物尺寸的保证。`metadata.json` 保留实例来源、几何范围及审核稿哈希。

- [场地图纸原帖](https://bbs.robomaster.com/article/1970956?source=9)
- [2027 官方资料中心](https://bbs.robomaster.com/wiki/20204847)
- [比赛规则手册页面](https://bbs.robomaster.com/wiki/20204847/1969149?source=7)
