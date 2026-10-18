> 后续记录：2026-10-18 用户机已通过 120 秒飞行检查。日常启动以仓库根目录的使用说明为准；本文件保留原交付时的检查范围描述。见 [当前使用说明](../docs/OPERATIONS.md) 和 [实测记录](../docs/VALIDATION.md)。

# 独立的官方相机位置配置

此配置使用收到的、已经成功悬停的 Hummingbird + VI 模型生成一个新文件。原来的 `vi_hover.launch` 和模型文件不修改；正在运行的 Gazebo 也不修改。只有关闭旧仿真并启动本包的 `simulation.launch` 后才使用新文件。

## 相机的位置和图像

官方 FUEL 的 CPU 仿真相机与机身原点重合，向机身正前方观察，光学坐标对应 `Rz(-pi/2) Rx(-pi/2)`。这与原 VI 深度相机向前、向右偏置且向下俯视的配置不同。

新配置把**真正生成深度图的相机**移到测量里程计的原点，同时提供一致的 `hummingbird/fuel_official_camera_optical_link`。并非只修改深度图上的坐标名称。sensor 仍依附原安装 link，以免移动带质量的物理部件；其自身 pose 抵消原安装偏置。新 optical link 没有质量、惯性、外观或碰撞。

图像大小 640 × 480、15 Hz，水平视角根据官方焦距 387.229248046875 计算。Gazebo 的默认主点与官方主点略有差别，因此连接 FUEL 时必须使用此相机**实际发布的 CameraInfo**，不能把官方内参直接写死。深度图、相机信息的原话题名保持不变。

## 仿真近裁剪

近裁剪为 0.35 m，远裁剪为 5 m。机身中心位于自身可见模型内部，必须略过机身和旋翼，才能得到类似官方理想相机的视野。已核对官方 RotorS 的原模型：机身 mesh 向前最大约 0.195 m，旋翼中心到机身中心 0.17 m，转动后最前约 0.271 m；0.35 m 近裁剪位于它们之外。

这仍是**理想化的 Gazebo 相机**：近裁剪以内的障碍也不会出现在图像中，不能把它描述成真实 VI 相机，也不能声称与官方点云生成深度图完全相同。FUEL 在线地图仍只处理相机观测；没有把完整办公室地图直接作为已知地图。

## 保持的内容

- 原有所有 link、joint、质量、惯性、碰撞与外观不变。
- 原有电机、IMU、测量里程计插件不变。
- 原悬停使用的质量 0.87607 与已经验证的旋翼配置不变。
- 新世界使用官方办公室外形，并保留 RotorS 原 `basic.world` 的动力学配置和 ROS interface plugin。
- 启动后仍用原 RotorS 悬停例程到 `(0, 0, 1)`；本文件不包含探索开始动作。

## 生成和启动

先生成新的模型文件；不需要运行 Gazebo：

```bash
python3 ~/fuel_official_ws/src/fuel_gazebo_official_baseline/scripts/make_official_robot.py
```

默认读取包内收到的原始模型，生成 `~/.ros/fuel_official_baseline/robot.urdf` 与检查记录 `robot.json`。也可用 `--input 文件` 或 `--from-running` 读取原模型，后者只读取参数。输入必须与已验证模型的物理内容一致，否则停止。

关闭旧 Gazebo 启动程序后，在新终端执行。不要 source 原来的 `fuel_ws`：

```bash
source /opt/ros/noetic/setup.bash
source ~/rotors_ws/devel/setup.bash
source ~/fuel_official_ws/devel/setup.bash
roslaunch fuel_gazebo_official_baseline simulation.launch
```

该启动文件不加载旧测试方块，不启动 FUEL 探索，不修改旧启动文件。实际深度图、CameraInfo、位置和首次规划仍需在用户的 Gazebo 中检查；此包的离线检查不能替代实际飞行。
