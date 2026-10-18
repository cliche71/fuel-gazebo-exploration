> 后续记录：2026-10-18 用户机已通过 120 秒飞行检查。日常启动以仓库根目录的使用说明为准；本文件保留原交付时的检查范围描述。见 [当前使用说明](../docs/OPERATIONS.md) 和 [实测记录](../docs/VALIDATION.md)。

# 0.1.3 转向修正与复测

本次报告确认停止发生在仿真时间 80.69、第 26 条路线，原因是
`FUEL command exceeds the command yaw rate limit`。当时消息时间差为 0，传感器正常。
此前的时间修正已经生效。报告里的“准备未完成”是停止后清除准备状态的附带结果。

## 改动范围

保留官方的探索区域选择、访问顺序、寻找路线与连续更新流程。正常转向结果通过检查
就直接使用；不合格时，使用相同目标重新计算转向，限制整段转向速度和加速度，并
与上一段转向衔接。飞行路线及其时间不改变，相机和地图不改变。

使用当前已有的探索设置：转向速度约 60 度/秒、转向加速度约 90 度/秒²。
控制接口的 3 rad/s 停止阈值没有调大。每个曲线区间的真实速度峰值均会检查，
不是只隔一段时间抽样检查。无法满足要求的候选不会发送，原有效路线记录会恢复。求解过程中只保留已经通过检查
的转向结果，求解时间耗尽时也不会发送不合格的中间结果。

终点朝向沿用官方的软目标：在当前飞行时间内可能只能完成部分转向，误差会记录到
日志中。不能据此声称每一段都精确对准最终方向；完整探索还需要实测观察。

这是对转向生成增加约束的源码修改，并非与官方代码逐字相同。原目标和探索规则保留。
本次没有处理 RViz 空显示提示和转子坐标重复提示。

## 安装与编译

适用于已经安装上一轮时间修正的 `~/fuel_official_ws`（0.1.2）。

1. 在 FUEL 和 Gazebo 两个启动终端分别按 Ctrl+C。
2. 解压 `fuel_official_yaw_fix.zip`，进入其中 `fuel_official_yaw_fix` 文件夹打开终端。
3. 执行：

```bash
python3 install.py
```

应显示 `YAW MOTION FIX INSTALLED`，并打印备份目录。安装器会检查现有文件，
只更新指定文件及对应记录，旧文件保存在 `~/fuel_official_ws/source_backups/yaw_motion_013/`。
安装失败会恢复旧文件。重复安装只检查，不重复修改。

然后重新编译：

```bash
source /opt/ros/noetic/setup.bash
source ~/rotors_ws/devel/setup.bash
cd ~/fuel_official_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2 -l2
source ~/fuel_official_ws/devel/setup.bash
```

编译成功后再开始下列步骤。无需重新生成相机配置。

## 从原点重新启动

终端 A：

```bash
source ~/rotors_ws/devel/setup.bash
source ~/fuel_official_ws/devel/setup.bash
roslaunch fuel_gazebo_official_baseline simulation.launch
```

等待无人机在原点附近 1 米高处稳定悬停。终端 C：

```bash
source ~/rotors_ws/devel/setup.bash
source ~/fuel_official_ws/devel/setup.bash
roslaunch fuel_gazebo_official_baseline planner.launch
```

等待 `WAIT_TRIGGER` 后，终端 B：

```bash
source ~/rotors_ws/devel/setup.bash
source ~/fuel_official_ws/devel/setup.bash
rosservice call /fuel_official/command_bridge/prepare "{}"
rosrun fuel_gazebo_official_baseline check_official.py --phase ready
```

只有服务返回 `success: True`、检查显示 `OFFICIAL READY CHECK PASSED` 后再执行：

```bash
rosservice call /fuel_official/command_bridge/start "{}"
```

返回 `success: True` 后立即记录 120 秒飞行：

```bash
rosrun fuel_gazebo_official_baseline check_official.py --phase flight --seconds 120
```

提供生成的 `fuel_official_flight_日期_时间.txt`。若中途停止，保持 FUEL 和 Gazebo
继续运行，在另外终端读取以下信息后再关闭，首次原因将保留：

```bash
timeout 5s rostopic echo -n 1 /fuel_official/command_status
timeout 5s rostopic echo -n 1 /fuel_official/command_timing
```

需要主动停止并悬停时：

```bash
rosservice call /fuel_official/command_bridge/stop "{}"
```

离线测试已经覆盖连续规划和不合格结果拒绝，但本环境没有 ROS/Gazebo，尚未完成
实际编译和完整办公室探索。120 秒记录通过也只代表该段记录通过。
