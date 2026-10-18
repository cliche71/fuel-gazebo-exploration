# 0.1.2 飞行指令时间修正

2026-10-18 的日志确认，停止发生在仿真时间 7144.01，原因为
`PositionCommand timestamp is stale or in the future`。20 秒飞行检查的报告生成时间
比该停止早约 55 毫秒，因此检查通过并不代表随后没有停止。旧日志没有记录时间差，
无法断定当时是过期还是超前，也不能将本次停止归因于后来的转向提示。

修正保留已正常工作的官方探索流程、相机位置、地图、控制参数和原有速度限制。
单条旧指令会丢弃；时间戳略超前于本地时钟的指令暂存，等时钟到达原时间后再使用。
不会将旧消息的时间改成当前时间。持续没有有效指令仍会停止，停止后不会自动恢复。
原有有效期仍为 0.3 秒仿真时间、1 秒实际时间，仿真暂停过久仍会停止。

## 安装现有工作区的增量修正

1. 在 Gazebo 与 FUEL 的两个启动终端分别按 Ctrl+C。下一次需要从原点重新开始。
2. 解压 `fuel_official_time_fix.zip`，进入其中的 `fuel_official_time_fix` 文件夹。
3. 在该目录打开终端并运行：

```bash
python3 install.py
```

应输出 `COMMAND TIME FIX INSTALLED`；重复安装会输出 `ALREADY INSTALLED`。
安装器只接受已识别的 `~/fuel_official_ws`，自动备份被替换的两个脚本和版本文件，
不会重建工作区或启动飞行。备份目录会打印出来。无需重新编译。

## 从原点复测

终端 A：

```bash
source ~/rotors_ws/devel/setup.bash
source ~/fuel_official_ws/devel/setup.bash
roslaunch fuel_gazebo_official_baseline simulation.launch
```

等待 1 米处稳定悬停。终端 C：

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

返回 `success: True` 后立即执行 120 秒记录，覆盖更长的连续飞行过程：

```bash
rosrun fuel_gazebo_official_baseline check_official.py --phase flight --seconds 120
```

将生成的 `fuel_official_flight_日期_时间.txt` 提供给助手。若中途不动，可在保持运行的
情况下查看以下两项；不要在取证前重启或调用停止服务，以免丢失原现场：

```bash
timeout 5s rostopic echo -n 1 /fuel_official/command_status
timeout 5s rostopic echo -n 1 /fuel_official/command_timing
```

检查只覆盖记录期间的运动和输入状态。完整探索仍需持续观察地图增长、转向和最终结束。
转向提示及 RViz 空显示提示没有混入此次修正；若实际因此停止，会记录具体原因。

## 已完成的离线验证

原有 17 项控制与坐标测试、独立 13 项时间回归测试通过。其中包含 1000 条连续指令、
旧消息混入、时钟领先、缓存保留原接收时间、真实断流、时钟重置和首次原因保留。
这些验证运行了实际脚本回调，但没有模拟 ROS 网络、Gazebo 动力学或真实长期飞行。
