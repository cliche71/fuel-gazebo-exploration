> 后续记录：2026-10-18 用户机已通过 120 秒飞行检查。日常启动以仓库根目录的使用说明为准；本文件保留原交付时的检查范围描述。见 [当前使用说明](../docs/OPERATIONS.md) 和 [实测记录](../docs/VALIDATION.md)。

# FUEL 官方流程的 Gazebo 对照版本

版本 0.1.3 针对“转向速度超过限制后停止”增加整段转向检查和有界转向求解，
使用已有探索参数，不提高控制接口的停止阈值。飞行位置路线及其时间保持不变。
已有 0.1.2 工作区使用 `fuel_official_yaw_fix.zip` 更新后重新编译；完整步骤见
`YAW_FIX.md`。从本完整包新建工作区则已经包含修正，无需另外安装增量包。

版本 0.1.2 修正了飞行指令时间检查：单条过期消息直接丢弃，时间戳略超前于本地时钟的消息
等待到时再使用，不再因单条时间异常立即停止。原有指令有效期与断流停止条件保留；
不会修改原消息时间或放宽转向、速度限制。新增 `/fuel_official/command_timing` 及首次
停止记录，检查报告会同时保存这些数据。已有工作区使用独立的 `fuel_official_time_fix.zip`
更新，无需重新编译。完整复测步骤见 `TIME_FIX.md`。

版本 0.1.1 补齐了 multi_map_server 的消息生成编译顺序。此前已安装 0.1.0、遇到
`MultiOccupancyGrid.h` 不存在时，可用随包的 `scripts/fix_message_dependency.py`
修复现有 `~/fuel_official_ws`，然后直接重新编译；它会保留修复记录，不更改飞行或相机。

本包使用官方办公室、官方连续探索流程和 Gazebo 深度观测。原来的 `fuel_ws`、`rotors_ws` 与悬停启动文件不会被覆盖。

新目录为 `~/fuel_official_ws`。官方源码固定在 `662dd23c7b52b258d3c4a0155ff6632118e8984f`；所有编译源文件及所需资源随包提供，不需要重新下载仓库。只省略了官方 README 中的演示动图。

已验证的无人机质量、惯性、旋翼和控制参数保留。模拟相机的真实光心改为官方示例采用的机身位置，它报告的位置也同步改变。相机近处 0.35 米不参与渲染，以避开无人机自身外观；这属于仿真设置，与原 VI 相机不同。细节见 `CAMERA.md`。

## 1. 创建独立版本并编译

解压整个 ZIP，不要只取其中一个文件夹。在包含 `fuel_gazebo_official_baseline` 和 `fuel_gazebo_official_world` 的目录打开终端：

```bash
python3 fuel_gazebo_official_baseline/scripts/prepare_workspace.py
```

应看到 `OFFICIAL WORKSPACE PREPARED`。目标目录已存在而且内容不属于本包时，脚本会停止，不覆盖它。

另开终端编译。不要 source 旧的 `~/fuel_ws/devel/setup.bash`：

```bash
source /opt/ros/noetic/setup.bash
source ~/rotors_ws/devel/setup.bash
cd ~/fuel_official_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2 -l2
source ~/fuel_official_ws/devel/setup.bash
rosrun fuel_gazebo_official_baseline make_official_robot.py
```

最后一条只生成新的无人机描述文件，不改正在运行的仿真。这份文件来自本次上传的已验证机型。

## 2. 启动新仿真，检查相机

关闭原 Gazebo 启动终端和导航、探索启动终端。不要同时运行两套 Gazebo。

终端 A：

```bash
source ~/rotors_ws/devel/setup.bash
source ~/fuel_official_ws/devel/setup.bash
roslaunch fuel_gazebo_official_baseline simulation.launch
```

等待无人机在办公室原点附近升至约 1 米并稳定悬停。

终端 B：

```bash
source ~/rotors_ws/devel/setup.bash
source ~/fuel_official_ws/devel/setup.bash
rosrun fuel_gazebo_official_baseline check_official.py --phase camera
```

应看到 `OFFICIAL CAMERA CHECK PASSED`。请保留报告；失败时提供报告及终端输出，暂停后续步骤。这个检查确认正在使用新相机，并检查数据持续更新、质量参数和悬停状态。

## 3. 启动 FUEL，执行官方准备运动

相机检查通过后，终端 C：

```bash
source ~/rotors_ws/devel/setup.bash
source ~/fuel_official_ws/devel/setup.bash
roslaunch fuel_gazebo_official_baseline planner.launch
```

启动后只等待，不会自动探索。真实相机参数送达后，FUEL 才启动。

终端 B：

```bash
rosservice call /fuel_official/command_bridge/prepare "{}"
rosrun fuel_gazebo_official_baseline check_official.py --phase ready
```

`prepare` 会让无人机按官方示例的指令先升高 1 米，再下降回原高度，然后等待测得的位置稳定。没有后退动作。
这是官方相同的高度指令序列，改由 Gazebo 接口在准备好后执行；不是启动时未经检查就重复发送。

应同时看到服务返回 `success: True`、`OFFICIAL READY CHECK PASSED`。后者要求已完成准备运动，已有可通行的第一步和可探索区域，FUEL 仍在等待开始。

## 4. 开始探索并检查实际飞行

```bash
rosservice call /fuel_official/command_bridge/start "{}"
rosrun fuel_gazebo_official_baseline check_official.py --phase flight --seconds 120
```

开始指令只发送一次，服务返回成功表示规划器已接收，不代表已经完成飞行。
飞行报告检查记录期间的位置变化和多次路线更新；无人机不动不会算通过。

需要停止时：

```bash
rosservice call /fuel_official/command_bridge/stop "{}"
```

停止后保持测得位置；再次探索需要重启 `planner.launch`，重新完成准备和检查。仿真时间被重置、数据中断或出现其他飞行指令发布者时，也会停止本次执行。

## 结果如何判断

- 场景、悬停及相机检查通过：说明新模拟环境和观测已经接好。
- 准备检查通过：说明官方准备运动已实际完成，并能从当前位置迈出第一步。
- 飞行检查通过：说明记录期间确实发生位移和多次路线更新，不代表整个办公室已经探索完成。
- 完整探索还需观察持续转向、返回、地图增长和最终结束状态。

每次检查会在主目录生成 `fuel_official_阶段_日期_时间.txt`。不要把仅显示路线、开始指令被接收或离线测试通过，当作实际探索成功。

## 与官方完全相同吗

探索区域选择、访问顺序和连续更新路线的规则保留官方版本。Gazebo 接口、相机与动力学需要适配；另修复了已确认的无效输入、越界和数据初始化问题。所有源码改动列在 `CHANGES.md`，没有声称源码逐字不变。

当前速度、加速度参数均为 0.6，障碍余量为 0.35 米；这些设置适用于当前 RotorS 机型，与官方演示的速度参数不同。

本包没有把完整场景交给在线规划地图，也没有直接把未知区域改成空闲。相机位置来自新模型实际设置。

交付前通过的是离线源码、相机位置、起步路线及命令转换检查。本环境没有 ROS/Gazebo，真实编译、显示和飞行必须按上面步骤在你的机器验证。当前支持教程中的 `devel` 工作区启动方式。
