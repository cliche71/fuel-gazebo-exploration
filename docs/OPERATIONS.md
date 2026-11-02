# 运行说明

以下命令在已完成构建的 `~/fuel_gazebo_ws` 中执行。启动场景、起飞准备和开始探索是三个阶段。

## 启动场景与规划器

终端一：

```bash
cd ~/fuel_gazebo_ws
./tools/run.sh
```

程序生成已校准的机体描述，启动办公室场景，使 Hummingbird 起飞至约 `(0, 0, 1)` 米悬停，再启动规划器。此时尚未开始探索。

可选参数：

```bash
./tools/run.sh --gui false
./tools/run.sh --seed 20261101
```

`--gui false` 关闭 Gazebo 和 RViz 窗口。`--seed` 设置 Gazebo 随机种子；相同种子不保证线程调度、地图更新和规划结果完全一致。

## 准备与开始探索

终端二：

```bash
cd ~/fuel_gazebo_ws
source ./env.sh
```

确认位置反馈和传感器就绪：

```bash
rostopic echo -n 1 /hummingbird/odometry_sensor1/odometry/pose/pose/position
rostopic echo -n 1 /fuel_official/sensor_report
rostopic echo -n 1 /fuel_official/planner/state
```

规划器应处于等待触发状态 `WAIT_TRIGGER`。随后调用准备服务：

```bash
rosservice call /fuel_official/command_bridge/prepare "{}"
rostopic echo /fuel_official/command_ready
```

准备阶段执行向上、向下的初始化运动，并等待实测悬停恢复。服务返回 `success: true` 且 `command_ready` 为 `true` 后，按 `Ctrl+C` 退出查看，只发送一次探索指令：

```bash
rosservice call /fuel_official/command_bridge/start "{}"
```

可在以下话题查看探索状态、命令处理状态和传感器状态：

```bash
rostopic echo /fuel_official/planner/state
rostopic echo /fuel_official/command_status
rostopic echo /fuel_official/sensor_report
```

原规划器在没有可覆盖活动前沿时进入 `FINISH`。少量残余未知或休眠前沿不阻止这一结束条件。

## 停止与重新运行

需要手动中止时：

```bash
rosservice call /fuel_official/command_bridge/stop "{}"
```

手动停止使命令桥进入目标保持，不等于探索完成，也不保证机体瞬间停止。随后在终端一按 `Ctrl+C` 结束本轮进程。

下一轮应重新启动场景和规划器。出现 `HALTED` 时，先查看 `command_status` 中的原因，不通过反复调用 `start`、清空地图或修改状态继续同一轮。

## 环境、端口与日志

每个操作终端均需加载 `env.sh`。默认公共依赖目录为 `~/rotors_ws`；其他位置可在构建和运行前统一设置：

```bash
export ROTORS_WS=/path/to/rotors_ws
```

默认端口为：

| 设置 | 默认值 |
| --- | --- |
| ROS master | `http://localhost:11421` |
| Gazebo master | `http://localhost:11422` |

可使用 `FUEL_ROS_MASTER_URI` 和 `FUEL_GAZEBO_MASTER_URI` 修改，所有终端应保持一致。不要同时在相同端口启动另一套同名仿真。

日志位于 `logs/run_<时间>/`，内部规划记录位于 `internal/`，ROS 日志位于 `ros/`。可在启动前通过 `FUEL_REGRESSION_TRACE_DIR` 和 `ROS_LOG_DIR` 指定输出位置。

机体描述和 TSP 可写文件分别位于 `~/.ros/fuel_gazebo/robot.urdf` 与 `~/.ros/fuel_gazebo/tsp/`。相机只读诊断命令为：

```bash
rosrun fuel_gazebo_official_baseline check_official.py --phase camera --seconds 8
```

该诊断不会发送准备或探索指令。常见运行限制见 [已知问题](KNOWN_LIMITATIONS.md)。
