# FUEL Gazebo Exploration

在 **ROS Noetic、Gazebo Classic 和 RotorS Hummingbird** 中运行 FUEL 无人机自主探索。

本项目基于 [HKUST FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)，提供办公室仿真场景、深度相机、飞行器模型与控制接口。无人机通过相机观测逐步建图，由 FUEL 选择探索目标并规划轨迹，再由 RotorS 控制器和飞行器动力学执行。

[安装与构建](#安装与构建) · [运行探索](#运行探索) · [配置](#主要配置) · [测试结果](#测试结果) · [文档](#文档)

## 功能

- Gazebo 办公室场景与 RotorS Hummingbird 飞行仿真。
- 深度图、相机标定和里程计接入，使用在线地图进行前沿探索。
- 轨迹命令转换、独立起飞准备和连续重新规划。
- 连续转向修正、失败候选状态恢复，以及执行前轨迹检查和限期截停。
- 构建与运行脚本，包含模型、相机、规划器及运行日志配置。

Gazebo 保存场景的物理几何；规划器的地图由传感器观测逐步建立。飞行器经过动力学和控制器跟随轨迹，实际运动会影响后续观测与规划。

## 安装与构建

### 环境要求

| 项目 | 使用环境 |
| --- | --- |
| 系统 | Ubuntu 20.04 |
| ROS | ROS Noetic、catkin |
| 仿真 | Gazebo Classic 11.15.1、RotorS、mav_comm |
| Python | Python 3.8、rospy、PyYAML、NumPy |
| 主要开发依赖 | Eigen3、PCL、OpenCV、Boost、NLopt C++ 接口，以及 ROS / Gazebo 开发库 |

先按 [依赖说明](docs/DEPENDENCIES.md)准备 RotorS、mav_comm 和其他开发依赖。默认 RotorS 工作区为 `~/rotors_ws`。使用其他位置时，在构建和运行终端中一致设置：

```bash
export ROTORS_WS=/path/to/rotors_ws
```

获取仓库并构建：

```bash
git clone https://github.com/cliche71/fuel-gazebo-exploration.git ~/fuel_gazebo_ws
cd ~/fuel_gazebo_ws
./tools/build.sh
```

仓库包含 FUEL 运行源码、适配包、相机插件和场景资源。构建产物位于本目录的 `build/` 和 `devel/`。

## 运行探索

Gazebo 启动、起飞准备和开始探索是三个独立阶段。每轮运行使用独立的场景和规划器进程。

### 1. 启动仿真

终端一：

```bash
cd ~/fuel_gazebo_ws
./tools/run.sh
```

脚本生成相机模型，启动办公室场景，让无人机起飞至约 `(0, 0, 1)` 悬停，再加载规划器。此时尚未开始探索。

不需要 Gazebo 和 RViz 窗口时：

```bash
./tools/run.sh --gui false
```

可通过 `--seed N` 指定 Gazebo 随机种子。相同种子不保证轨迹完全相同。

默认 ROS 和 Gazebo 端口分别为 `11421`、`11422`，启动前应结束占用这些端口的旧仿真。端口设置与运行日志位置见 [运行说明](docs/OPERATIONS.md)。

### 2. 完成准备

终端二进入仓库并加载环境：

```bash
cd ~/fuel_gazebo_ws
source ./env.sh
```

等待传感器就绪、规划器进入 `WAIT_TRIGGER`，然后执行准备运动：

```bash
rosservice call /fuel_official/command_bridge/prepare "{}"
rostopic echo /fuel_official/command_ready
```

`prepare` 执行初始化运动并等待实测位置稳定。服务返回 `success: True`，且 `command_ready` 显示 `true` 后，按 `Ctrl+C` 退出状态查看。

### 3. 开始探索

在终端二执行一次：

```bash
rosservice call /fuel_official/command_bridge/start "{}"
```

可查看规划器状态：

```bash
rostopic echo /fuel_official/planner/state
```

探索完成沿用 FUEL 的原 `FINISH` 条件：没有可覆盖的活动前沿。小前沿、休眠前沿和少量未观测区域可能仍然存在。

### 4. 手动停止

```bash
rosservice call /fuel_official/command_bridge/stop "{}"
```

手动停止会进入目标保持，不算探索完成。随后在终端一按 `Ctrl+C` 关闭本轮进程。重新探索或出现 `HALTED` 后，应重新启动仿真和规划器。

## 主要配置

默认使用办公室场景的 [`rotors_1mps.yaml`](src/fuel_gazebo_official_baseline/config/profiles/rotors_1mps.yaml)。原 0.6 m/s、0.6 m/s² 配置保留为 [`regression_C_final.yaml`](src/fuel_gazebo_official_baseline/config/profiles/regression_C_final.yaml)，可通过以下命令选择：

```bash
./tools/run.sh --profile regression_C_final
```

| 项目 | 默认值 |
| --- | ---: |
| 规划速度 / 加速度配置 | 1.0 m/s / 1.0 m/s² |
| 地图分辨率 | 0.1 m |
| 障碍膨胀 | 0.35 m |
| 优化避障距离 | 0.7 m |
| 执行前轨迹检查时长 | 2.1 s |
| 轨迹检查距离余量 | 0.1 m |
| 截停提前量 | 0.05 s |
| 深度图分辨率 / 视场 | 640×480 / 80°×60° |
| 地图最大射线距离 | 4.5 m |

这些是针对当前仿真平台的适配配置。运动与避障优化含软约束，配置值不等于实际运动的硬上界；限期截停也不保证机体同时停住。完整字段、相机参数和各距离含义见 [参数说明](docs/PARAMETERS.md)。

## 测试结果

2026-11-02 使用当前 1.0 m/s、1.0 m/s² 配置的五次办公室场景独立运行结果如下：

| 运行次数 | 自然完成 | 停滞 | 接触记录 |
| ---: | ---: | ---: | ---: |
| 5 | 4 | 0 | 1 |

四次自然完成分别耗时 173.22、194.65、205.13、192.09 仿真秒；另一轮在探索后 41.32 秒发生旋翼与墙面接触。旧 0.6 配置的五轮记录为四次完成、一次停滞，两套配置的结果分别保留。逐次耗时、实测航程和历史结果见 [测试结果](docs/VALIDATION.md)。

当前版本的已知问题包括轨迹跟随误差、突然保持时的运动响应，以及轨迹检查中的未知区域状态判断。详细说明见 [已知限制](docs/KNOWN_LIMITATIONS.md)。当前结果集中在办公室和 Hummingbird 仿真条件，不代表其他场景或实体无人机的运行表现。

## 文档

| 文档 | 内容 |
| --- | --- |
| [Gazebo 适配](docs/GAZEBO_ADAPTATION.md) | 接口、相机、数值修正及轨迹执行改动 |
| [依赖说明](docs/DEPENDENCIES.md) | RotorS、mav_comm、NLopt 与构建依赖 |
| [运行说明](docs/OPERATIONS.md) | 启动顺序、状态、端口及日志 |
| [参数说明](docs/PARAMETERS.md) | 当前配置与实际观测参数 |
| [测试结果](docs/VALIDATION.md) | 探索完成、停滞、接触及运行数据 |
| [已知限制](docs/KNOWN_LIMITATIONS.md) | 当前问题与适用范围 |

## 上游项目与许可

- [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)：前沿探索、建图与轨迹规划。
- [RotorS](https://github.com/ethz-asl/rotors_simulator)：飞行器动力学、模型与控制器。
- [mav_comm](https://github.com/ethz-asl/mav_comm)：相关消息与接口。

研究使用请引用原论文 [FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning](https://doi.org/10.1109/LRA.2021.3051563)。

本项目保留上游的 [GPL-3.0 许可证](LICENSE)及各组件的原始声明，详见 [第三方来源与许可](THIRD_PARTY_NOTICES.md)。
