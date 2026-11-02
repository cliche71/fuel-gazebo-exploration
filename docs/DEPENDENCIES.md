# 依赖与构建

## 环境

| 组件 | 使用版本或要求 |
| --- | --- |
| 系统 | Ubuntu 20.04 |
| ROS | ROS Noetic，catkin，Python 3.8 |
| 仿真 | Gazebo Classic 11.15.1，`gazebo_ros`，Gazebo 开发库 |
| 飞行器与控制器 | RotorS、mav_comm、Hummingbird、Lee 位置控制器 |
| 优化与几何 | NLopt 2.7.1、Eigen3、PCL、OpenCV、Boost |
| ROS 图像接口 | `cv_bridge`、`image_transport`、相关消息包 |
| Python | `rospy`、PyYAML、NumPy |

RotorS 与 mav_comm 使用以下提交：

| 项目 | 固定提交 |
| --- | --- |
| [RotorS](https://github.com/ethz-asl/rotors_simulator) | `cd813b7a8c375d677352aa20ad20047feb661126` |
| [mav_comm](https://github.com/ethz-asl/mav_comm) | `56122218eb1ec34deaa641598ebacf24e26c349b` |

## 公共 RotorS 工作区

公共依赖默认放在 `~/rotors_ws`，两个项目分别位于其 `src/` 下。安装各项目声明的 ROS 与系统依赖后，在仅加载 ROS Noetic 的终端构建：

```bash
cd ~/rotors_ws
source /opt/ros/noetic/setup.bash
catkin config --extend /opt/ros/noetic --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_DISABLE_FIND_PACKAGE_mavlink=TRUE
catkin build
```

以上命令使用 `catkin-tools`。公共工作区不应依赖另一套 FUEL 工作区的构建结果。

## NLopt

轨迹优化需要 NLopt 的 C++ 头文件 `nlopt.hpp` 和共享库。可安装对应开发包，或从 NLopt 2.7.1 源码构建：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DNLOPT_PYTHON=OFF \
  -DNLOPT_GUILE=OFF \
  -DNLOPT_OCTAVE=OFF \
  -DNLOPT_MATLAB=OFF
cmake --build build -j2
```

将构建结果安装到所选前缀，并确保 CMake 能找到头文件和库。非标准位置可设置 `NLOPT_INCLUDE_DIR` 与 `NLOPT_LIBRARY`。仅提供 C 接口而没有 `nlopt.hpp` 的安装不能满足当前构建要求。

## 构建本项目

```bash
git clone https://github.com/cliche71/fuel-gazebo-exploration.git ~/fuel_gazebo_ws
cd ~/fuel_gazebo_ws
./tools/build.sh
```

公共依赖不在默认位置时，在构建前设置 `ROTORS_WS`：

```bash
export ROTORS_WS=/path/to/rotors_ws
./tools/build.sh
```

构建脚本加载 ROS Noetic、指定的 RotorS 工作区和当前项目，使用 Release 配置与 `-j2 -l2`。运行入口使用当前项目的 `devel` 空间。

仓库包含 FUEL 规划与地图源码、消息、LKH 求解库、办公室资源，以及 `fuel_camera_adapter` 的深度相机、接触和随机种子插件源码。相机插件需要 Gazebo / OGRE 开发库。`src/CMakeLists.txt` 使用 ROS Noetic 的标准 catkin 入口。

构建完成后的运行步骤见 [运行说明](OPERATIONS.md)。
