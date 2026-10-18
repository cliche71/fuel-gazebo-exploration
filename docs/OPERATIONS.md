# FUEL 接入 Gazebo：成功版本使用说明

版本：0.1.3。更新日期：2026-10-18。

## 当前成果

FUEL 已能够使用 Gazebo 中 Hummingbird 的深度观测与测量位置，逐步建立地图、选择探索方向，并驱动无人机连续飞行。
2026-10-18 的 120 秒 Gazebo 仿真飞行检查通过，记录到 171 个不同路线编号，未记录异常停止。
这次仿真由用户在自己的 ROS 电脑上实际运行，不是实体无人机飞行测试；文档编写环境只进行了离线检查。

当前确认的是持续探索成功，尚未取得完整探索结束的记录，也没有将其表述为全部空间已被覆盖。

## 系统各部分负责什么

| 部分 | 作用 |
| --- | --- |
| Gazebo 办公室场景 | 提供墙、障碍物与物理碰撞环境 |
| RotorS Hummingbird | 提供无人机运动、旋翼、传感器和原有控制程序 |
| 深度相机 | 根据无人机当前的位置和朝向生成深度观测 |
| 传感器连接程序 | 配对深度与测量位置，提供相机在世界中的实际姿态 |
| FUEL | 从观测建图，寻找待探索区域，选择访问顺序并连续生成路线 |
| 飞行指令连接程序 | 将 FUEL 输出转换为 RotorS 接受的指令，并保存异常停止原因 |
| RViz | 显示已观测地图、待探索区域、计划路线与实际飞行记录 |

Gazebo 中的完整办公室负责“环境是什么样”。FUEL 自己的在线地图负责“目前看到了什么”。
完整场景用于产生相机观测，没有直接作为已知地图交给规划器。

## 已验证环境

| 项目 | 当前配置 |
| --- | --- |
| 系统 | Ubuntu 20.04 |
| ROS | Noetic，`/opt/ros/noetic` |
| 仿真 | Gazebo Classic 11.15.1 |
| 无人机 | RotorS Hummingbird，带 VI 传感器模型 |
| RotorS 工作区 | `~/rotors_ws` |
| 当前独立 FUEL 工作区 | `~/fuel_official_ws` |
| 旧探索工作区 | `~/fuel_ws`，保留作此前工作的副本 |
| 控制器质量参数 | 0.87607 kg |
| 初始悬停目标 | `(0, 0, 1)` 米，朝向为 0 |

在其他电脑上使用，还需要安装并编译对应的 RotorS、消息包和 FUEL 依赖。
当前仓库没有收录已安装的系统软件或用户电脑上的完整 RotorS 源码。
不能据此承诺换一台机器后不需要配置依赖。

## 首次准备独立工作区

已有成功工作区时，日常启动即可；本节用于从交付代码新建独立工作区。

保留仓库中的两个 ROS 包为相邻目录。在仓库根目录运行：

```bash
python3 fuel_gazebo_official_baseline/scripts/prepare_workspace.py
```

程序会核对固定的官方源码，再应用已记录的修改。成功时显示 `OFFICIAL WORKSPACE PREPARED`。
目标目录已存在但内容不匹配时会停止，应保留原目录，不直接覆盖成功版本。

在已经装好依赖的 ROS 电脑上编译：

```bash
source /opt/ros/noetic/setup.bash
source ~/rotors_ws/devel/setup.bash
cd ~/fuel_official_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2 -l2
source ~/fuel_official_ws/devel/setup.bash
rosrun fuel_gazebo_official_baseline make_official_robot.py
```

最后一条生成 `~/.ros/fuel_official_baseline/robot.urdf` 与相应检查记录。
生成配置时无需打开旧 Gazebo。新的模型文件要在下一次启动仿真时才生效。
独立版本的终端不要再加载旧 `~/fuel_ws/devel/setup.bash`。

## 日常启动顺序

### 1. 只启动 Gazebo

终端 A：

```bash
source /opt/ros/noetic/setup.bash
source ~/rotors_ws/devel/setup.bash
source ~/fuel_official_ws/devel/setup.bash
roslaunch fuel_gazebo_official_baseline simulation.launch
```

等待自动悬停。此时不要启动 FUEL。

终端 B：

```bash
source /opt/ros/noetic/setup.bash
source ~/rotors_ws/devel/setup.bash
source ~/fuel_official_ws/devel/setup.bash
rostopic echo -n 1 /hummingbird/odometry_sensor1/odometry/pose/pose/position
```

应看到 `x、y` 接近 0、`z` 接近 1.0。观察无人机已稳定后再继续。
若需要，用同一条命令再次读取位置，不能仅凭出现模型或收到启动日志判断已经悬停。

### 2. 检查实际相机

终端 B：

```bash
rosrun fuel_gazebo_official_baseline check_official.py --phase camera
```

应显示 `OFFICIAL CAMERA CHECK PASSED`。失败时保留报告，不继续发送探索开始命令。

### 3. 启动 FUEL

终端 C：

```bash
source /opt/ros/noetic/setup.bash
source ~/rotors_ws/devel/setup.bash
source ~/fuel_official_ws/devel/setup.bash
roslaunch fuel_gazebo_official_baseline planner.launch
```

等待 `WAIT_TRIGGER`。这表示 FUEL 正在等待开始，不是探索已经开始。

### 4. 完成准备运动

终端 B：

```bash
rosservice call /fuel_official/command_bridge/prepare "{}"
rosrun fuel_gazebo_official_baseline check_official.py --phase ready
```

无人机会按官方示例的高度指令上升，再回到约 1 米并等待稳定。这里没有自动后退动作。
必须同时得到服务返回 `success: True` 和 `OFFICIAL READY CHECK PASSED`。
如果返回 `False`，先看同一次返回中的 `message`，不要继续发送开始命令。

### 5. 开始并记录飞行

终端 B：

```bash
rosservice call /fuel_official/command_bridge/start "{}"
rosrun fuel_gazebo_official_baseline check_official.py --phase flight --seconds 120
```

`start` 成功说明 FUEL 接受了本轮开始请求；是否实际飞行，以测量位置与飞行报告为准。
开始请求只发送一次，不需要反复点击或重复执行。

检查完成后仍可继续观察探索，不必因为 120 秒记录结束就关闭仿真。
报告保存在主目录，名称为 `fuel_official_flight_日期_时间.txt`。

### 6. 判断本轮是否结束

另开已加载相同环境的终端：

```bash
rostopic echo -n 1 /fuel_official/planner/state
```

| 状态 | 含义 |
| --- | --- |
| `WAIT_TRIGGER` | 等待探索开始 |
| `PLAN_TRAJ` | 正在计算下一段路线 |
| `PUB_TRAJ` | 正在提交新路线 |
| `EXEC_TRAJ` | 正在执行路线 |
| `FINISH` | FUEL 判断本轮已无可供继续探索的区域 |

`FINISH` 不是几何上 100% 覆盖的证明，仍应结合最终地图检查明显遗漏。
当前没有承诺探索完成后自动返回起点或自动降落。
飞行指令连接程序可能继续显示 `RUNNING`，结束判断应读取上面的规划状态。

需要主动停止并保持当前位置时：

```bash
rosservice call /fuel_official/command_bridge/stop "{}"
```

主动停止或异常停止后，需要重启本轮 FUEL，并重新完成准备检查。
若无人机已经离开原点，下一轮应重新启动仿真，从初始位置开始。

## 常见问题及已解决原因

| 现象 | 已确认的原因或判断方法 | 处理 |
| --- | --- | --- |
| 高度约 0.285 米，始终不开始 | 在低处启动 FUEL 时，程序可能开始持续保持当前位置；准备检查不能通过 | 停止 FUEL，单独升至约 1 米，再启动 FUEL |
| 一直 `WAIT_TRIGGER`，指令数为 0 | 本轮尚未进入探索，需核对 prepare、ready 和 start 返回结果 | 按顺序检查，每一步均须成功 |
| 起步附近全是未知空间 | 相机位置、方向与初始观测影响第一步是否可走 | 当前版本已将实际相机光心对齐官方示例，并执行准备运动；不要直接把未知区域改为空闲 |
| 曾出现时间异常后停止 | 上一版本对单条时间异常立即停止 | 0.1.2 已修正孤立消息的处理，仍保留真正断流时的停止检查 |
| 曾飞行一段后转向超限停止 | 官方转向求解没有保证整段都满足所需的转向限制 | 0.1.3 增加转向速度、加速度检查和重新求解；未提高停止阈值 |
| `MultiOccupancyGrid.h` 缺失 | 消息生成与使用它的程序编译先后顺序有误 | 已包含编译顺序修正 |
| 只有路线或目标点，没有真实移动 | 显示路线不等于控制程序实际执行 | 读取测量位置与飞行检查报告 |
| 转子坐标重复或 RViz 空标记提示 | 仍有显示方面的问题，本次成功飞行未被它们阻止 | 保留记录，与导致飞行停止的首次原因区分判断 |

### 低处保持时的恢复命令

先在 `planner.launch` 终端按 Ctrl+C，保留 Gazebo。然后执行：

```bash
rosrun rotors_gazebo hovering_example __ns:=/hummingbird \
  _x:=0.0 _y:=0.0 _z:=1.0 _yaw:=0.0
```

等待稳定，再读取测量高度。到达约 1 米后，重新启动 FUEL，执行 prepare、ready、start。
必须先关闭原 FUEL，否则它可能继续发送低处保持指令。

### 中途不动时留下什么数据

保持 Gazebo 和 FUEL 运行，在另一终端执行：

```bash
timeout 5s rostopic echo -n 1 /fuel_official/command_status
timeout 5s rostopic echo -n 1 /fuel_official/command_timing
timeout 5s rostopic echo -n 1 /fuel_official/planner/state
rosrun fuel_gazebo_official_baseline check_official.py --phase flight --seconds 20
```

先保存报告与第一次停止原因，再关闭程序。后续仍出现路线计算日志，不能证明无人机恢复了飞行。

## 相机、地图与关键设置

| 项目 | 当前值或说明 |
| --- | --- |
| 深度图话题 | `/hummingbird/vi_sensor/camera_depth/depth/disparity` |
| 深度编码 | `32FC1`，这里承载米制深度，不能按话题名解释为立体视差 |
| 相机信息 | `/hummingbird/vi_sensor/camera_depth/depth/camera_info` |
| 测量位置 | `/hummingbird/odometry_sensor1/odometry` |
| RotorS 飞行输入 | `/hummingbird/command/trajectory` |
| 新相机光学坐标 | `hummingbird/fuel_official_camera_optical_link` |
| 相机光心 | 与测量位置原点重合，向机身前方观察 |
| 图像 | 640 × 480；使用实际发布的相机参数 |
| 相机近裁剪 | 0.35 米；近处障碍不会出现在图像中，属于理想化仿真设置 |
| 办公室场景 | 官方 office.pcd 按 0.1 米网格转换，147 个合并实体；保留原坐标 |
| 地图分辨率 | 0.1 米 |
| 障碍膨胀余量 | 0.35 米 |
| 规划速度、加速度配置 | 分别 0.6 和 0.6；这些配置值不等于实测严格上限 |
| 探索转向设置 | 约 60 度/秒、90 度/秒² |

转向修正保留位置路线及其时间，终点朝向仍为可优化的目标。
在当前飞行时间内可能只完成部分转向，误差会记录到日志中。
不应将当前适配直接视为真实相机、真实无人机或全部物理场景下的验证结果。

## 与官方版本的差别

官方 FUEL 固定提交：`662dd23c7b52b258d3c4a0155ff6632118e8984f`。

| 保留内容 | 为接入 Gazebo 作出的修改 |
| --- | --- |
| 探索区域选择与访问顺序 | 深度、测量位置与飞行命令连接 |
| 连续更新路线的主要流程 | 实际相机位置、观测范围和参数适配 |
| 在线地图从观测逐步建立 | 办公室点云转换为 Gazebo 可见和可碰撞实体 |
| 官方位置路线与原有优化目标 | 已确认的编译、无效输入、初始化及深度像素处理修正 |
| 原有探索流程 | 0.1.3 增加转向约束与失败结果拒绝 |

这是明确记录修改的 Gazebo 移植版本，不是与官方逐字相同的复现。
没有把完整办公室预先写进规划地图，也没有通过把未知空间直接改为空闲来绕过起步问题。
完整细节见相邻代码包中的 `CHANGES.md`、`CAMERA.md`、`TIME_FIX.md` 和 `YAW_FIX.md`。

## 备份与恢复

2026-10-18 用户已实际执行备份并通过检查。备份目录形式如下，具体名称以备份程序输出为准：

```text
~/fuel_gazebo_backups/official_working_日期时间_编号/
```

| 文件 | 用途 |
| --- | --- |
| `working_source.tar.gz` | 两个工作区的源码、地图、相机配置及检查报告 |
| `FILES.txt` | 备份范围 |
| `SHA256SUMS` | 检查文件是否损坏 |
| `RESTORE.txt` | 恢复说明 |

备份不含 build、devel、系统已安装依赖、ROS 日志或录像。恢复需要原有依赖，并重新编译。
它与 GitHub 仓库作用不同：本机备份保存成功电脑上的实际源码；仓库记录交付代码和后续有意保留的改动。

之后再次备份，在仓库根目录执行：

```bash
bash tools/backup_fuel_success.sh
```

每次产生新的目录，不覆盖上一次备份。

恢复时先关闭 FUEL 和 Gazebo，在相应备份目录执行：

```bash
sha256sum -c SHA256SUMS
```

校验通过后，先解压到一个新的空目录检查内容，保留正在使用的工作区副本，再放回原位置并重新编译。
不要将备份直接覆盖到仍在运行的工作区。

## 后续验证建议

1. 保存一次出现 `FINISH` 的完整运行记录，并检查最终地图中的明显遗漏。
2. 从新启动的 Gazebo 重复运行，确认成功不依赖某次旧地图或旧位置。
3. 需要算法对比时，另行记录总航程、覆盖率、探索用时、碰撞情况和跟踪误差；现有 PASSED 报告不计算这些指标。
4. 后续改动在新分支中进行，保留当前 0.1.3 基线和本机成功备份。
