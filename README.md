# FUEL Gazebo Exploration

基于官方 FUEL 的 Gazebo Classic / RotorS Hummingbird 自主探索适配与运行说明。
当前交付版本：**0.1.3**；说明更新日期：**2026-10-18**。

公开仓库：[cliche71/fuel-gazebo-exploration](https://github.com/cliche71/fuel-gazebo-exploration)。

## 当前已经做到什么

用户在 Ubuntu 20.04、ROS Noetic、Gazebo 11.15.1 上完成了实际编译与 Gazebo 仿真飞行检查；这不是实体无人机飞行测试：

| 项目 | 2026-10-18 实测结果 |
| --- | --- |
| 连续检查时间 | 120 秒 |
| 检查结果 | PASSED |
| 记录到的不同路线编号 | 171 个，编号 3–173 |
| 相对记录起点的最大直线位移 | 14.8593 米，不是累计航程 |
| 检查结束时的状态 | 正在飞行，EXEC_TRAJ / RUNNING |
| 首次异常停止记录 | 无，first_stop 为 null |

这证明本次记录中的持续自主探索已跑通。完整办公室覆盖、最终 FINISH 和多次重复运行尚待单独记录。
指标来源见 [验证记录](docs/VALIDATION.md) 和 [结果摘要](validation/2026-10-18-flight-summary.json)。

本仓库保存与该次运行对应的 **0.1.3 交付代码、补丁和说明**，不声称已经逐文件取得用户电脑上的最终源码。
电脑上的成功版本另有完整源码备份，见 [备份与恢复](docs/OPERATIONS.md#备份与恢复)。

## 从哪里开始

- 已有成功环境，日常启动：读 [使用说明](docs/OPERATIONS.md#日常启动顺序)。
- 新建独立 FUEL 工作区：读 [首次准备](docs/OPERATIONS.md#首次准备独立工作区)。
- 了解与官方的差别：读 [改动范围](docs/OPERATIONS.md#与官方版本的差别) 和 [完整源码改动记录](fuel_gazebo_official_baseline/CHANGES.md)。
- GitHub 建仓与日常保存：读 [仓库管理说明](docs/GITHUB.md)。
- 检查仓库文件：执行 `python3 tools/verify_repository.py`，此项不运行 ROS 或 Gazebo。

**启动顺序必须是：Gazebo → 确认约 1 米稳定悬停 → FUEL → prepare → ready 检查 → start。**
如果在无人机仍靠近地面时启动 FUEL，当前程序可能持续保持低处的位置。恢复方法见使用说明。

## 环境与首次准备

本仓库面向已经装好并编译成功的 ROS Noetic、RotorS 和 FUEL 依赖环境；它不是操作系统或完整依赖镜像。
已验证的 RotorS 工作区为 `~/rotors_ws`，独立 FUEL 工作区为 `~/fuel_official_ws`。

在仓库根目录执行：

```bash
python3 fuel_gazebo_official_baseline/scripts/prepare_workspace.py
```

然后在安装了 ROS 的电脑上编译：

```bash
source /opt/ros/noetic/setup.bash
source ~/rotors_ws/devel/setup.bash
cd ~/fuel_official_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2 -l2
source ~/fuel_official_ws/devel/setup.bash
rosrun fuel_gazebo_official_baseline make_official_robot.py
```

已有工作区时，准备程序会检查而不是覆盖不匹配的文件。已成功运行的电脑无需为了建立仓库重新执行准备或覆盖工作区。

## 仓库内容

| 路径 | 用途 |
| --- | --- |
| `fuel_gazebo_official_baseline/` | Gazebo 连接程序、配置、启动文件、官方源码补丁与固定源码压缩包 |
| `fuel_gazebo_official_world/` | 从官方办公室点云转换的 Gazebo 场景和检查程序 |
| `docs/OPERATIONS.md` | 使用、故障判断和备份恢复说明 |
| `docs/VALIDATION.md` | 已验证结果与验证范围 |
| `docs/GITHUB.md` | 创建私有仓库、保存修改和标记版本的方法 |
| `tools/backup_fuel_success.sh` | 在 ROS 电脑上备份成功源码及配置 |
| `tools/verify_repository.py` | 不依赖 ROS 的仓库文件检查 |
| `validation/` | 不含原始截图和个人目录信息的结果摘要 |

两个 ROS 包必须保持相邻，准备程序通过这一关系找到场景包。仓库本身不直接作为 Catkin 工作区编译。

官方 FUEL 源码固定于提交 `662dd23c7b52b258d3c4a0155ff6632118e8984f`。
源码压缩包完整保留在 `fuel_gazebo_official_baseline/vendor/` 中，不依赖在线下载；原始文件记录与补丁支持核对修改来源。

## 当前边界

- 使用 Gazebo 实际深度观测和测量位置，在线地图从观测逐步建立。
- 保留官方探索区域选择、访问顺序和连续规划流程，另有明确记录的兼容修正及转向约束。
- 相机位置、可视范围和办公室几何是本适配中的仿真设置，并非原 VI 相机或真实飞行实验。
- 规划参数中的 0.6 是配置值，不能当作实测速度始终不超过 0.6 m/s 的证明。
- RViz 空标记提示、转子重复坐标提示及启动先后顺序问题仍有记录，没有在本仓库整理中改动飞行代码。

## 来源

- [官方 FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)
- [RotorS](https://github.com/ethz-asl/rotors_simulator)
- [来源与文件版本](SOURCE_MANIFEST.md)

保留官方 GNU GPL v3 许可证和各文件原有第三方声明，见 [LICENSE](LICENSE)。本仓库不把上游算法或资源声明为原创。
