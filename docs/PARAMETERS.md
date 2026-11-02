# 参数说明

默认配置为 `src/fuel_gazebo_official_baseline/config/profiles/regression_C_final.yaml`。文件名沿用开发阶段命名，当前标准入口使用这一份配置。

## 规划与地图

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `exploration/vm`、`manager/max_vel`、`search/max_vel`、`optimization/max_vel`、`bspline/limit_vel` | 各0.6 m/s | 探索、搜索、优化和样条使用的速度配置 |
| `exploration/am`、`manager/max_acc`、`search/max_acc`、`optimization/max_acc`、`bspline/limit_acc` | 各0.6 m/s² | 对应环节的加速度配置 |
| `sdf_map/resolution` | 0.1 m | 地图分辨率 |
| `sdf_map/obstacles_inflation` | 0.35 m | 占据栅格膨胀 |
| `optimization/dist0` | 0.7 m | 轨迹优化的软距离目标 |
| `exploration/yd` | 约1.0472 rad/s | 名义 yaw 速度 |
| `exploration/ydd` | 约1.5708 rad/s² | 名义 yaw 加速度 |
| `astar/max_search_time` | 0.05 s | 观点连接搜索预算 |
| 飞行路径 A* 对象 | 1.0 s | 实现中设置的实际飞行搜索预算 |

速度、加速度和距离优化中的配置值不是对实测运动的硬保证。例如2026-11-01测试中，最高实测速度约1.259 m/s。完整结果见 [测试结果](VALIDATION.md)。

配置中的 `regression/historical_Y`、`historical_T`、`historical_Q`、`historical_G` 均启用，分别对应连续转向修正、候选状态恢复、周期角差和时间梯度修正。保留这些内部参数名是为了与现有实现一致。

## 执行检查

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `manager/execution_prefix_guard` | true | 启用执行前缀检查 |
| `manager/execution_check_dynamics` | true | 检查曲线内部动态量 |
| `manager/execution_check_horizon` | 2.1 s | 检查前方轨迹时长 |
| `manager/execution_tracking_margin` | 0.1 m | 地图距离余量 |
| `manager/execution_stop_guard_time` | 0.05 s | 限期截停提前量 |
| `fsm/safety_stop_before_hit` | true | 启用按轨迹编号和期限的截停 |
| 命令桥速度 / 加速度 / yaw rate 拒绝阈值 | 3 m/s / 6 m/s² / 3 rad/s | 检查速度、加速度向量范数和绝对 yaw rate |

0.1米是地图检查余量，不是实际跟踪误差上界。截停后目标保持也不保证机体立即停下。

## 相机与感知

| 环节 | 当前设置或观测值 |
| --- | --- |
| 渲染尺寸 | 640×480 |
| 水平 / 竖直视场 | 80° / 60° |
| 渲染 near / far | 0.1 / 5 m |
| 配置初始 fx / fy | 381.3611496301472 / 415.69219381653056 |
| 实际 `CameraInfo` fx / fy | 约381.3611602783203 / 415.6921863555908 |
| cx / cy | 320 / 240 |
| 深度过滤最小 / 最大距离 | 0.2 / 5 m |
| 最短 / 最大射线 | 0.5 / 4.5 m |
| 视点感知最大距离 | 4.5 m |
| 名义传感器更新率 | 15 Hz |

规划器使用实际 `CameraInfo` 完成相机参数设置，因此配置初始值与读回值存在很小的浮点差异。渲染、过滤、射线和视点参数作用不同，不能混成同一个量程。

`CameraInfo` 消息事件率不等于独立深度图帧率，不能仅凭消息头时间戳数量推断实际图像更新频率。
