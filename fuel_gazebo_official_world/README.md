# FUEL 官方办公室场景 → Gazebo Classic

这个包只提供官方 `office.pcd` 对应的办公室场景。不会启动 FUEL、修改 FUEL 源码、发送飞行指令或把完整地图写入探索地图。当前只检查场景显示、碰撞对象及相机观测。起点附近没有被相机看见的问题，仍需在后续相机与启动设置对齐后验证；安装本包不代表探索已经能飞。

来源：[官方 FUEL，固定版本 662dd23](https://github.com/HKUST-Aerial-Robotics/FUEL/tree/662dd23c7b52b258d3c4a0155ff6632118e8984f)。原文件为 `uav_simulator/map_generator/resource/office.pcd`，207,840 个点；本包保留原文件与原 LICENSE。默认起点 `(0,0,1)`，官方探索范围 `x∈[-10,10], y∈[-15,15], z∈[0,2]`，单位米。场景实际点云略超出官方探索范围，未裁剪。

原文件是点云，不是 Gazebo 场景。转换时用 0.1 米网格把点变成有厚度的实体，再合并为 147 个长方体。世界坐标、大小比例与方向保持不变，没有补洞或向空白区域扩张网格。每个原点都包含在转换后的实体里；但厚度与点的位置存在离散差异，因此不能称实体表面与原点云完全重合。每个点到所属网格中心的单轴偏差不超过 0.05 米，实体还会从中心向各轴延伸 0.05 米。保守的实体点到原始采样点单轴距离上界为 0.1 米。147 个长方体与所选网格的占据范围逐格完全一致。

网格化可能改变细小缝隙，不能声称与原始点云的所有细节完全相同。以统一厚度作离线对照，在预留 0.1、0.175、0.35 米通行余量时，没有发现起点原本可到达的有效区域被切断；实际门洞仍须在 Gazebo 检查。

## 放入工作空间并检查

解压后将 `fuel_gazebo_official_world` 文件夹放到 `~/fuel_ws/src/`。已存在这个同名文件夹时，先另存旧文件夹，不要混合覆盖。

这一步不用重新编译 FUEL：

```bash
source ~/fuel_ws/devel/setup.bash
python3 ~/fuel_ws/src/fuel_gazebo_official_world/scripts/check_scene.py
```

应出现 `OFFICIAL OFFICE OFFLINE CHECK PASSED`。检查包括：原始文件校验、207,840 个原点的位置、202,920 个占据网格、147 个长方体覆盖完全一致、显示与碰撞尺寸相同，以及 `(0,0,1)` 到 `(0,0,2)` 的竖直路径有 0.35 米余量。这个检查在离线完成，不代替实际 Gazebo 检查与飞行测试。

## 已有 Gazebo 悬停场景：装入办公室

关闭导航与探索的启动终端，只保留原来已验证的 Gazebo 和悬停控制。让 hummingbird 在 `(0,0,1)` 附近稳定悬停。不要发送探索开始命令。

```bash
source ~/fuel_ws/devel/setup.bash
python3 ~/fuel_ws/src/fuel_gazebo_official_world/scripts/spawn_office.py
```

脚本要求无人机水平位置距原点不超过 0.25 米，高度 0.6–2.0 米，线速度不超过 0.15 米/秒、角速度不超过 0.3 弧度/秒。脚本只接受这些已知模型名：`ground_plane`、`sun`、`hummingbird`、`depth_test_box`、`fuel_official_office`；发现其他模型或仍运行导航/探索节点时会停止。

办公室成功添加并读回确认后，才会删除旧的 `depth_test_box`。现有地面和无人机保持原样。中途检查失败时，只撤销本次新建的办公室；不会删除原来已存在的办公室。重复执行会核对同名办公室的静态属性、标识、碰撞对象和位置，不会重复添加或覆盖不一致对象。

此命令不要求另开 Gazebo；不要同时运行下面的预览命令。

## 收集当前配置

保持悬停仿真运行，另开终端执行：

```bash
source ~/fuel_ws/devel/setup.bash
python3 ~/fuel_ws/src/fuel_gazebo_official_world/scripts/collect_inputs.py
```

脚本只读取当前无人机、相机和启动设置，生成 `~/fuel_official_inputs_日期_时间.zip`。
请提供这个 ZIP 及装入办公室时的终端输出，用于恢复官方流程和核对相机设置。

## 独立预览：只看办公室（可选）

只有当前没有其他 Gazebo 运行时，才使用：

```bash
source ~/fuel_ws/devel/setup.bash
cd ~/fuel_ws
catkin_make -j2 -l2
source ~/fuel_ws/devel/setup.bash
roslaunch fuel_gazebo_official_world world.launch
```

预览不包含无人机。光源和地面在场景内定义，不需要在线下载模型。地面保持 Gazebo 原场景的 `z=0`，用于物理仿真；它不是从 `office.pcd` 中转换出的整片地板。

## 当前需要确认的结果

- Gazebo 显示办公室墙体和走廊；原测试箱消失。
- 无人机仍在原点附近悬停，没有发出自主探索指令。
- 深度图能看到墙体，时间正常前进。

完整办公室只是仿真世界中的真实环境。后续 FUEL 仍必须通过相机逐步建立自己的地图。当前包没有改变相机安装位置，也没有恢复官方的启动升降动作或连续探索流程。官方地图转换和这些设置对齐是不同的检查，不能用更换场景代替后者。

`manifest.json` 记录来源版本、校验值、转换规则、147 个长方体范围和离线检查约束。当前环境没有 ROS / Gazebo，交付前完成的是离线几何与脚本检查；实际显示、相机与碰撞仍需在用户的 Gazebo 中确认。

关于官方后退、启动动作及当前版本差异，见 `OFFICIAL_BEHAVIOR.md`。
