# 项目来源

## FUEL

本项目基于 [HKUST-Aerial-Robotics/FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)，对应官方提交：

```text
662dd23c7b52b258d3c4a0155ff6632118e8984f
```

保留原前沿搜索、全局访问顺序、局部观点选择、轨迹规划和探索结束流程，在此基础上加入 Gazebo / RotorS 接口、数值及状态处理修正、相机校准、参数适配和执行余量检查。

实际改动按类别说明在 [Gazebo 移植说明](GAZEBO_ADAPTATION.md)。当前默认参数见 [参数说明](PARAMETERS.md)，仿真结果见 [测试结果](VALIDATION.md)。

## 仿真与运行支撑

- [RotorS](https://github.com/ethz-asl/rotors_simulator)：Hummingbird 模型、Gazebo 飞行器动力学和控制器。
- [mav_comm](https://github.com/ethz-asl/mav_comm)：飞行器控制和状态相关消息。
- Gazebo Classic / ROS Noetic：场景、传感器、进程通信及运行环境。
- 仓库中的 `fuel_camera_adapter`：相机投影、接触和种子读取插件。
- 办公室 Gazebo 几何由官方场景点云转换，保留原资源来源。

具体版本和构建要求见 [依赖说明](DEPENDENCIES.md)。

## 许可证与引用

本仓库保留官方 FUEL 的 GNU GPL v3 许可证，以及各文件原有的版权和第三方声明。详见根目录 [LICENSE](../LICENSE) 和 [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)。

研究使用本项目时，请引用原 FUEL 工作；论文及 BibTeX 见 [官方项目说明](https://github.com/HKUST-Aerial-Robotics/FUEL)。Gazebo 接口和配置适配不替代上游算法的署名与引用。
