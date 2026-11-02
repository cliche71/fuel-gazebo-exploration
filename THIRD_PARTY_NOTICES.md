# 来源、版权与许可证

本仓库保留官方FUEL的GNU GPL v3（根LICENSE及src/FUEL/LICENSE），固定来源为HKUST-Aerial-Robotics/FUEL 662dd23c7b52b258d3c4a0155ff6632118e8984f。原源码头、作者、包声明和许可证原样保留；新目录整理不把所有第三方组件重新授权。

- fuel_gazebo_official_baseline与办公室转换资源沿用随附GPL-3.0，包内LICENSE保留。办公室PCD及几何来源、原hash在world/manifest.json；147个碰撞盒及源资源未改。
- fuel_camera_adapter的源文件标记Apache-2.0，随附licenses/Apache-2.0.txt；原生Gazebo/OGRE/ROS库是外部依赖，保留各自条款。
- quadrotor_msgs由官方FUEL所带原消息/编码源码构建，原package.xml声明BSD，原注释与版权保留。
- LKH由官方FUEL的lkh_tsp_solver原源码构建，保留其原作者/来源注释及组件条款；根LICENSE不取代组件自己的许可。当前算法调用库接口，不调用历史硬编码路径的独立示例。
- Eigen、PCL、OpenCV、Boost、NLopt、ROS、Gazebo及RotorS/mav_comm按其各自许可证使用；依赖未拷入本仓库。Hummingbird可视网格从rotors_description提供，原机体物理参考URDF保留来源，不改控制/碰撞体。

删除出当前树的旧生成器、官方输入压缩包和过期例子仍可从外部Git bundle恢复。没有将第三方许可证、版权或归档中的失败证据当作缓存删除。精确文件身份见SOURCE_MAP.tsv、官方差异和DEPENDENCIES。
