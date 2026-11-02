# 当前世界与在线探索

Gazebo中的办公室/地面是完整物理几何，保留被测B的碰撞体与物理配置。FUEL通过真实模拟深度/CameraInfo/odom逐步更新内部地图；不将office.pcd预装成全部已探索地图。独立原SO3/map_generator演示不在当前入口。

当前B保留Y/T/Q/G和新增余量/限期截停，不能称官方零改动。FINISH语义未改，详细说明见[适配说明](../../docs/GAZEBO_ADAPTATION.md)和[验证结果](../../docs/VALIDATION.md)。
