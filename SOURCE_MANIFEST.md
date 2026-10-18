# 源码与资源来源

| 内容 | 来源与版本 |
| --- | --- |
| FUEL | [HKUST-Aerial-Robotics/FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) |
| 固定 FUEL 提交 | `662dd23c7b52b258d3c4a0155ff6632118e8984f` |
| 固定 Git tree | `4acfe443f46226497d9543ac0905d908505a9500` |
| FUEL 原始文件 | `fuel_gazebo_official_baseline/vendor/FUEL-662dd23.tar.gz`，1,527 个文件 |
| 压缩包 SHA-256 | `9e0115042c0cb6976ceef794ef79118f46872645e4e8928f2452add7c2efd86a` |
| 原始文件记录 | `fuel_gazebo_official_baseline/vendor/upstream_files.json`，记录每个文件的 SHA-256、Git blob 和模式 |
| Gazebo 适配包 | `fuel_gazebo_official_baseline`，版本 0.1.3 |
| Gazebo 场景包 | `fuel_gazebo_official_world`，版本 0.1.0；场景没有因转向修正改版 |
| 办公室输入 | 官方 `uav_simulator/map_generator/resource/office.pcd` |
| 办公室输入 SHA-256 | `f81065db52a6d83ae6d9bb357f321e45327f59eb7105cd005d18e8a1ba3f2336` |
| RotorS | 用户电脑的 `~/rotors_ws`；本仓库不声称已记录其精确提交 |

两个包复制自 0.1.3 交付内容，保留原有程序与资源。此次仓库整理只增加说明、备份工具和文件检查工具，
并在历史说明开头增加后续实测记录入口。公开时去除了模型自动生成注释中的本机目录；
没有改动飞行程序、参数、模型物理内容、上游源码压缩包或修改记录。

FUEL 压缩包保留固定提交中完整的所选源码与资源目录，只省略官方展示动图。
其中包含上游历史提交的构建缓存或旧路径文本；为保持逐文件校验，它们仍保留在原始压缩包内。
准备工作区时会先核对原始文件，再应用四组已记录的修改。
不要将该固定输入替换成用户工作目录或未记录的新版本。

用户电脑上实际通过飞行的最终源码，以用户已校验的本机成功备份为准。
当前仓库尚未取得该备份的完整内容，不能声称它是该电脑的逐文件镜像。

## 作者与许可证

保留官方 FUEL 的 GNU GPL v3 许可证，以及源码、模型和依赖中原有的第三方版权与许可证声明。
Hummingbird 参考描述来自用户已成功运行的 RotorS 模型，保留其原有声明；运行时网格使用 `package://rotors_description/...`。
参考描述的自动生成注释已去除原电脑目录，运行时网格引用保持不变。

本项目的新增说明解释适配与运行过程，不把官方 FUEL、RotorS 或办公室资源归为原创成果。
