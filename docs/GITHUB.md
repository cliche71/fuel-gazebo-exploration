# GitHub 仓库管理

本项目使用公开仓库：[cliche71/fuel-gazebo-exploration](https://github.com/cliche71/fuel-gazebo-exploration)。
仓库用于管理适配代码、场景、说明和公开验证摘要。本机完整备份、原始截图和 ROS 运行日志继续单独保存。

## 取得本项目

在 ROS 电脑上选择一个用于管理源码的目录，执行：

```bash
git clone https://github.com/cliche71/fuel-gazebo-exploration.git
cd fuel-gazebo-exploration
python3 tools/verify_repository.py
```

已有成功工作区无需覆盖或重建。新建独立工作区的步骤见 [使用说明](OPERATIONS.md)。
仓库与运行工作区是两个不同目录，准备程序会将适配包复制到 `~/fuel_official_ws/src`。

## 保存后续修改

如果只在 `~/fuel_official_ws/src` 改文件，这些修改不会自动出现在本仓库。
应核对并将有意保留的适配程序修改同步到本仓库同名位置，再检查和提交。
修改 FUEL 源码时，要维护可重复应用的补丁，而不是改动固定上游压缩包。

例如修改连接程序后，在仓库根目录执行：

```bash
python3 tools/verify_repository.py --refresh-delivery-record
git status --short
git diff
git add fuel_gazebo_official_baseline validation/delivery_file_hashes.json docs
git commit -m "Describe the concrete change and validation"
git push origin main
```

刷新文件记录只代表记录当前文件，不能代替编译或仿真飞行验证。
修改版本号时，同时维护根目录 `VERSION` 和适配包版本。

探索新方法时，从成功版本创建独立分支：

```bash
git switch -c experiment/new-method
```

完成验证后再合并到 main。每次提交只保留一个明确变化，并写清验证结果。

## 标记一个成功版本

需要标记版本时，在已提交且验证过的版本上执行：

```bash
git tag -a v0.1.3 -m "FUEL Gazebo delivered baseline; user simulation flight check passed"
git push origin v0.1.3
```

如果同名标签已经存在，先核对其内容，不覆盖旧标签。
标签标记的是仓库交付代码，不能替代用户电脑上实际成功源码的完整备份。

## 上传时的文件范围

保留官方 FUEL 原作者、来源、许可证和本项目改动说明。
仓库中的固定源码压缩包和场景点云是复现输入，应保留。
`build`、`devel`、个人凭据、本机完整备份、大型录像和 ROS 日志不加入仓库。

推送使用你已配置好的 GitHub 认证；如果尚未配置，可按
[GitHub CLI 登录说明](https://cli.github.com/manual/gh_auth_login) 使用浏览器登录：

```bash
gh auth login --hostname github.com --git-protocol https --web
```

本仓库已存在，无需再次运行创建同名仓库的命令。
