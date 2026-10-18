#!/usr/bin/env bash
# Back up the successful FUEL/RotorS source and configuration on the ROS computer.
# Does not stop ROS or modify the workspaces. Recovery requires rebuilding them.
set -euo pipefail
umask 077

fuel_backup_home="${FUEL_BACKUP_HOME:-$HOME}"
fuel_backup_home="$(cd -- "$fuel_backup_home" && pwd -P)"
fuel_backup_root="$fuel_backup_home/fuel_gazebo_backups"
fuel_backup_stage=""
fuel_backup_complete=0

fuel_backup_cleanup() {
  local result=$?
  if [[ "$fuel_backup_complete" -eq 0 && -n "$fuel_backup_stage" && -d "$fuel_backup_stage" ]]; then
    rm -rf -- "$fuel_backup_stage"
  fi
  if [[ "$result" -ne 0 ]]; then
    printf '\n备份未完成；工作目录中的原文件未改动。\n' >&2
  fi
}
trap fuel_backup_cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for fuel_backup_tool in tar gzip sha256sum du df awk mktemp; do
  command -v "$fuel_backup_tool" >/dev/null || {
    printf '缺少命令：%s\n' "$fuel_backup_tool" >&2
    exit 1
  }
done

fuel_backup_items=(
  "fuel_official_ws/src"
  "rotors_ws/src"
  "fuel_official_ws/fuel_official_baseline_manifest.json"
  ".ros/fuel_official_baseline/robot.urdf"
)
for fuel_backup_item in "${fuel_backup_items[@]}"; do
  if [[ ! -e "$fuel_backup_home/$fuel_backup_item" ]]; then
    printf '未找到需要备份的文件：%s\n' "$fuel_backup_home/$fuel_backup_item" >&2
    exit 1
  fi
done
[[ -d "$fuel_backup_home/fuel_official_ws/src" && -d "$fuel_backup_home/rotors_ws/src" ]] || {
  printf 'FUEL 和 RotorS 的 src 必须是目录。\n' >&2
  exit 1
}
if [[ -f "$fuel_backup_home/.ros/fuel_official_baseline/robot.json" ]]; then
  fuel_backup_items+=(".ros/fuel_official_baseline/robot.json")
fi
shopt -s nullglob
for fuel_backup_report in "$fuel_backup_home"/fuel_official_*.txt; do
  [[ -f "$fuel_backup_report" ]] && fuel_backup_items+=("${fuel_backup_report#"$fuel_backup_home/"}")
done
shopt -u nullglob

mkdir -p -- "$fuel_backup_root"
fuel_backup_kb=0
for fuel_backup_item in "${fuel_backup_items[@]}"; do
  fuel_backup_size="$(du -sk -- "$fuel_backup_home/$fuel_backup_item" | awk '{print $1}')"
  fuel_backup_kb=$((fuel_backup_kb + fuel_backup_size))
done
fuel_backup_free="$(df -Pk -- "$fuel_backup_root" | awk 'END {print $4}')"
fuel_backup_need=$((fuel_backup_kb + fuel_backup_kb / 5 + 131072))
if (( fuel_backup_free < fuel_backup_need )); then
  printf '空间不足：建议至少预留 %s MB。\n' "$((fuel_backup_need / 1024 + 1))" >&2
  exit 1
fi

fuel_backup_stage="$(mktemp -d "$fuel_backup_root/.incomplete_XXXXXXXX")"
fuel_backup_final="$fuel_backup_root/official_working_$(date +%Y%m%d_%H%M%S)_${fuel_backup_stage##*_}"
printf '正在备份源码、地图、相机配置及检查报告……\n'
printf '%s\n' "${fuel_backup_items[@]}" > "$fuel_backup_stage/FILES.txt"
tar -czf "$fuel_backup_stage/working_source.tar.gz" \
  -C "$fuel_backup_home" -- "${fuel_backup_items[@]}"

printf '正在检查压缩包，并与原文件比较……\n'
gzip -t "$fuel_backup_stage/working_source.tar.gz"
tar -dzf "$fuel_backup_stage/working_source.tar.gz" -C "$fuel_backup_home"
cat > "$fuel_backup_stage/RESTORE.txt" <<'TEXT'
FUEL + Gazebo 成功版本备份

备份内容见 FILES.txt，保留了两个工作区的全部 src、官方来源记录、
生成的相机模型，以及运行过的官方检查报告。源码中的链接按原样保留。
build、devel、安装的软件、飞行录像及 ROS 日志不包含在内。

恢复前停止 FUEL 和 Gazebo，保留当前工作区副本。
在本备份目录先执行：
  sha256sum -c SHA256SUMS
然后将 working_source.tar.gz 解压到一个新建的空目录检查内容。
压缩包内的路径相对于原用户主目录。
不要直接解压覆盖仍在使用的工作区。
放回对应目录后，需要在原 ROS Noetic 环境重新编译 RotorS 和 FUEL。
TEXT
(
  cd -- "$fuel_backup_stage"
  sha256sum working_source.tar.gz FILES.txt RESTORE.txt > SHA256SUMS
  sha256sum -c SHA256SUMS
)
mv -- "$fuel_backup_stage" "$fuel_backup_final"
fuel_backup_complete=1
printf '\n备份检查通过：%s/working_source.tar.gz\n' "$fuel_backup_final"
printf '恢复说明：%s/RESTORE.txt\n' "$fuel_backup_final"
