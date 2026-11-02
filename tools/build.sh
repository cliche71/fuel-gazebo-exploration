#!/usr/bin/env bash
set -euo pipefail
fuel_build_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec env -i HOME="$HOME" USER="${USER:-ljz}" LOGNAME="${LOGNAME:-ljz}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  ROTORS_WS="${ROTORS_WS:-$HOME/rotors_ws}" \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/noetic/setup.bash
    source "$ROTORS_WS/devel/setup.bash"
    exec catkin_make -C "$1" -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2 -l2
  ' fuel-clean-build "$fuel_build_root"
