#!/usr/bin/env bash
# Source this file in each terminal. Clear inherited FUEL overlays first.
fuel_entry_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROTORS_WS="${ROTORS_WS:-$HOME/rotors_ws}"
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
unset GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH GAZEBO_RESOURCE_PATH
source /opt/ros/noetic/setup.bash
source "$ROTORS_WS/devel/setup.bash"
if [[ -f "$fuel_entry_root/devel/setup.bash" ]]; then
  source "$fuel_entry_root/devel/setup.bash"
fi
export ROS_MASTER_URI="${FUEL_ROS_MASTER_URI:-http://localhost:11421}"
export GAZEBO_MASTER_URI="${FUEL_GAZEBO_MASTER_URI:-http://localhost:11422}"
export GAZEBO_PLUGIN_PATH="$fuel_entry_root/devel/lib:$ROTORS_WS/devel/lib"
export GAZEBO_MODEL_PATH="$fuel_entry_root/src/fuel_gazebo_official_world/models:$ROTORS_WS/src/rotors_simulator/rotors_gazebo/models:$ROTORS_WS/src/rotors_simulator/rotors_description/models"
export GAZEBO_RESOURCE_PATH="$ROTORS_WS/src/rotors_simulator/rotors_gazebo/models"
