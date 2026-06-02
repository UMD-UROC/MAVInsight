#!/usr/bin/env bash
set -euo pipefail

: "${ROS_DISTRO:=humble}"
: "${ROS_WS:=/ws}"
: "${MAVINSIGHT_LAUNCH:=launch_all_vehicles.launch.py}"
: "${START_UXRCE_AGENT:=1}"
: "${UXRCE_DDS_TRANSPORT:=udp4}"
: "${UXRCE_DDS_PORT:=8888}"
: "${FOXGLOVE_PORT:=8765}"

# Avoid nounset failures from generated setup scripts.
set +u
export PYTHONPATH="${PYTHONPATH:-/opt/ros/humble/lib/python3.10/site-packages}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if [ "${SKIP_COLCON_BUILD:-0}" != "1" ]; then
  cd "${ROS_WS}"
  colcon build --symlink-install
fi

set +u
source "${ROS_WS}/install/setup.bash"
set -u

pids=()

shutdown() {
  if [ "${#pids[@]}" -gt 0 ]; then
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}

if [ "${START_UXRCE_AGENT}" != "0" ]; then
  MicroXRCEAgent "${UXRCE_DDS_TRANSPORT}" -p "${UXRCE_DDS_PORT}" &
  pids+=("$!")
fi

ros2 launch mavinsight "${MAVINSIGHT_LAUNCH}" &
pids+=("$!")
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="${FOXGLOVE_PORT}" &
pids+=("$!")

trap shutdown SIGINT SIGTERM EXIT
wait -n "${pids[@]}"
