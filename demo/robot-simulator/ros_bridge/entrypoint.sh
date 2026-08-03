#!/usr/bin/env bash
set -Eeo pipefail

# ROS setup files reference optional variables before assigning them, so enable
# nounset only after the environment has been sourced.
source /opt/ros/humble/setup.bash
set -u
mkdir -p "${ROS_LOG_DIR:-/tmp/rovera/ros-logs}"

bridge_pid=""
mux_pid=""

stop_children() {
  [[ -z "$bridge_pid" ]] || kill -INT "$bridge_pid" 2>/dev/null || true
  [[ -z "$mux_pid" ]] || kill -INT "$mux_pid" 2>/dev/null || true
  [[ -z "$bridge_pid" ]] || wait "$bridge_pid" 2>/dev/null || true
  [[ -z "$mux_pid" ]] || wait "$mux_pid" 2>/dev/null || true
}
trap stop_children EXIT INT TERM

python3 /opt/rovera/ros_bridge/control_bridge.py &
bridge_pid=$!

if [[ "${ROS_USE_TWIST_MUX:-true}" == "true" ]]; then
  ros2 run twist_mux twist_mux --ros-args \
    --params-file /opt/rovera/ros_bridge/twist_mux.yaml \
    -r cmd_vel_out:=/cmd_vel &
  mux_pid=$!
fi

if [[ -n "$mux_pid" ]]; then
  wait -n "$bridge_pid" "$mux_pid"
else
  wait "$bridge_pid"
fi
