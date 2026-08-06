#!/usr/bin/env bash
set -Eeo pipefail

required_ack="I_ACCEPT_EXCLUSIVE_CMD_VEL_OWNERSHIP"
if [[ "${ROVERA_CONTROL_MODE:-}" != "managed-motion" ]] || \
   [[ "${ROVERA_EXCLUSIVE_CMD_VEL_ACK:-}" != "$required_ack" ]]; then
  echo >&2 "control bridge refused to start outside the managed-motion cutover"
  exit 78
fi
if [[ "${ROS_WEB_CMD_VEL_TOPIC:-}" != "/cmd_vel_web" ]] || \
   [[ "${ROS_USE_TWIST_MUX:-}" != "false" ]]; then
  echo >&2 "control bridge refused unsafe routing: Web must publish /cmd_vel_web and the safety stack owns the only mux"
  exit 78
fi

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

if [[ "${ROS_USE_TWIST_MUX:-false}" == "true" ]]; then
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
