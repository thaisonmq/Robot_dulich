#!/usr/bin/env bash
set -eo pipefail
test -S "${NAVIGATION_SOCKET_PATH:-/var/lib/rovera/navigation/navigation.sock}"
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
set -u
ros2 node list --no-daemon 2>/dev/null | grep -qx /rovera_navigation_adapter
if [ "${REQUIRE_SENSOR_HEALTH:-1}" = "1" ]; then
  python3 - <<'PY'
import json
import os
import socket

path = os.environ.get("NAVIGATION_SOCKET_PATH", "/var/lib/rovera/navigation/navigation.sock")
client = socket.socket(socket.AF_UNIX)
client.settimeout(2.0)
client.connect(path)
client.sendall(b'{"command":"system.status","payload":{}}\n')
response = json.loads(client.recv(1_048_576))
state = response.get("state") or {}
required = ("scan_fresh", "odometry_ready", "lidar_tf_ready")
missing = [key for key in required if not state.get(key)]
if missing:
    raise SystemExit("ROS sensor gate failed: " + ", ".join(missing))
PY
fi
