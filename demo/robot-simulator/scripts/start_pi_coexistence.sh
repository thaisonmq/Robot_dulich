#!/usr/bin/env bash
set -Eeo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
cd "$project_dir"

if ! command -v docker >/dev/null 2>&1; then
  echo >&2 "Docker is required"
  exit 69
fi
if [ ! -f .env ]; then
  echo >&2 "Missing $project_dir/.env (copy edge.env.example and add the existing robot credentials)"
  exit 66
fi

forbidden_services=(micro-ros-agent ros-control-bridge motion-safety navigation-stack yahboom-joystick)
for service in "${forbidden_services[@]}"; do
  if docker ps -q --filter "label=com.docker.compose.service=$service" | grep -q .; then
    echo >&2 "Refusing coexistence start: conflicting service is running: $service"
    exit 73
  fi
done

agent_containers=()
while IFS= read -r container_id; do
  [ -n "$container_id" ] || continue
  if docker top "$container_id" -eo pid,args 2>/dev/null \
      | grep -E -q 'micro_ros_agent.+serial.+/dev/ttyUSB0'; then
    agent_containers+=("$container_id")
  fi
done < <(docker ps -q)
if [ "${#agent_containers[@]}" -ne 1 ]; then
  echo >&2 "Refusing start: expected exactly one existing micro-ROS Agent for /dev/ttyUSB0, found ${#agent_containers[@]}"
  exit 73
fi

legacy_joystick_count=0
legacy_joystick_container=""
vendor_base_runtime=0
while IFS= read -r container_id; do
  [ -n "$container_id" ] || continue
  if docker top "$container_id" -eo pid,args 2>/dev/null \
      | grep -q 'yahboomcar_joy_launch.py'; then
    legacy_joystick_count=$((legacy_joystick_count + 1))
    legacy_joystick_container="$container_id"
    if docker top "$container_id" -eo pid,args 2>/dev/null \
        | grep -q 'yahboomcar_bringup_launch.py'; then
      vendor_base_runtime=1
    fi
  fi
done < <(docker ps -q)
if [ "$legacy_joystick_count" -ne 1 ]; then
  echo >&2 "Refusing start: expected one vendor joystick runtime, found $legacy_joystick_count"
  exit 73
fi

graph_check=""
if ! graph_check="$(docker exec -i -e ROS_DOMAIN_ID=20 "$legacy_joystick_container" bash -lc \
  'source /opt/ros/humble/setup.bash >/dev/null 2>&1; python3 -' <<'PY'
import time
import rclpy
from rclpy.node import Node

rclpy.init()
node = Node("rovera_coexistence_probe")
deadline = time.monotonic() + 4
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
topics = {name for name, _types in node.get_topic_names_and_types()}
missing = sorted({"/scan", "/imu", "/odom_raw"} - topics)
print("ROS sensor preflight OK" if not missing else f"ROS sensor preflight ERROR: missing {missing}")
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if not missing else 1)
PY
)"; then
  echo >&2 "$graph_check"
  echo >&2 "Refusing mapping start before any container is changed"
  exit 73
fi

compose=(
  docker compose
  --env-file .env
  -f compose.yaml
  -f compose.coexistence.yml
  --profile legacy-coexistence
)

echo "Preflight passed: preserving the existing serial Agent and vendor joystick."
echo "$graph_check"
echo "Starting only read-only edge + mapping services; Web chassis motion is disabled."
ROVERA_USE_VENDOR_BASE_RUNTIME="$vendor_base_runtime" \
  "${compose[@]}" up -d --build robot-simulator mapping-stack

"${compose[@]}" ps robot-simulator mapping-stack
echo "Coexistence mode is active. No service in this mode opens /dev/ttyUSB0 or publishes /cmd_vel."
