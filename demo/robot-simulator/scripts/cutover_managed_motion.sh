#!/usr/bin/env bash
set -Eeo pipefail

required_ack="I_ACCEPT_EXCLUSIVE_CMD_VEL_OWNERSHIP"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
cd "$project_dir"

apply=0
if [ "${1:-}" = "--apply" ]; then
  apply=1
elif [ "$#" -ne 0 ]; then
  echo >&2 "Usage: $0 [--apply]"
  exit 64
fi

if [ ! -f .env ]; then
  echo >&2 "Missing $project_dir/.env"
  exit 66
fi

for service in micro-ros-agent motion-safety ros-control-bridge yahboom-joystick navigation-stack mapping-stack; do
  if docker ps -q --filter "label=com.docker.compose.service=$service" | grep -q .; then
    echo >&2 "Refusing cutover: stop Rovera service first: $service"
    exit 73
  fi
done
edge_was_running=0
if docker ps -q --filter "label=com.docker.compose.service=robot-simulator" | grep -q .; then
  edge_was_running=1
fi

agent_containers=()
legacy_joystick_containers=()
while IFS= read -r container_id; do
  [ -n "$container_id" ] || continue
  processes="$(docker top "$container_id" -eo pid,args 2>/dev/null || true)"
  if grep -E -q 'micro_ros_agent.+serial.+/dev/ttyUSB0' <<<"$processes"; then
    agent_containers+=("$container_id")
  fi
  if grep -q 'yahboomcar_joy_launch.py' <<<"$processes"; then
    legacy_joystick_containers+=("$container_id")
  fi
done < <(docker ps -q)

if [ "${#agent_containers[@]}" -ne 1 ]; then
  echo >&2 "Refusing cutover: expected one existing serial Agent, found ${#agent_containers[@]}"
  exit 73
fi
if [ "${#legacy_joystick_containers[@]}" -ne 1 ]; then
  echo >&2 "Refusing cutover: expected one vendor joystick container, found ${#legacy_joystick_containers[@]}"
  exit 73
fi

legacy_joystick="${legacy_joystick_containers[0]}"

graph_check=""
if ! graph_check="$(docker exec -i -e ROS_DOMAIN_ID=20 "$legacy_joystick" bash -lc \
  'source /opt/ros/humble/setup.bash >/dev/null 2>&1; python3 -' <<'PY'
import time
import rclpy
from rclpy.node import Node

rclpy.init()
node = Node("rovera_preflight_probe")
deadline = time.monotonic() + 4
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)

topics = {name for name, _types in node.get_topic_names_and_types()}
required = {"/scan", "/imu", "/odom_raw"}
missing = sorted(required - topics)
cmd_publishers = [
    item.node_name for item in node.get_publishers_info_by_topic("/cmd_vel")
]
errors = []
if missing:
    errors.append(f"sensor topics missing: {missing}")
if len(cmd_publishers) != 1 or cmd_publishers[0] not in {"joy_ctrl", "yahboom_joy"}:
    errors.append(f"unexpected /cmd_vel publishers: {cmd_publishers}")

print("ROS preflight OK" if not errors else "ROS preflight ERROR: " + "; ".join(errors))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if not errors else 1)
PY
)"; then
  echo >&2 "$graph_check"
  echo >&2 "Refusing cutover before any container is changed"
  exit 73
fi

operator_home="$(getent passwd "$(id -un)" | cut -d: -f6)"
vendor_autostart="$operator_home/.config/autostart/uros.desktop"
vendor_autostart_disabled="$operator_home/.config/autostart/uros.desktop.rovera-disabled"
if [ ! -f "$vendor_autostart" ] || \
   ! grep -q '/home/pi/ros2_humble.sh' "$vendor_autostart"; then
  echo >&2 "Refusing cutover: the vendor joystick autostart was not found at the expected path"
  exit 73
fi
if [ -e "$vendor_autostart_disabled" ]; then
  echo >&2 "Refusing cutover: autostart backup already exists: $vendor_autostart_disabled"
  exit 73
fi
echo "Preflight passed. Legacy sensor Agent will be preserved: ${agent_containers[0]}"
echo "$graph_check"
echo "Vendor joystick selected for reversible replacement: $legacy_joystick"
echo "Vendor desktop autostart will be disabled during apply: $vendor_autostart"
if [ "$edge_was_running" -eq 1 ]; then
  echo "The existing Robot Edge will stay online during build and be recreated only after safety is ready."
fi

if [ "$apply" -eq 0 ]; then
  echo "CHECK ONLY: no container was changed."
  echo "To apply, set ROVERA_EXCLUSIVE_CMD_VEL_ACK=$required_ack and rerun with --apply."
  exit 0
fi
if [ "${ROVERA_EXCLUSIVE_CMD_VEL_ACK:-}" != "$required_ack" ]; then
  echo >&2 "Refusing apply: missing exact ROVERA_EXCLUSIVE_CMD_VEL_ACK"
  exit 77
fi

edge_compose=(docker compose --env-file .env -f compose.yaml)
navigation_compose=(docker compose --env-file .env -f compose.navigation.yml)
mapping_compose=(
  docker compose
  --env-file .env
  -f compose.yaml
  -f compose.coexistence.yml
  --profile legacy-coexistence
)

# Finish all builds before touching the known-good vendor process.
"${edge_compose[@]}" --profile managed-motion build robot-simulator ros-control-bridge
"${navigation_compose[@]}" --profile managed-motion build motion-safety

rollback_needed=0
rollback() {
  status=$?
  if [ "$rollback_needed" -eq 1 ]; then
    echo >&2 "Managed-motion verification failed; restoring the vendor joystick."
    "${edge_compose[@]}" stop robot-simulator ros-control-bridge yahboom-joystick >/dev/null 2>&1 || true
    "${mapping_compose[@]}" stop mapping-stack >/dev/null 2>&1 || true
    ROVERA_CMD_VEL_MODE=exclusive \
      ROVERA_EXCLUSIVE_CMD_VEL_ACK="$required_ack" \
      "${navigation_compose[@]}" stop motion-safety >/dev/null 2>&1 || true
    docker start "$legacy_joystick" >/dev/null 2>&1 || true
    if [ "$edge_was_running" -eq 1 ]; then
      "${edge_compose[@]}" up -d robot-simulator >/dev/null 2>&1 || true
    fi
    if [ -f "$vendor_autostart_disabled" ] && [ ! -e "$vendor_autostart" ]; then
      mv "$vendor_autostart_disabled" "$vendor_autostart" || true
    fi
  fi
  exit "$status"
}
trap rollback ERR INT TERM

rollback_needed=1
mv "$vendor_autostart" "$vendor_autostart_disabled"
docker stop "$legacy_joystick" >/dev/null

ROVERA_CMD_VEL_MODE=exclusive \
  ROVERA_EXCLUSIVE_CMD_VEL_ACK="$required_ack" \
  "${navigation_compose[@]}" --profile managed-motion up -d motion-safety

MOTION_BACKEND=ros2 \
  NAVIGATION_BACKEND=ros2 \
  ROVERA_CONTROL_MODE=managed-motion \
  ROVERA_EXCLUSIVE_CMD_VEL_ACK="$required_ack" \
  ROS_DOMAIN_ID=20 \
  ROS_USE_TWIST_MUX=false \
  ROS_WEB_CMD_VEL_TOPIC=/cmd_vel_web \
  ROS_SAFETY_CMD_VEL_TOPIC=/cmd_vel_safety \
  "${edge_compose[@]}" --profile managed-motion up -d \
    yahboom-joystick ros-control-bridge robot-simulator

ROVERA_USE_VENDOR_BASE_RUNTIME=1 \
  "${mapping_compose[@]}" up -d mapping-stack

sleep 6
safety_id="$(docker ps -q --filter label=com.docker.compose.service=motion-safety)"
if [ -z "$safety_id" ]; then
  echo >&2 "motion-safety did not stay running"
  false
fi

graph_result=""
if ! graph_result="$(docker exec -i \
    -e ROS_DOMAIN_ID=20 \
    "$safety_id" bash -lc \
    'source /opt/ros/humble/setup.bash >/dev/null 2>&1; source /ws/install/setup.bash >/dev/null 2>&1; python3 -' <<'PY'
import time
import rclpy
from rclpy.node import Node

rclpy.init()
node = Node("rovera_cutover_probe")
deadline = time.monotonic() + 5
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)

publishers = [item.node_name for item in node.get_publishers_info_by_topic("/cmd_vel")]
subscribers = [item.node_name for item in node.get_subscriptions_info_by_topic("/cmd_vel")]
joy_publishers = [item.node_name for item in node.get_publishers_info_by_topic("/cmd_vel_joy")]
obstacle_subscribers = [
    item.node_name
    for item in node.get_subscriptions_info_by_topic("/rovera/obstacle_directions")
]
topics = {name for name, _types in node.get_topic_names_and_types()}
required_topics = {"/scan", "/imu", "/odom_raw"}

errors = []
if publishers != ["rovera_motion_safety"]:
    errors.append(f"/cmd_vel publishers={publishers}")
if "YB_Car_Node" not in subscribers:
    errors.append(f"YB_Car_Node is not subscribed to /cmd_vel: {subscribers}")
if len(joy_publishers) != 1:
    errors.append(f"expected exactly one managed /cmd_vel_joy publisher: {joy_publishers}")
if "rovera_motion_safety" not in obstacle_subscribers:
    errors.append(f"global obstacle interlock missing: {obstacle_subscribers}")
missing = sorted(required_topics - topics)
if missing:
    errors.append(f"legacy sensor topics missing: {missing}")

print("OK" if not errors else "ERROR: " + "; ".join(errors))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if not errors else 1)
PY
)"; then
  echo >&2 "Managed-motion graph verification failed: ${graph_result:-probe process failed without output}"
  false
fi
echo "$graph_result"

ROVERA_EXCLUSIVE_CMD_VEL_ACK="$required_ack" \
  "$script_dir/persist_managed_motion_env.sh"

rollback_needed=0
trap - ERR INT TERM
echo "Managed motion is active: motion-safety is the sole /cmd_vel publisher."
echo "The guarded legacy micro-ROS Agent and the legacy sensor topic names were preserved."
echo "Vendor direct-/cmd_vel desktop autostart is disabled at: $vendor_autostart_disabled"
