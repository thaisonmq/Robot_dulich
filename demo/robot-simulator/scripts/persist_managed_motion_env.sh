#!/usr/bin/env bash
set -Eeo pipefail

required_ack="I_ACCEPT_EXCLUSIVE_CMD_VEL_OWNERSHIP"
if [ "${ROVERA_EXCLUSIVE_CMD_VEL_ACK:-}" != "$required_ack" ]; then
  echo >&2 "Refusing managed-motion persistence: missing exact ROVERA_EXCLUSIVE_CMD_VEL_ACK"
  exit 77
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
env_file="$project_dir/.env"

if [ ! -f "$env_file" ]; then
  echo >&2 "Missing $env_file"
  exit 66
fi

backup_dir="$project_dir/backups/managed-motion-env-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup_dir"
cp "$env_file" "$backup_dir/.env"

tmp_file="$(mktemp "$project_dir/.env.managed-motion.XXXXXX")"
trap 'rm -f "$tmp_file"' EXIT

awk '
BEGIN {
  count = 10
  remove["ROS_LOCALHOST_ONLY"] = 1
  keys[1] = "MOTION_BACKEND"; values[keys[1]] = "ros2"
  keys[2] = "NAVIGATION_BACKEND"; values[keys[2]] = "ros2"
  keys[3] = "ROS_WEB_CMD_VEL_TOPIC"; values[keys[3]] = "/cmd_vel_web"
  keys[4] = "ROS_USE_TWIST_MUX"; values[keys[4]] = "false"
  keys[5] = "ROVERA_CONTROL_MODE"; values[keys[5]] = "managed-motion"
  keys[6] = "ROVERA_CMD_VEL_MODE"; values[keys[6]] = "exclusive"
  keys[7] = "ROVERA_EXCLUSIVE_CMD_VEL_ACK"; values[keys[7]] = "I_ACCEPT_EXCLUSIVE_CMD_VEL_OWNERSHIP"
  keys[8] = "MAPPING_MAX_FORWARD_SPEED"; values[keys[8]] = "0.18"
  keys[9] = "MAPPING_MAX_REVERSE_SPEED"; values[keys[9]] = "0.14"
  keys[10] = "MAPPING_MAX_ANGULAR_SPEED"; values[keys[10]] = "0.40"
}
{
  separator = index($0, "=")
  key = separator > 0 ? substr($0, 1, separator - 1) : ""
  if (key in values) {
    print key "=" values[key]
    seen[key] = 1
  } else if (key in remove) {
    next
  } else {
    print
  }
}
END {
  for (item = 1; item <= count; item++) {
    key = keys[item]
    if (!(key in seen)) print key "=" values[key]
  }
}
' "$env_file" >"$tmp_file"

chmod --reference="$env_file" "$tmp_file"
mv "$tmp_file" "$env_file"
trap - EXIT

echo "Managed-motion environment persisted; backup: $backup_dir/.env"
