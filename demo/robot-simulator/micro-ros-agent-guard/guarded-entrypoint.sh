#!/bin/sh
set -eu

device="${MICRO_ROS_SERIAL_DEVICE:-/dev/ttyUSB0}"
mode="${ROVERA_HARDWARE_MODE:-}"
lock_file="${ROVERA_SERIAL_LOCK_FILE:-/var/lock/rovera-micro-ros/ttyUSB0.lock}"
agent_vmem_kb="${MICRO_ROS_AGENT_VMEM_KB:-1572864}"

# Some Raspberry Pi kernels do not mount the memory cgroup, so Docker ignores
# mem_limit. Keep an Agent/DDS fault bounded without relying on the host.
ulimit -v "$agent_vmem_kb"

case "$mode" in
  legacy)
    ;;
  managed)
    if [ "${ROVERA_MANAGED_HARDWARE_ACK:-}" != "I_ACCEPT_EXCLUSIVE_SERIAL_OWNERSHIP" ]; then
      echo "Refusing managed micro-ROS: set ROVERA_MANAGED_HARDWARE_ACK=I_ACCEPT_EXCLUSIVE_SERIAL_OWNERSHIP after disabling the legacy Agent." >&2
      exit 78
    fi
    ;;
  *)
    echo "Refusing micro-ROS: ROVERA_HARDWARE_MODE must be legacy or managed." >&2
    exit 78
    ;;
esac

if [ ! -c "$device" ]; then
  echo "Refusing micro-ROS: serial device $device is not ready." >&2
  exit 75
fi

# Compose uses the host PID namespace so fuser can also see an Agent launched
# by the legacy desktop script. Refuse before either process consumes XRCE.
owners=""
for process in /proc/[0-9]*; do
  for descriptor in "$process"/fd/*; do
    if [ "$(readlink "$descriptor" 2>/dev/null || true)" = "$device" ]; then
      owners="$owners ${process#/proc/}"
      break
    fi
  done
done
if [ -n "$owners" ]; then
  echo "Refusing micro-ROS: $device is already owned by PID(s):$owners" >&2
  exit 73
fi

mkdir -p "$(dirname "$lock_file")"
exec 9>"$lock_file"
if ! flock -n 9; then
  echo "Refusing micro-ROS: project serial lock is already held: $lock_file" >&2
  exit 73
fi

exec /bin/sh /micro-ros_entrypoint.sh "$@"
