#!/bin/sh
set -eu

device="${MICRO_ROS_SERIAL_DEVICE:-/dev/ttyUSB0}"
mode="${ROVERA_HARDWARE_MODE:-}"
lock_file="${ROVERA_SERIAL_LOCK_FILE:-/var/lock/rovera-micro-ros/ttyUSB0.lock}"
agent_vmem_kb="${MICRO_ROS_AGENT_VMEM_KB:-0}"

# A FastCDR abort can otherwise spend tens of seconds dumping a multi-GiB
# virtual address space, keeping the serial owner wedged while the MCU waits.
ulimit -c 0

# Fast DDS can reserve several GiB of virtual address space while its resident
# memory stays small. Capping address space makes the Agent throw std::bad_alloc
# during a healthy MCU session; after that crash the Yahboom firmware does not
# reconnect until it is reset. Keep the cap opt-in and rely on the container's
# resident-memory limit on hosts where memory cgroups are available.
if [ "$agent_vmem_kb" -gt 0 ]; then
  ulimit -v "$agent_vmem_kb"
fi

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
