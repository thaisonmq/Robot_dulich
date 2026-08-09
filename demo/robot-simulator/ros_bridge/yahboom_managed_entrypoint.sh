#!/usr/bin/env bash
set -Eeo pipefail

source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash

joy_pid=""
ip_pid=""
bringup_pid=""
guard_pid=""
memory_fault_file="/tmp/rovera-yahboom-memory-fault"

[ ! -e "$memory_fault_file" ] || unlink "$memory_fault_file"

# Docker memory limits are ignored on some Raspberry Pi kernels. Bound each
# ROS process and also stop the container if their combined RSS runs away.
process_vmem_kb="${YAHBOOM_PROCESS_VMEM_KB:-0}"
container_rss_kb="${YAHBOOM_CONTAINER_RSS_KB:-921600}"
if [ "$process_vmem_kb" -gt 0 ]; then
  ulimit -v "$process_vmem_kb"
fi

stop_children() {
  [ -z "$guard_pid" ] || kill -TERM "$guard_pid" 2>/dev/null || true
  [ -z "$joy_pid" ] || kill -INT "$joy_pid" 2>/dev/null || true
  [ -z "$ip_pid" ] || kill -INT "$ip_pid" 2>/dev/null || true
  [ -z "$bringup_pid" ] || kill -INT "$bringup_pid" 2>/dev/null || true
  [ -z "$joy_pid" ] || wait "$joy_pid" 2>/dev/null || true
  [ -z "$ip_pid" ] || wait "$ip_pid" 2>/dev/null || true
  [ -z "$bringup_pid" ] || wait "$bringup_pid" 2>/dev/null || true
  [ -z "$guard_pid" ] || wait "$guard_pid" 2>/dev/null || true
}

handle_signal() {
  stop_children
  exit 143
}

trap stop_children EXIT
trap handle_signal INT TERM

memory_guard() {
  while sleep 0.5; do
    # A process may exit between glob expansion and awk opening its status
    # file. That normal /proc race must not terminate the guard/container.
    rss_kb="$(awk '/^VmRSS:/ { total += $2 } END { print total + 0 }' /proc/[0-9]*/status 2>/dev/null || true)"
    rss_kb="${rss_kb:-0}"
    if [ "$rss_kb" -gt "$container_rss_kb" ]; then
      echo >&2 "Yahboom memory guard: RSS ${rss_kb} KiB exceeded ${container_rss_kb} KiB; stopping managed runtime"
      ps -eo pid,rss,args --sort=-rss 2>/dev/null | head -n 12 >&2 || true
      kill -TERM "$joy_pid" "$ip_pid" "$bringup_pid" 2>/dev/null || true
      return 70
    fi
  done
}

# Preserve the two functions of the vendor container. Only the joystick's
# chassis output is remapped; /joy, /JoyState and /rpi5_ip remain available.
ros2 launch /opt/rovera/yahboomcar_joy_web_compatible.launch.py &
joy_pid=$!
python3 /root/publish_ip.py &
ip_pid=$!
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py &
bringup_pid=$!
memory_guard &
guard_pid=$!

set +e
wait -n "$joy_pid" "$ip_pid" "$bringup_pid" "$guard_pid"
runtime_status=$?
set -e

if [ "$runtime_status" -eq 70 ]; then
  stop_children
  touch "$memory_fault_file"
  echo >&2 "Yahboom runtime latched after a memory fault; container remains stopped internally to prevent a restart storm"
  while sleep 3600; do :; done
fi

exit "$runtime_status"
