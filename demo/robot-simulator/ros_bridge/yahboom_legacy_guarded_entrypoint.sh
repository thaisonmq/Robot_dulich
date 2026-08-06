#!/usr/bin/env bash
set -Eeo pipefail

# Raspberry Pi images that do not mount the memory cgroup silently ignore
# Docker's mem_limit. RLIMIT_AS stops a single Fast DDS process from growing
# into several GiB, while the RSS guard protects against several processes
# growing together.
process_vmem_kb="${YAHBOOM_PROCESS_VMEM_KB:-1048576}"
container_rss_kb="${YAHBOOM_CONTAINER_RSS_KB:-921600}"

ulimit -v "$process_vmem_kb"

vendor_pid=""
guard_pid=""

stop_children() {
  [ -z "$guard_pid" ] || kill -TERM "$guard_pid" 2>/dev/null || true
  [ -z "$vendor_pid" ] || kill -TERM "$vendor_pid" 2>/dev/null || true
  [ -z "$guard_pid" ] || wait "$guard_pid" 2>/dev/null || true
  [ -z "$vendor_pid" ] || wait "$vendor_pid" 2>/dev/null || true
}
trap stop_children EXIT INT TERM

memory_guard() {
  while sleep 0.5; do
    # A process may exit between glob expansion and awk opening its status
    # file. That normal /proc race must not terminate the guard/container.
    rss_kb="$(awk '/^VmRSS:/ { total += $2 } END { print total + 0 }' /proc/[0-9]*/status 2>/dev/null || true)"
    rss_kb="${rss_kb:-0}"
    if [ "$rss_kb" -gt "$container_rss_kb" ]; then
      echo >&2 "Yahboom memory guard: RSS ${rss_kb} KiB exceeded ${container_rss_kb} KiB; stopping vendor runtime"
      kill -TERM "$vendor_pid" 2>/dev/null || true
      return 70
    fi
  done
}

# /root/1.sh ends in `exec bash` and therefore exits immediately without an
# interactive TTY. Start the same supervisor config in the foreground so the
# hardware runtime is deployable from Compose/systemd as well as a terminal.
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
/usr/bin/supervisord -n -c /etc/supervisor/supervisord.conf &
vendor_pid=$!
memory_guard &
guard_pid=$!

set +e
wait -n "$vendor_pid" "$guard_pid"
status=$?
set -e
exit "$status"
