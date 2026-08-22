#!/usr/bin/env bash
set -eo pipefail

required_ack="I_ACCEPT_EXCLUSIVE_CMD_VEL_OWNERSHIP"
if [ "${ROVERA_CMD_VEL_MODE:-}" != "exclusive" ] || \
   [ "${ROVERA_EXCLUSIVE_CMD_VEL_ACK:-}" != "$required_ack" ]; then
  echo >&2 "motion-safety refused to start: exclusive /cmd_vel ownership was not acknowledged"
  echo >&2 "use the managed-motion cutover; do not start this beside a legacy /cmd_vel publisher"
  exit 78
fi

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
rm -f /tmp/rovera-safety/velocity-smoother-active
set -u
exec "$@"
