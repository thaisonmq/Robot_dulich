#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
set -u
exec "$@"
