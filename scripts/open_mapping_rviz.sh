#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
project_ros_distro="humble"
ros_setup="/opt/ros/${project_ros_distro}/setup.bash"

if [[ ! -r "${ros_setup}" ]]; then
  echo "Không tìm thấy ROS 2 Humble tại ${ros_setup}. Xem docs/RVIZ_MAPPING_GUIDE.md." >&2
  exit 1
fi

# ROS Humble setup scripts legitimately probe optional variables such as
# AMENT_TRACE_SETUP_FILES. Temporarily disable nounset while sourcing them.
set +u
# shellcheck disable=SC1090
source "${ros_setup}"
set -u

# shellcheck source=check_rviz_install.sh
source "${project_root}/scripts/check_rviz_install.sh"
check_rviz_install

# shellcheck source=prepare_rviz_media_compat.sh
source "${project_root}/scripts/prepare_rviz_media_compat.sh"
prepare_rviz_media_compat "${project_root}" "/opt/ros/${project_ros_distro}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${project_root}/config/rviz/rviz_lan_fastdds.xml}"
echo "ROS 2: ${project_ros_distro}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "RMW: ${RMW_IMPLEMENTATION}"

if command -v ros2 >/dev/null 2>&1; then
  topics="$(ros2 topic list 2>/dev/null || true)"
  for topic in /scan_mapping /map /tf /tf_static /odometry/filtered; do
    if grep -Fxq "${topic}" <<<"${topics}"; then
      echo "${topic} : OK"
    else
      echo "${topic} : CHƯA THẤY"
    fi
  done
else
  echo "Thiếu ros2 CLI; cài ros-humble-ros2cli ros-humble-ros2topic để kiểm tra topic." >&2
fi

echo "Opening RViz2 (subscriber only; không khởi động robot stack)..."
exec rviz2 -d "${project_root}/config/rviz/mapping.rviz"
