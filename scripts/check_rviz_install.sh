#!/usr/bin/env bash

# This file is sourced by the RViz launch helpers. Keep it side-effect free
# except for the validation output and the non-zero return on a mixed install.
check_rviz_install() {
  local -a rviz_packages=(
    ros-humble-rviz-assimp-vendor
    ros-humble-rviz-common
    ros-humble-rviz-default-plugins
    ros-humble-rviz-ogre-vendor
    ros-humble-rviz-rendering
    ros-humble-rviz2
  )
  local package package_version release_versions unique_versions

  release_versions=""
  for package in "${rviz_packages[@]}"; do
    package_version="$(dpkg-query -W -f='${Version}' "${package}" 2>/dev/null || true)"
    if [[ -n "${package_version}" ]]; then
      release_versions+="${package_version%%-*}"$'\n'
    fi
  done
  unique_versions="$(printf '%s' "${release_versions}" | sed '/^$/d' | sort -u)"

  if [[ "$(printf '%s\n' "${unique_versions}" | sed '/^$/d' | wc -l)" -gt 1 ]]; then
    echo "RViz đang bị trộn phiên bản (${unique_versions//$'\n'/, }). Điều này có thể gây màn hình màu hồng/lỗi GLSL." >&2
    echo "Đóng RViz rồi chạy:" >&2
    echo "  sudo apt update" >&2
    echo "  sudo apt install --only-upgrade ${rviz_packages[*]}" >&2
    return 1
  fi
}
