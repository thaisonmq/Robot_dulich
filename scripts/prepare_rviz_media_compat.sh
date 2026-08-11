#!/usr/bin/env bash

# Prepare an unprivileged RViz media overlay. RViz discovers its core media
# through the ament package index, so a complete copied resource directory is
# needed; only the occupancy-grid shader and material are replaced.
prepare_rviz_media_compat() {
  local project_root="$1"
  local ros_prefix="$2"
  local source_share="${ros_prefix}/share/rviz_rendering"
  local compat_assets="${project_root}/config/rviz/mesa_compat"
  local runtime_parent compat_hash rviz_version compat_root marker

  if [[ ! -d "${source_share}/ogre_media" ]]; then
    echo "Không tìm thấy tài nguyên rviz_rendering tại ${source_share}." >&2
    return 1
  fi

  compat_hash="$(sha256sum \
    "${compat_assets}/indexed_8bit_image.frag" \
    "${compat_assets}/indexed_8bit_image.material" | sha256sum | cut -c1-16)"
  rviz_version="$(dpkg-query -W -f='${Version}' ros-humble-rviz-rendering 2>/dev/null || echo unknown)"
  rviz_version="${rviz_version//[^A-Za-z0-9._-]/_}"
  runtime_parent="${XDG_RUNTIME_DIR:-/tmp}/rovera-rviz-media-$(id -u)"
  compat_root="${runtime_parent}/${rviz_version}-${compat_hash}"
  marker="${compat_root}/share/ament_index/resource_index/packages/rviz_rendering"

  if [[ ! -f "${marker}" ]]; then
    mkdir -p "${compat_root}/share/ament_index/resource_index/packages"
    cp -a "${source_share}" "${compat_root}/share/"
    install -m 0644 \
      "${compat_assets}/indexed_8bit_image.frag" \
      "${compat_root}/share/rviz_rendering/ogre_media/materials/glsl120/indexed_8bit_image.frag"
    install -m 0644 \
      "${compat_assets}/indexed_8bit_image.material" \
      "${compat_root}/share/rviz_rendering/ogre_media/materials/scripts/indexed_8bit_image.material"
    install -m 0644 /dev/null "${marker}"
  fi

  export AMENT_PREFIX_PATH="${compat_root}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  echo "RViz map renderer: compatibility shader (${rviz_version})"
}
