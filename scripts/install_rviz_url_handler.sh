#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
install_root="${XDG_DATA_HOME:-${HOME}/.local/share}/rovera-rviz"
application_dir="${XDG_DATA_HOME:-${HOME}/.local/share}/applications"
desktop_file="${application_dir}/rovera-rviz.desktop"

source "${project_root}/scripts/check_rviz_install.sh"
check_rviz_install

install -d "${install_root}/scripts" "${install_root}/config" "${application_dir}"
cp -a "${project_root}/config/rviz" "${install_root}/config/"
install -m 0755 \
  "${project_root}/scripts/open_mapping_rviz.sh" \
  "${project_root}/scripts/check_rviz_install.sh" \
  "${project_root}/scripts/prepare_rviz_media_compat.sh" \
  "${install_root}/scripts/"
install -m 0755 \
  "${project_root}/scripts/rovera_rviz_url_handler.py" \
  "${install_root}/rovera_rviz_url_handler.py"

escaped_handler="${install_root}/rovera_rviz_url_handler.py"
sed "s|@HANDLER_PATH@|${escaped_handler}|g" \
  "${project_root}/config/rviz/rovera-rviz.desktop.in" > "${desktop_file}"
chmod 0644 "${desktop_file}"

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "${application_dir}" >/dev/null 2>&1 || true
fi
xdg-mime default rovera-rviz.desktop x-scheme-handler/rovera-rviz

echo "Đã cài Rovera RViz launcher cho user ${USER}."
echo "Kiểm tra bằng: xdg-open 'rovera-rviz://mapping?domain=21'"
