#!/usr/bin/env bash
set -Eeo pipefail

if [ "${EUID}" -ne 0 ]; then
  echo >&2 "Run with sudo: sudo $0 <project-dir> [service-user]"
  exit 77
fi

project_dir="$(realpath "${1:?missing project directory}")"
service_user="${2:-${SUDO_USER:-pi}}"
env_file="$project_dir/.env"
if [ ! -f "$env_file" ]; then
  echo >&2 "Missing $env_file"
  exit 66
fi

state_value="$(sed -n 's/^ROBOT_STATE_DIR=//p' "$env_file" | tail -1)"
state_value="${state_value:-./state}"
if [[ "$state_value" = /* ]]; then
  state_dir="$state_value"
else
  state_dir="$(realpath -m "$project_dir/$state_value")"
fi

install -d -m 0755 /usr/local/lib/rovera
install -m 0755 "$project_dir/scripts/mode_supervisor.py" /usr/local/lib/rovera/mode_supervisor.py
install -d -o "$service_user" -g "$service_user" -m 0755 "$state_dir/navigation"

cat >/etc/systemd/system/rovera-mode-supervisor.service <<EOF
[Unit]
Description=Rovera safe SLAM/Nav2 mode supervisor
After=docker.service
Requires=docker.service

[Service]
Type=simple
User=$service_user
Group=$service_user
Environment=ROVERA_PROJECT_DIR=$project_dir
Environment=ROVERA_STATE_DIR=$state_dir
ExecStart=/usr/bin/python3 /usr/local/lib/rovera/mode_supervisor.py
Restart=always
RestartSec=2
TimeoutStopSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now rovera-mode-supervisor.service
systemctl --no-pager --full status rovera-mode-supervisor.service
