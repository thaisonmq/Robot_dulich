#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
VENV_DIR="$PROJECT_ROOT/.venv"
REQUIREMENTS_FILE="$SCRIPT_DIR/requirements.txt"
STAMP_DIR="$VENV_DIR/.rovera-stamps"
STAMP_FILE="$STAMP_DIR/robot-simulator.sha256"

if [[ "${ROVERA_RUNTIME:-container}" != "native" ]]; then
  if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
    exec "$SCRIPT_DIR/run-container.sh" "$@"
  fi
  echo "Lỗi: Docker chưa sẵn sàng; không chạy để tránh phụ thuộc Python/FFmpeg của host." >&2
  echo "Cài Docker Engine hoặc chủ động đặt ROVERA_RUNTIME=native nếu cần chế độ cũ." >&2
  exit 1
fi

# shellcheck disable=SC1091
source "$PROJECT_ROOT/scripts/load-env.sh"
if [[ -n "${ROBOT_ENV_FILE:-}" ]]; then
  ROBOT_ENV_FILE_VALUE="$ROBOT_ENV_FILE"
elif [[ -f "$SCRIPT_DIR/.env" ]]; then
  ROBOT_ENV_FILE_VALUE="$SCRIPT_DIR/.env"
elif [[ -f "$PROJECT_ROOT/.env" ]]; then
  ROBOT_ENV_FILE_VALUE="$PROJECT_ROOT/.env"
elif [[ -f /etc/rovera/robot.env ]]; then
  ROBOT_ENV_FILE_VALUE="/etc/rovera/robot.env"
else
  ROBOT_ENV_FILE_VALUE="$SCRIPT_DIR/.env"
fi
load_env_file "$ROBOT_ENV_FILE_VALUE"

if [[ "${1:-}" == "--list-cameras" ]]; then
  if command -v v4l2-ctl >/dev/null 2>&1; then
    exec v4l2-ctl --list-devices
  fi
  echo "Các thiết bị video tìm thấy:"
  compgen -G "/dev/video*" || echo "  Không tìm thấy /dev/video*"
  echo "Cài v4l-utils để xem đầy đủ định dạng camera."
  exit 0
fi

if [[ ! -x "$VENV_DIR/bin/python" ]]; then
  if command -v python3.12 >/dev/null 2>&1; then
    PYTHON_BIN="python3.12"
  elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
  else
    echo "Lỗi: cần cài Python 3.10 trở lên." >&2
    exit 1
  fi

  echo "Tạo môi trường Python tại $VENV_DIR ..."
  "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

PYTHON_VERSION="$("$VENV_DIR/bin/python" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if ! "$VENV_DIR/bin/python" -c 'import sys; raise SystemExit(sys.version_info < (3, 10))'; then
  echo "Lỗi: .venv đang dùng Python $PYTHON_VERSION; cần Python 3.10 trở lên." >&2
  exit 1
fi

mkdir -p "$STAMP_DIR"
REQUIREMENTS_HASH="$(sha256sum "$REQUIREMENTS_FILE" | awk '{print $1}')"
INSTALLED_HASH=""
if [[ -f "$STAMP_FILE" ]]; then
  INSTALLED_HASH="$(<"$STAMP_FILE")"
fi

if [[ "$REQUIREMENTS_HASH" != "$INSTALLED_HASH" ]]; then
  echo "Cài dependency cho robot simulator ..."
  "$VENV_DIR/bin/python" -m pip install --upgrade pip
  "$VENV_DIR/bin/python" -m pip install -r "$REQUIREMENTS_FILE"
  printf '%s\n' "$REQUIREMENTS_HASH" >"$STAMP_FILE"
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "Cảnh báo: chưa có FFmpeg; video/audio simulator sẽ không phát được." >&2
fi
if ! command -v aplay >/dev/null 2>&1 || ! command -v arecord >/dev/null 2>&1; then
  echo "Cảnh báo: cài alsa-utils để thu/phát ALSA với độ trễ thấp." >&2
fi
if ! command -v pacat >/dev/null 2>&1; then
  echo "Cảnh báo: cài pulseaudio-utils để thu/phát PipeWire/PulseAudio với độ trễ thấp." >&2
fi

export CENTER_API_URL="${CENTER_API_URL:-http://localhost:8888}"
export CENTER_ROBOT_WS_URL="${CENTER_ROBOT_WS_URL:-ws://localhost:8888/ws/robot/connect}"
export PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"

ROBOT_CREDENTIAL_VALUE="${ROBOT_CREDENTIAL:-}"
ROBOT_ENROLLMENT_TOKEN_VALUE="${ROBOT_ENROLLMENT_TOKEN:-}"
ROBOT_USERNAME_VALUE="${ROBOT_USERNAME:-}"
ROBOT_PASSWORD_VALUE="${ROBOT_PASSWORD:-}"
ROBOT_STATE_FILE_VALUE="${ROBOT_STATE_FILE:-$HOME/.config/rovera/device.json}"
if [[ "$ROBOT_STATE_FILE_VALUE" == "~/"* ]]; then
  ROBOT_STATE_FILE_VALUE="$HOME/${ROBOT_STATE_FILE_VALUE:2}"
  export ROBOT_STATE_FILE="$ROBOT_STATE_FILE_VALUE"
fi
if [[
  ${#ROBOT_CREDENTIAL_VALUE} -lt 16
  && ${#ROBOT_ENROLLMENT_TOKEN_VALUE} -lt 32
  && ( -z "$ROBOT_USERNAME_VALUE" || ${#ROBOT_PASSWORD_VALUE} -lt 6 )
  && ! -f "$ROBOT_STATE_FILE_VALUE"
]]; then
  echo "Lỗi: robot chưa có tài khoản/mật khẩu, credential, enrollment token hoặc device state." >&2
  exit 1
fi

if [[ ! -f "$ROBOT_STATE_FILE_VALUE" ]]; then
  ROBOT_STATE_DIRECTORY="$(dirname -- "$ROBOT_STATE_FILE_VALUE")"
  if ! mkdir -p "$ROBOT_STATE_DIRECTORY" 2>/dev/null; then
    echo "Lỗi: không thể tạo thư mục lưu device state: $ROBOT_STATE_DIRECTORY" >&2
    echo "Khi chạy thủ công, đặt ROBOT_STATE_FILE=~/.config/rovera/device.json" >&2
    echo "Chỉ dùng /var/lib/rovera/device.json khi chạy bằng systemd hoặc đã cấp quyền." >&2
    exit 1
  fi
  if [[ ! -w "$ROBOT_STATE_DIRECTORY" ]]; then
    echo "Lỗi: không có quyền ghi device state vào $ROBOT_STATE_DIRECTORY" >&2
    echo "Đổi ROBOT_STATE_FILE=~/.config/rovera/device.json rồi chạy lại." >&2
    exit 1
  fi
fi

MEDIA_SOURCE_TYPE_VALUE="${SIMULATOR_MEDIA_SOURCE_TYPE:-test}"
CAMERA_DEVICE_VALUE="${SIMULATOR_MEDIA_SOURCE:-${SIMULATOR_CAMERA_DEVICE:-/dev/video0}}"
if [[ "$MEDIA_SOURCE_TYPE_VALUE" == "camera" && -f "$ROBOT_STATE_FILE_VALUE" ]]; then
  echo "Nguồn camera sẽ được nạp từ cấu hình đã lưu của Center."
elif [[ "$MEDIA_SOURCE_TYPE_VALUE" == "camera" ]]; then
  if [[ ! -e "$CAMERA_DEVICE_VALUE" ]]; then
    echo "Cảnh báo: chưa tìm thấy camera USB $CAMERA_DEVICE_VALUE." >&2
    echo "Tạm dùng test pattern; hãy chọn nguồn camera từ màn Cấu hình trên Center." >&2
    export SIMULATOR_MEDIA_SOURCE_TYPE="test"
    export SIMULATOR_MEDIA_SOURCE="generated://test-pattern"
    MEDIA_SOURCE_TYPE_VALUE="test"
  elif [[ ! -r "$CAMERA_DEVICE_VALUE" ]]; then
    echo "Cảnh báo: chưa có quyền đọc $CAMERA_DEVICE_VALUE." >&2
    echo "Tạm dùng test pattern; thêm user vào group video trước khi dùng camera." >&2
    export SIMULATOR_MEDIA_SOURCE_TYPE="test"
    export SIMULATOR_MEDIA_SOURCE="generated://test-pattern"
    MEDIA_SOURCE_TYPE_VALUE="test"
  fi
fi

if [[ "${1:-}" == "--check-config" ]]; then
  echo "Cấu hình robot hợp lệ."
  echo "  File env: $ROBOT_ENV_FILE_VALUE"
  echo "  Center API: $CENTER_API_URL"
  echo "  Robot gateway: $CENTER_ROBOT_WS_URL"
  echo "  Địa chỉ robot: ${ROBOT_MANAGEMENT_ADDRESS:-chưa đặt}"
  echo "  Device state: $ROBOT_STATE_FILE_VALUE"
  echo "  Nguồn media: $MEDIA_SOURCE_TYPE_VALUE"
  exit 0
fi

if [[ "$CENTER_API_URL" != http://localhost:* && "$CENTER_API_URL" != http://127.0.0.1:* ]]; then
  if [[ "$CENTER_API_URL" == http://* || "$CENTER_ROBOT_WS_URL" == ws://* ]]; then
    echo "Cảnh báo bảo mật: máy từ xa nên dùng HTTPS + WSS cho kết nối Center." >&2
  fi
fi

cd "$SCRIPT_DIR"
echo "Nạp cấu hình robot từ $ROBOT_ENV_FILE_VALUE"
echo "Robot edge xác thực tại $CENTER_API_URL"
echo "Robot edge kết nối tới $CENTER_ROBOT_WS_URL"
exec "$VENV_DIR/bin/python" -m simulator.main
