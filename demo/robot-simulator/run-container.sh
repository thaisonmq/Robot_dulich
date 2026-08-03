#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
IMAGE_NAME="${ROVERA_EDGE_IMAGE:-rovera/robot-edge:1.1.0}"

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

if ! command -v docker >/dev/null 2>&1 || ! docker info >/dev/null 2>&1; then
  echo "Lỗi: cần Docker Engine đang chạy để dùng runtime đóng gói." >&2
  echo "Có thể đặt ROVERA_RUNTIME=native để dùng .venv tạm thời." >&2
  exit 1
fi

if [[ "${1:-}" == "--list-cameras" ]]; then
  echo "Các thiết bị video tìm thấy:"
  compgen -G "/dev/video*" || echo "  Không tìm thấy /dev/video*"
  exit 0
fi

if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  echo "Build runtime ROVERA cố định (chỉ thực hiện khi chưa có image) ..."
  docker build \
    --build-arg ROVERA_EDGE_VERSION=1.1.0 \
    --tag "$IMAGE_NAME" \
    "$SCRIPT_DIR"
fi

HOST_ARCHITECTURE="$(docker info --format '{{.Architecture}}')"
IMAGE_ARCHITECTURE="$(docker image inspect "$IMAGE_NAME" --format '{{.Architecture}}')"
case "$HOST_ARCHITECTURE" in
  x86_64) HOST_ARCHITECTURE="amd64" ;;
  aarch64) HOST_ARCHITECTURE="arm64" ;;
esac
if [[ "$IMAGE_ARCHITECTURE" != "$HOST_ARCHITECTURE" ]]; then
  echo "Lỗi: image $IMAGE_ARCHITECTURE không chạy được trên máy $HOST_ARCHITECTURE." >&2
  echo "Hãy dùng package được build đúng kiến trúc CPU." >&2
  exit 1
fi

export CENTER_API_URL="${CENTER_API_URL:-http://localhost:8888}"
export CENTER_ROBOT_WS_URL="${CENTER_ROBOT_WS_URL:-ws://localhost:8888/ws/robot/connect}"

ROBOT_STATE_FILE_VALUE="${ROBOT_STATE_FILE:-$HOME/.config/rovera/device.json}"
if [[ "$ROBOT_STATE_FILE_VALUE" == "~/"* ]]; then
  ROBOT_STATE_FILE_VALUE="$HOME/${ROBOT_STATE_FILE_VALUE:2}"
fi
ROBOT_STATE_DIRECTORY="$(dirname -- "$ROBOT_STATE_FILE_VALUE")"
ROBOT_STATE_BASENAME="$(basename -- "$ROBOT_STATE_FILE_VALUE")"
mkdir -p "$ROBOT_STATE_DIRECTORY"
if [[ ! -w "$ROBOT_STATE_DIRECTORY" ]]; then
  echo "Lỗi: không có quyền ghi device state: $ROBOT_STATE_DIRECTORY" >&2
  echo "Đặt ROBOT_STATE_FILE=~/.config/rovera/device.json rồi chạy lại." >&2
  exit 1
fi

ROBOT_CREDENTIAL_VALUE="${ROBOT_CREDENTIAL:-}"
ROBOT_ENROLLMENT_TOKEN_VALUE="${ROBOT_ENROLLMENT_TOKEN:-}"
ROBOT_USERNAME_VALUE="${ROBOT_USERNAME:-}"
ROBOT_PASSWORD_VALUE="${ROBOT_PASSWORD:-}"
if [[
  ${#ROBOT_CREDENTIAL_VALUE} -lt 16
  && ${#ROBOT_ENROLLMENT_TOKEN_VALUE} -lt 32
  && ( -z "$ROBOT_USERNAME_VALUE" || ${#ROBOT_PASSWORD_VALUE} -lt 6 )
  && ! -f "$ROBOT_STATE_FILE_VALUE"
]]; then
  echo "Lỗi: robot chưa có tài khoản/mật khẩu, credential, enrollment token hoặc device state." >&2
  exit 1
fi

if [[ "${1:-}" == "--check-config" ]]; then
  echo "Cấu hình container robot hợp lệ."
  echo "  Image: $IMAGE_NAME"
  echo "  File env: $ROBOT_ENV_FILE_VALUE"
  echo "  Center API: $CENTER_API_URL"
  echo "  Robot gateway: $CENTER_ROBOT_WS_URL"
  echo "  Device state: $ROBOT_STATE_FILE_VALUE"
  exit 0
fi

ENVIRONMENT_NAMES=(
  CENTER_API_URL CENTER_ROBOT_WS_URL CENTER_TLS_VERIFY CENTER_TLS_CA_FILE
  ROBOT_ID ROBOT_CREDENTIAL ROBOT_ENROLLMENT_TOKEN ROBOT_MANAGEMENT_ADDRESS
  ROBOT_USERNAME ROBOT_PASSWORD MAP_ID MAP_WIDTH_M MAP_HEIGHT_M INITIAL_X
  INITIAL_Y INITIAL_YAW SIMULATION_HZ TELEMETRY_HZ COMMAND_WATCHDOG_MS
  MAX_FORWARD_SPEED MAX_REVERSE_SPEED MAX_ANGULAR_SPEED HEARTBEAT_SECONDS
  MOTION_BACKEND MOTION_SOCKET_PATH MOTION_WATCHDOG_MS
  ROS_MAX_FORWARD_SPEED ROS_MAX_REVERSE_SPEED ROS_MAX_ANGULAR_SPEED
  LIVEKIT_URL SIMULATOR_MEDIA_SOURCE_TYPE SIMULATOR_MEDIA_SOURCE
  SIMULATOR_AUDIO_SOURCE SIMULATOR_AUDIO_SOURCE_TYPE SIMULATOR_AUDIO_OUTPUT
  SIMULATOR_AUDIO_OUTPUT_TYPE SIMULATOR_CAMERA_DEVICE
  SIMULATOR_CAMERA_FORMAT SIMULATOR_CAMERA_WIDTH SIMULATOR_CAMERA_HEIGHT
  SIMULATOR_CAMERA_FPS DEVICE_IP CAMERA_LABEL MICROPHONE_LABEL SPEAKER_LABEL VIDEO_PROFILE
  RTSP_TRANSPORT MEDIA_ENABLED VIDEO_WIDTH VIDEO_HEIGHT VIDEO_FPS VIDEO_BITRATE
  VIDEO_ENCODER VIDEO_PIPELINE VIDEO_PASSTHROUGH VIDEO_FFMPEG_BINARY
  SIMULATOR_RTSP_PATH
)

DOCKER_ARGUMENTS=(
  run --rm --network host
  --hostname "$(hostname)"
  --user "$(id -u):$(id -g)"
  --volume "$ROBOT_STATE_DIRECTORY:/var/lib/rovera"
  --env "ROBOT_STATE_FILE=/var/lib/rovera/$ROBOT_STATE_BASENAME"
)

for environment_name in "${ENVIRONMENT_NAMES[@]}"; do
  if [[ -v "$environment_name" ]]; then
    DOCKER_ARGUMENTS+=(--env "$environment_name")
  fi
done

if [[ -n "${CENTER_TLS_CA_FILE:-}" && -f "$CENTER_TLS_CA_FILE" ]]; then
  DOCKER_ARGUMENTS+=(--volume "$CENTER_TLS_CA_FILE:$CENTER_TLS_CA_FILE:ro")
fi

mapfile -t VIDEO_DEVICES < <(compgen -G "/dev/video*" || true)
mapfile -t AUDIO_DEVICES < <(compgen -G "/dev/snd/*" || true)
mapfile -t DRM_DEVICES < <(compgen -G "/dev/dri/*" || true)
mapfile -t DMA_HEAP_DEVICES < <(compgen -G "/dev/dma_heap/*" || true)
ROCKCHIP_DEVICES=()
for device in \
  /dev/mpp_service /dev/mpp-service \
  /dev/rga /dev/iep \
  /dev/rkvdec /dev/rkvenc \
  /dev/vpu_service /dev/vpu-service \
  /dev/hevc_service /dev/hevc-service \
  /dev/vepu /dev/h265e; do
  [[ -e "$device" ]] && ROCKCHIP_DEVICES+=("$device")
done
for device in \
  "${VIDEO_DEVICES[@]}" \
  "${AUDIO_DEVICES[@]}" \
  "${DRM_DEVICES[@]}" \
  "${DMA_HEAP_DEVICES[@]}" \
  "${ROCKCHIP_DEVICES[@]}"; do
  [[ -c "$device" ]] || continue
  DOCKER_ARGUMENTS+=(--device "$device:$device")
  device_group="$(stat -c '%g' "$device")"
  DOCKER_ARGUMENTS+=(--group-add "$device_group")
done

PULSE_HOST_UID="${PULSE_UID:-${SUDO_UID:-$(id -u)}}"
HOST_PULSE_DIRECTORY="${PULSE_SOCKET_DIR:-/run/user/$PULSE_HOST_UID/pulse}"
if [[ -S "$HOST_PULSE_DIRECTORY/native" ]]; then
  DOCKER_ARGUMENTS+=(
    --volume "$HOST_PULSE_DIRECTORY:/run/rovera-pulse:ro"
    --env "PULSE_SERVER=unix:/run/rovera-pulse/native"
  )
fi

if [[ -r /etc/machine-id ]]; then
  DOCKER_ARGUMENTS+=(--volume "/etc/machine-id:/etc/machine-id:ro")
fi

echo "Runtime: container $IMAGE_NAME"
echo "Nạp cấu hình robot từ $ROBOT_ENV_FILE_VALUE"
echo "Robot edge xác thực tại $CENTER_API_URL"
echo "Robot edge kết nối tới $CENTER_ROBOT_WS_URL"
exec docker "${DOCKER_ARGUMENTS[@]}" "$IMAGE_NAME"
