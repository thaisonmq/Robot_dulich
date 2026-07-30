#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
VENV_DIR="$PROJECT_ROOT/.venv"
REQUIREMENTS_FILE="$SCRIPT_DIR/requirements.txt"
STAMP_DIR="$VENV_DIR/.rovera-stamps"
STAMP_FILE="$STAMP_DIR/center-backend.sha256"

# shellcheck disable=SC1091
source "$PROJECT_ROOT/scripts/load-env.sh"
load_env_file "$PROJECT_ROOT/.env"

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
  echo "Cài dependency cho center backend ..."
  "$VENV_DIR/bin/python" -m pip install --upgrade pip
  "$VENV_DIR/bin/python" -m pip install -r "$REQUIREMENTS_FILE"
  printf '%s\n' "$REQUIREMENTS_HASH" >"$STAMP_FILE"
fi

export LIVEKIT_URL="${LIVEKIT_URL:-${LIVEKIT_PUBLIC_URL:-ws://localhost:7880}}"
export PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"

cd "$SCRIPT_DIR"
"$VENV_DIR/bin/alembic" upgrade head

echo "Center backend: http://${CENTER_BACKEND_HOST:-0.0.0.0}:${CENTER_BACKEND_PORT:-8888}"
exec "$VENV_DIR/bin/uvicorn" app.main:app \
  --host "${CENTER_BACKEND_HOST:-0.0.0.0}" \
  --port "${CENTER_BACKEND_PORT:-8888}" \
  --reload
