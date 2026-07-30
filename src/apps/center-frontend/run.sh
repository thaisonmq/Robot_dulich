#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
LOCK_FILE="$SCRIPT_DIR/package-lock.json"
STAMP_FILE="$SCRIPT_DIR/node_modules/.rovera-package-lock.sha256"

# shellcheck disable=SC1091
source "$PROJECT_ROOT/scripts/load-env.sh"
load_env_file "$PROJECT_ROOT/.env"

if ! command -v npm >/dev/null 2>&1; then
  echo "Lỗi: cần cài Node.js 20 trở lên và npm." >&2
  exit 1
fi

NODE_MAJOR="$(node -p 'Number(process.versions.node.split(".")[0])')"
if (( NODE_MAJOR < 20 )); then
  echo "Lỗi: đang dùng Node.js $(node --version); cần Node.js 20 trở lên." >&2
  exit 1
fi

LOCK_HASH="$(sha256sum "$LOCK_FILE" | awk '{print $1}')"
INSTALLED_HASH=""
if [[ -f "$STAMP_FILE" ]]; then
  INSTALLED_HASH="$(<"$STAMP_FILE")"
fi

if [[ ! -x "$SCRIPT_DIR/node_modules/.bin/vite" || "$LOCK_HASH" != "$INSTALLED_HASH" ]]; then
  echo "Cài dependency cho center frontend ..."
  cd "$SCRIPT_DIR"
  npm ci
  printf '%s\n' "$LOCK_HASH" >"$STAMP_FILE"
fi

cd "$SCRIPT_DIR"
echo "Center frontend: http://${CENTER_FRONTEND_HOST:-0.0.0.0}:${CENTER_FRONTEND_PORT:-5173}"
exec "$SCRIPT_DIR/node_modules/.bin/vite" \
  --host "${CENTER_FRONTEND_HOST:-0.0.0.0}" \
  --port "${CENTER_FRONTEND_PORT:-5173}"
