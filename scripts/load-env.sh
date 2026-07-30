#!/usr/bin/env bash

# Load a conventional KEY=VALUE file without evaluating its content as shell
# code. This keeps values such as RTSP URLs containing &, ?, or # intact.
load_env_file() {
  local env_file="$1"
  local line key value first last

  [[ -f "$env_file" ]] || return 0

  while IFS= read -r line || [[ -n "$line" ]]; do
    line="${line%$'\r'}"
    line="${line#"${line%%[![:space:]]*}"}"
    line="${line%"${line##*[![:space:]]}"}"

    [[ -z "$line" || "${line:0:1}" == "#" ]] && continue
    [[ "$line" == export\ * ]] && line="${line#export }"
    [[ "$line" == *=* ]] || continue

    key="${line%%=*}"
    value="${line#*=}"
    key="${key#"${key%%[![:space:]]*}"}"
    key="${key%"${key##*[![:space:]]}"}"
    value="${value#"${value%%[![:space:]]*}"}"
    value="${value%"${value##*[![:space:]]}"}"

    if [[ ! "$key" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
      echo "Cảnh báo: bỏ qua biến .env không hợp lệ: $key" >&2
      continue
    fi

    if (( ${#value} >= 2 )); then
      first="${value:0:1}"
      last="${value: -1}"
      if [[ "$first" == "$last" && ( "$first" == \" || "$first" == "'" ) ]]; then
        value="${value:1:${#value}-2}"
      fi
    fi

    export "$key=$value"
  done <"$env_file"
}
