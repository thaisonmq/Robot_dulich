#!/usr/bin/env bash
set -Eeo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
cd "$project_dir"

compose=(
  docker compose
  --env-file .env
  -f compose.yaml
  -f compose.coexistence.yml
  --profile legacy-coexistence
)

# Stop only Rovera's read-only services. Never use `down` here because the
# vendor and guarded legacy hardware runtimes are outside this ownership set.
"${compose[@]}" stop mapping-stack robot-simulator

