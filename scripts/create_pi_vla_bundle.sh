#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
head_sha="$(git -C "$repo_root" rev-parse --short HEAD)"
output="${1:-/tmp/phoenix-pi-vla-${head_sha}.tar.gz}"

git -C "$repo_root" archive --format=tar.gz --output="$output" HEAD \
  src/fire_vla_core \
  src/fire_vla_bringup \
  src/uncc_example \
  docs/VLA_ROBOT_E2E_CURRENT_STATUS.md \
  docs/VLA_HARDWARE_RESUME_RUNBOOK.md \
  docs/CURRENT_VLA_DATA_ARCHITECTURE.md

printf '%s\n' "$output"
