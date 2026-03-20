#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/mnt/workspace/usv_workspace"

usage() {
  cat <<'EOF'
usage: resume_mappo_training_cpu_local.sh <resume_checkpoint.pt> <new_total_timesteps> [output_path.pt] [extra train args...]

Forces a local MAPPO resume run onto CPU.
Useful when the local GPU is incompatible with the installed PyTorch CUDA wheel.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -lt 2 ]]; then
  usage
  exit 0
fi

contains_flag() {
  local wanted_flag="$1"
  shift || true
  for arg in "$@"; do
    if [[ "${arg}" == "${wanted_flag}" ]]; then
      return 0
    fi
  done
  return 1
}

USER_ARGS=("$@")
DEFAULT_ARGS=()

if ! contains_flag --device "${USER_ARGS[@]}"; then
  DEFAULT_ARGS+=(--device cpu)
fi
if ! contains_flag --amp "${USER_ARGS[@]}"; then
  DEFAULT_ARGS+=(--amp off)
fi
if ! contains_flag --checkpoint-eval-device "${USER_ARGS[@]}"; then
  DEFAULT_ARGS+=(--checkpoint-eval-device cpu)
fi

export CUDA_VISIBLE_DEVICES="-1"

cd "${WORKSPACE_DIR}"
exec ./scripts/resume_mappo_training.sh "${USER_ARGS[@]}" "${DEFAULT_ARGS[@]}"
