#!/bin/bash
# fresh94 selected evaluation: compare fresh93 best/final against fresh94 decoupled-yield candidates.

set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh94_selected_eval}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh94_checkpoints}"
export LOG="${LOG:-/tmp/fresh94_selected_eval.log}"
export BASE_DOMAIN="${BASE_DOMAIN:-171}"
export FULL5_EPISODES="${FULL5_EPISODES:-5}"
export TARGETED_EPISODES="${TARGETED_EPISODES:-9}"
export STEPS="${STEPS:-300}"
export EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-70}"
export NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-20}"
export EVAL_TIMEOUT="${EVAL_TIMEOUT:-360}"
export FRESH94_LATEST_COUNT="${FRESH94_LATEST_COUNT:-6}"

declare -A SEEN=()
CANDIDATES=()

add_candidate() {
  local model="$1"
  if [[ -z "$model" || ! -f "$model" || -n "${SEEN[$model]:-}" ]]; then
    return 0
  fi
  SEEN["$model"]=1
  CANDIDATES+=("$model")
}

add_candidate /mnt/data/checkpoints/usv_rl/fresh92_crossing_branch_repair.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh93_checkpoints/fresh93_crossing_yield_branch_repair_step_0009792.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh93_crossing_yield_branch_repair.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh94_crossing_yield_decouple_repair.pt

mapfile -t latest_fresh94 < <(ls -1t /mnt/data/checkpoints/usv_rl/fresh94_checkpoints/fresh94_crossing_yield_decouple_repair_step_*.pt 2>/dev/null | head -n "$FRESH94_LATEST_COUNT")
for model in "${latest_fresh94[@]}"; do
  add_candidate "$model"
done

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "No existing fresh94 selected-eval candidates found."
  exit 1
fi

bash src/scripts/fresh91_selected_eval.sh "${CANDIDATES[@]}"
