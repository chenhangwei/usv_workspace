#!/bin/bash
# fresh100 selected evaluation: compare fresh98 baselines and fresh100
# pretrain-role candidates after crossing-focus breakthrough.

set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh100_selected_eval}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh100_checkpoints}"
export LOG="${LOG:-/tmp/fresh100_selected_eval.log}"
export BASE_DOMAIN="${BASE_DOMAIN:-193}"
export FULL5_EPISODES="${FULL5_EPISODES:-5}"
export TARGETED_EPISODES="${TARGETED_EPISODES:-9}"
export STEPS="${STEPS:-360}"
export EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-96}"
export NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-32}"
export EVAL_TIMEOUT="${EVAL_TIMEOUT:-520}"
export FRESH100_LATEST_COUNT="${FRESH100_LATEST_COUNT:-16}"

CANDIDATES=()
add_candidate() {
  local model="$1"
  if [[ -z "$model" || ! -f "$model" ]]; then
    return 0
  fi
  for existing in "${CANDIDATES[@]}"; do
    [[ "$existing" == "$model" ]] && return 0
  done
  CANDIDATES+=("$model")
}

add_candidate /mnt/data/checkpoints/usv_rl/fresh98_checkpoints/fresh98_pairwise_conflict_repair_step_0062784.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh98_pairwise_conflict_repair.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh100_crossing_pretrain_role.pt

mapfile -t latest_fresh100 < <(ls -1t "$CKPT_DIR"/fresh100_crossing_pretrain_role_step_*.pt 2>/dev/null | head -n "$FRESH100_LATEST_COUNT")
for model in "${latest_fresh100[@]}"; do
  add_candidate "$model"
done

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "No existing fresh100 selected-eval candidates found."
  exit 1
fi

bash src/scripts/fresh91_selected_eval.sh "${CANDIDATES[@]}"