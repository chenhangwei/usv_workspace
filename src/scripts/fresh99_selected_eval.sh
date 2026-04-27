#!/bin/bash
# fresh99 selected evaluation: compare fresh98 baselines and fresh99 role-imitation candidates.

set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh99_selected_eval}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh99_checkpoints}"
export LOG="${LOG:-/tmp/fresh99_selected_eval.log}"
export BASE_DOMAIN="${BASE_DOMAIN:-163}"
export FULL5_EPISODES="${FULL5_EPISODES:-5}"
export TARGETED_EPISODES="${TARGETED_EPISODES:-9}"
export STEPS="${STEPS:-360}"
export EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-96}"
export NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-32}"
export EVAL_TIMEOUT="${EVAL_TIMEOUT:-520}"
export FRESH99_LATEST_COUNT="${FRESH99_LATEST_COUNT:-16}"

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
add_candidate /mnt/data/checkpoints/usv_rl/fresh99_crossing_role_imitation.pt

mapfile -t latest_fresh99 < <(ls -1t "$CKPT_DIR"/fresh99_crossing_role_imitation_step_*.pt 2>/dev/null | head -n "$FRESH99_LATEST_COUNT")
for model in "${latest_fresh99[@]}"; do
  add_candidate "$model"
done

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "No existing fresh99 selected-eval candidates found."
  exit 1
fi

bash src/scripts/fresh91_selected_eval.sh "${CANDIDATES[@]}"
