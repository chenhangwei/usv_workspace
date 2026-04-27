#!/bin/bash
# fresh98 selected evaluation: compare fresh97 baselines and fresh98 pairwise candidates.

set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh98_selected_eval}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh98_checkpoints}"
export LOG="${LOG:-/tmp/fresh98_selected_eval.log}"
export BASE_DOMAIN="${BASE_DOMAIN:-163}"
export FULL5_EPISODES="${FULL5_EPISODES:-5}"
export TARGETED_EPISODES="${TARGETED_EPISODES:-9}"
export STEPS="${STEPS:-360}"
export EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-92}"
export NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-30}"
export EVAL_TIMEOUT="${EVAL_TIMEOUT:-500}"
export FRESH98_LATEST_COUNT="${FRESH98_LATEST_COUNT:-14}"

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

add_candidate /mnt/data/checkpoints/usv_rl/fresh97_checkpoints/fresh97_unfrozen_eta_temporal_repair_step_0033984.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh97_checkpoints/fresh97_unfrozen_eta_temporal_repair_step_0040896.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh97_unfrozen_eta_temporal_repair.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh98_pairwise_conflict_repair.pt

mapfile -t latest_fresh98 < <(ls -1t "$CKPT_DIR"/fresh98_pairwise_conflict_repair_step_*.pt 2>/dev/null | head -n "$FRESH98_LATEST_COUNT")
for model in "${latest_fresh98[@]}"; do
  add_candidate "$model"
done

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "No existing fresh98 selected-eval candidates found."
  exit 1
fi

bash src/scripts/fresh91_selected_eval.sh "${CANDIDATES[@]}"
