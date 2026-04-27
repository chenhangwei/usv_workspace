#!/bin/bash
# fresh97 selected evaluation: compare fresh96 and fresh97 temporal candidates.

set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh97_selected_eval}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh97_checkpoints}"
export LOG="${LOG:-/tmp/fresh97_selected_eval.log}"
export BASE_DOMAIN="${BASE_DOMAIN:-161}"
export FULL5_EPISODES="${FULL5_EPISODES:-5}"
export TARGETED_EPISODES="${TARGETED_EPISODES:-9}"
export STEPS="${STEPS:-340}"
export EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-88}"
export NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-28}"
export EVAL_TIMEOUT="${EVAL_TIMEOUT:-470}"
export FRESH97_LATEST_COUNT="${FRESH97_LATEST_COUNT:-12}"

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

add_candidate /mnt/data/checkpoints/usv_rl/fresh95_checkpoints/fresh95_crossing_multiway_starboard_repair_step_0028800.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh96_crossing_eta_priority_repair.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh97_unfrozen_eta_temporal_repair.pt

mapfile -t latest_fresh96 < <(ls -1t /mnt/data/checkpoints/usv_rl/fresh96_checkpoints/fresh96_crossing_eta_priority_repair_step_*.pt 2>/dev/null | head -n 4)
for model in "${latest_fresh96[@]}"; do
  add_candidate "$model"
done

mapfile -t latest_fresh97 < <(ls -1t "$CKPT_DIR"/fresh97_unfrozen_eta_temporal_repair_step_*.pt 2>/dev/null | head -n "$FRESH97_LATEST_COUNT")
for model in "${latest_fresh97[@]}"; do
  add_candidate "$model"
done

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "No existing fresh97 selected-eval candidates found."
  exit 1
fi

bash src/scripts/fresh91_selected_eval.sh "${CANDIDATES[@]}"