#!/bin/bash
# fresh93 selected evaluation: compare fresh92 baseline/checkpoints against fresh93 yield-repair candidates.
#
# This intentionally reuses fresh91_selected_eval.sh so full5 and targeted3 semantics stay identical:
#   - full5: 5 total episodes across the five deployable scenarios
#   - targeted3: 9 total episodes across crossing/overtaking/random

set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh93_selected_eval}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh93_checkpoints}"
export LOG="${LOG:-/tmp/fresh93_selected_eval.log}"
export BASE_DOMAIN="${BASE_DOMAIN:-191}"
export FULL5_EPISODES="${FULL5_EPISODES:-5}"
export TARGETED_EPISODES="${TARGETED_EPISODES:-9}"
export STEPS="${STEPS:-300}"
export EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-70}"
export NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-20}"
export EVAL_TIMEOUT="${EVAL_TIMEOUT:-360}"
export FRESH93_LATEST_COUNT="${FRESH93_LATEST_COUNT:-5}"

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

add_candidate /mnt/data/checkpoints/usv_rl/fresh91_checkpoints/fresh91_full_scenario_trunk_branch_repair_step_0015870.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh92_checkpoints/fresh92_crossing_branch_repair_step_0009792.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh92_crossing_branch_repair.pt
add_candidate /mnt/data/checkpoints/usv_rl/fresh93_crossing_yield_branch_repair.pt

mapfile -t latest_fresh93 < <(ls -1t /mnt/data/checkpoints/usv_rl/fresh93_checkpoints/fresh93_crossing_yield_branch_repair_step_*.pt 2>/dev/null | head -n "$FRESH93_LATEST_COUNT")
for model in "${latest_fresh93[@]}"; do
	add_candidate "$model"
done

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
	echo "No existing fresh93 selected-eval candidates found."
	exit 1
fi

bash src/scripts/fresh91_selected_eval.sh "${CANDIDATES[@]}"
