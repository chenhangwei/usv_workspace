#!/bin/bash
# fresh315: trace-fit single_usv_overtaking scripted return targets from fresh313 failures.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh313_single_overtake_early_return_from_fresh310.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh315_single_overtake_tracefit_from_fresh313.pt}"
TRACE_DIR="${TRACE_DIR:-/tmp/fresh315_single_overtake_trace}"
BASE_DOMAIN="${BASE_DOMAIN:-210}"
STEPS="${STEPS:-800}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-145.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-108.0}"
SEEDS="${SEEDS:-3071 3072 3073 3074}"
EPOCHS="${EPOCHS:-180}"
LEARNING_RATE="${LEARNING_RATE:-1.2e-5}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-2.2}"
SCRIPTED_WEIGHT="${SCRIPTED_WEIGHT:-5.5}"
TRACE_STRIDE="${TRACE_STRIDE:-5}"

mkdir -p "$TRACE_DIR" "$(dirname "$OUTPUT")"

trace_args=()
index=0
for seed in $SEEDS; do
  json="$TRACE_DIR/fresh313_single_overtake_seed${seed}_raw_trace.json"
  trace_args+=(--trace-json "$json")
  if [[ "${SKIP_TRACE:-0}" != "1" ]]; then
    echo "----- fresh315 trace seed=${seed} domain=$((BASE_DOMAIN + index)) -----"
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    ROS_DOMAIN_ID="$((BASE_DOMAIN + index))" timeout 1800 /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
      --policy mappo \
      --model "$SOURCE_MODEL" \
      --episodes 1 \
      --steps-per-episode "$STEPS" \
      --device cpu \
      --episode-timeout "$EPISODE_TIMEOUT" \
      --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
      --seed "$seed" \
      --output-json "$json" \
      --scenario single_usv_overtaking \
      --trace-stride "$TRACE_STRIDE" \
      --trace-raw-observation
  fi
  index=$((index + 1))
done

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  "${trace_args[@]}" \
  --output "$OUTPUT" \
  --epochs "$EPOCHS" \
  --batch-size 96 \
  --learning-rate "$LEARNING_RATE" \
  --max-grad-norm 0.35 \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --anchor-batch-size 256 \
  --min-error-norm 0.00 \
  --target-priority deconf,offroute,cte,finish \
  --target-shape-kinds scripted_overtake \
  --target-linear-blend 1.0 \
  --target-omega-blend 1.0 \
  --scripted-overtake-weight "$SCRIPTED_WEIGHT" \
  --scripted-overtake-scenario single_usv_overtaking \
  --scripted-overtake-min-distance 0.8 \
  --scripted-overtake-max-distance 28.0 \
  --scripted-overtake-rear-approach-speed 0.18 \
  --scripted-overtake-rear-close-speed 0.075 \
  --scripted-overtake-rear-pass-speed 0.34 \
  --scripted-overtake-close-separation 2.25 \
  --scripted-overtake-release-separation 28.0 \
  --scripted-overtake-target-starboard-offset 0.78 \
  --scripted-overtake-rear-omega 0.24 \
  --scripted-overtake-omega-weight-scale 1.70 \
  --scripted-overtake-single-return-progress 0.46 \
  --scripted-overtake-single-return-rel-x 0.05 \
  --scripted-overtake-single-return-min-separation 2.05 \
  --scripted-overtake-single-return-cte-start 0.32 \
  --scripted-overtake-single-return-cte-full 2.40 \
  --scripted-overtake-single-return-speed 0.34 \
  --scripted-overtake-single-return-omega 0.50 \
  --scripted-overtake-single-finish-distance 5.00 \
  --scripted-overtake-single-finish-max-omega 0.07 \
  "$@"