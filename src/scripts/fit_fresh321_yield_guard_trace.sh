#!/bin/bash
# fresh321: trace-fit close-range yield compliance from fresh320b.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh320b_pairwise_guard_from_fresh318_step1260.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh321_yield_guard_tracefit_from_fresh320b.pt}"
TRACE_DIR="${TRACE_DIR:-/tmp/fresh321_yield_guard_trace}"
SEEDS="${SEEDS:-1460 1461}"
ATTEMPTS="${ATTEMPTS:-2}"
BASE_DOMAIN="${BASE_DOMAIN:-170}"
STEPS="${STEPS:-1200}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-240.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-220.0}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-1800}"
TRACE_STRIDE="${TRACE_STRIDE:-1}"
EPOCHS="${EPOCHS:-150}"
LEARNING_RATE="${LEARNING_RATE:-3.0e-6}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-18.0}"
DECONF_WEIGHT="${DECONF_WEIGHT:-2.10}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-2.60}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.020}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.18}"

mkdir -p "$TRACE_DIR" "$(dirname "$OUTPUT")"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi

trace_args=()
anchor_args=()
domain_index=0
for attempt in $(seq 1 "$ATTEMPTS"); do
  for seed in $SEEDS; do
    json="$TRACE_DIR/fresh320b_seed${seed}_attempt${attempt}_raw_trace.json"
    trace_args+=(--trace-json "$json")
    anchor_args+=(--anchor-trace-json "$json")
    if [[ "${SKIP_TRACE:-0}" != "1" || ! -s "$json" ]]; then
      domain=$((BASE_DOMAIN + domain_index))
      echo "----- fresh321 source trace seed=${seed} attempt=${attempt} domain=${domain} -----"
      rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
      ROS_DOMAIN_ID="$domain" timeout "$EVAL_TIMEOUT" /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
        --policy mappo \
        --model "$SOURCE_MODEL" \
        --episodes 1 \
        --steps-per-episode "$STEPS" \
        --device cpu \
        --episode-timeout "$EPISODE_TIMEOUT" \
        --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
        --seed "$seed" \
        --scenario three_usv_random_encounter \
        --output-json "$json" \
        --trace-stride "$TRACE_STRIDE" \
        --trace-raw-observation
    fi
    domain_index=$((domain_index + 1))
  done
done

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  "${trace_args[@]}" \
  "${anchor_args[@]}" \
  --output "$OUTPUT" \
  --target-priority deconf \
  --target-shape-kinds deconf \
  --epochs "$EPOCHS" \
  --batch-size 64 \
  --learning-rate "$LEARNING_RATE" \
  --max-grad-norm "$MAX_GRAD_NORM" \
  --min-error-norm "$MIN_ERROR_NORM" \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --anchor-batch-size 512 \
  --deconf-weight "$DECONF_WEIGHT" \
  --deconf-linear-blend 1.0 \
  --deconf-omega-blend 0.80 \
  --low-speed-source-threshold 0.13 \
  --low-speed-weight 1.60 \
  --low-speed-weight-kinds deconf \
  --risk-guard-weight "$RISK_GUARD_WEIGHT" \
  --risk-guard-yield-only \
  --risk-guard-require-threat \
  --risk-guard-min-threat 0.10 \
  --risk-guard-max-separation 2.35 \
  --risk-guard-max-linear 0.09 \
  --risk-guard-min-starboard-omega 0.20 \
  --risk-guard-min-starboard-threat 0.12 \
  --risk-guard-min-distance 1.20 \
  --device cpu \
  "$@"

echo "fresh321 yield-guard trace fit saved: $OUTPUT"