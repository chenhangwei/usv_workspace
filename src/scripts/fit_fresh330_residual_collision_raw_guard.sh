#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source ../install/setup.bash
set -u
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh329_balanced_collision_raw_guard_from_fresh321.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh330_residual_collision_raw_guard_from_fresh329.pt}"
TRACE_1461_POS="${TRACE_1461_POS:-/tmp/fresh321_seed1461_d207_collect/repeat3_seed1461/seed1461.json}"
TRACE_1460_NEG_A="${TRACE_1460_NEG_A:-/tmp/fresh328_seed1460_d206_attempts/attempt2/repeat1_seed1460/seed1460.json}"
TRACE_1460_NEG_B="${TRACE_1460_NEG_B:-/tmp/fresh328_seed1460_d206_attempts/attempt3/repeat1_seed1460/seed1460.json}"
TRACE_1460_NEG_C="${TRACE_1460_NEG_C:-/tmp/fresh328_seed1460_d206_attempts/attempt5/repeat1_seed1460/seed1460.json}"
TRACE_1460_NEG_D="${TRACE_1460_NEG_D:-/tmp/fresh329_direction_check/seed1460_attempt2/repeat1_seed1460/seed1460.json}"
EPOCHS="${EPOCHS:-55}"
LEARNING_RATE="${LEARNING_RATE:-1.8e-7}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-320.0}"
DECONF_WEIGHT="${DECONF_WEIGHT:-0.85}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-0.75}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.008}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.030}"
RISK_GUARD_MAX_LINEAR="${RISK_GUARD_MAX_LINEAR:-0.055}"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi

for trace in "$TRACE_1461_POS" "$TRACE_1460_NEG_A" "$TRACE_1460_NEG_B" "$TRACE_1460_NEG_C" "$TRACE_1460_NEG_D"; do
  if [[ ! -f "$trace" ]]; then
    echo "missing collision raw trace: $trace" >&2
    exit 1
  fi
done

anchor_args=()
for trace in \
  /tmp/fresh329_direction_check/seed1460_attempt1/repeat1_seed1460/seed1460.json \
  /tmp/fresh329_direction_check/seed1460_attempt3/repeat1_seed1460/seed1460.json \
  /tmp/fresh329_direction_check/seed1460_attempt4/repeat1_seed1460/seed1460.json \
  /tmp/fresh329_direction_check/seed1461_attempt1/repeat1_seed1461/seed1461.json \
  /tmp/fresh329_direction_check/seed1461_attempt2/repeat1_seed1461/seed1461.json \
  /tmp/fresh329_direction_check/seed1461_attempt3/repeat1_seed1461/seed1461.json \
  /tmp/fresh329_direction_check/seed1461_attempt4/repeat1_seed1461/seed1461.json \
  /tmp/fresh321_collision_collect_hard4/repeat1_seed1458/seed1458.json \
  /tmp/fresh321_collision_collect_hard4/repeat1_seed1460/seed1460.json \
  /tmp/fresh321_collision_collect_hard4/repeat1_seed1461/seed1461.json \
  /tmp/fresh321_collision_collect_hard4/repeat1_seed1463/seed1463.json; do
  if [[ -f "$trace" ]]; then
    anchor_args+=(--anchor-trace-json "$trace")
  fi
done

mkdir -p "$(dirname "$OUTPUT")"

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  --trace-json "$TRACE_1461_POS" \
  --trace-json "$TRACE_1460_NEG_A" \
  --trace-json "$TRACE_1460_NEG_B" \
  --trace-json "$TRACE_1460_NEG_C" \
  --trace-json "$TRACE_1460_NEG_D" \
  "${anchor_args[@]}" \
  --output "$OUTPUT" \
  --epochs "$EPOCHS" \
  --batch-size 32 \
  --learning-rate "$LEARNING_RATE" \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --anchor-batch-size 256 \
  --max-grad-norm "$MAX_GRAD_NORM" \
  --min-error-norm "$MIN_ERROR_NORM" \
  --target-priority deconf \
  --target-shape-kinds deconf,guard \
  --deconf-weight "$DECONF_WEIGHT" \
  --deconf-omega-blend 1.0 \
  --risk-guard-weight "$RISK_GUARD_WEIGHT" \
  --risk-guard-yield-only \
  --risk-guard-require-threat \
  --risk-guard-min-threat 0.35 \
  --risk-guard-max-separation 1.55 \
  --risk-guard-max-linear "$RISK_GUARD_MAX_LINEAR" \
  --risk-guard-min-starboard-omega 0.0 \
  --risk-guard-min-distance 0.40

echo "$OUTPUT"