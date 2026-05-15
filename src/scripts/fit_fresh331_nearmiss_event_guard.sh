#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source ../install/setup.bash
set -u
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh329_balanced_collision_raw_guard_from_fresh321.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh331_nearmiss_event_guard_from_fresh329.pt}"
EPOCHS="${EPOCHS:-70}"
LEARNING_RATE="${LEARNING_RATE:-3.5e-7}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-260.0}"
DECONF_WEIGHT="${DECONF_WEIGHT:-1.15}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-0.90}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.012}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.035}"
RISK_GUARD_MAX_LINEAR="${RISK_GUARD_MAX_LINEAR:-0.050}"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi

trace_args=()
for trace in \
  /tmp/fresh329_nearmiss_fixed_v1/seed1460_attempt1/repeat1_seed1460/seed1460.json \
  /tmp/fresh329_nearmiss_fixed_v1/seed1460_attempt2/repeat1_seed1460/seed1460.json \
  /tmp/fresh329_nearmiss_fixed_v1/seed1460_attempt3/repeat1_seed1460/seed1460.json \
  /tmp/fresh329_nearmiss_fixed_v1/seed1460_attempt4/repeat1_seed1460/seed1460.json \
  /tmp/fresh329_nearmiss_guard_v1/repeat3_seed1458/seed1458.json; do
  if [[ ! -f "$trace" ]]; then
    echo "missing near-miss trace: $trace" >&2
    exit 1
  fi
  trace_args+=(--trace-json "$trace")
done

anchor_args=()
for trace in \
  /tmp/fresh329_nearmiss_fixed_v1/seed1461_attempt1/repeat1_seed1461/seed1461.json \
  /tmp/fresh329_nearmiss_fixed_v1/seed1461_attempt2/repeat1_seed1461/seed1461.json \
  /tmp/fresh329_nearmiss_fixed_v1/seed1461_attempt3/repeat1_seed1461/seed1461.json \
  /tmp/fresh329_nearmiss_fixed_v1/seed1461_attempt4/repeat1_seed1461/seed1461.json \
  /tmp/fresh329_nearmiss_guard_v1/repeat1_seed1458/seed1458.json \
  /tmp/fresh329_nearmiss_guard_v1/repeat2_seed1458/seed1458.json \
  /tmp/fresh329_nearmiss_guard_v1/repeat1_seed1463/seed1463.json \
  /tmp/fresh329_nearmiss_guard_v1/repeat2_seed1463/seed1463.json \
  /tmp/fresh329_nearmiss_guard_v1/repeat3_seed1463/seed1463.json \
  /tmp/fresh329_direction_check/seed1460_attempt1/repeat1_seed1460/seed1460.json \
  /tmp/fresh329_direction_check/seed1460_attempt3/repeat1_seed1460/seed1460.json \
  /tmp/fresh329_direction_check/seed1460_attempt4/repeat1_seed1460/seed1460.json \
  /tmp/fresh329_direction_check/seed1461_attempt1/repeat1_seed1461/seed1461.json \
  /tmp/fresh329_direction_check/seed1461_attempt2/repeat1_seed1461/seed1461.json \
  /tmp/fresh329_direction_check/seed1461_attempt3/repeat1_seed1461/seed1461.json \
  /tmp/fresh329_direction_check/seed1461_attempt4/repeat1_seed1461/seed1461.json; do
  if [[ -f "$trace" ]]; then
    anchor_args+=(--anchor-trace-json "$trace")
  fi
done

mkdir -p "$(dirname "$OUTPUT")"

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  "${trace_args[@]}" \
  "${anchor_args[@]}" \
  --output "$OUTPUT" \
  --epochs "$EPOCHS" \
  --batch-size 48 \
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
  --risk-guard-min-threat 0.30 \
  --risk-guard-max-separation 1.55 \
  --risk-guard-max-linear "$RISK_GUARD_MAX_LINEAR" \
  --risk-guard-min-starboard-omega 0.0 \
  --risk-guard-min-distance 0.40

echo "$OUTPUT"