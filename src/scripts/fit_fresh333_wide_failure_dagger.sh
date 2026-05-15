#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source ../install/setup.bash
set -u
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh331_nearmiss_event_guard_from_fresh329.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh333_wide_failure_dagger_from_fresh331.pt}"
EPOCHS="${EPOCHS:-45}"
LEARNING_RATE="${LEARNING_RATE:-9.0e-8}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-820.0}"
DECONF_WEIGHT="${DECONF_WEIGHT:-0.42}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-0.55}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.020}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.020}"
DECONF_LINEAR_BLEND="${DECONF_LINEAR_BLEND:-0.35}"
DECONF_OMEGA_BLEND="${DECONF_OMEGA_BLEND:-0.45}"
RISK_GUARD_MAX_LINEAR="${RISK_GUARD_MAX_LINEAR:-0.050}"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi

target_args=()
for trace in \
  /tmp/fresh331_hard9_v1/repeat1_seed1456/seed1456.json \
  /tmp/fresh331_fixed_gate_v1/repeat4_seed1460/seed1460.json \
  /tmp/fresh331_wide_fixed_v1/repeat2_seed1461/seed1461.json; do
  if [[ ! -f "$trace" ]]; then
    echo "missing failure trace: $trace" >&2
    exit 1
  fi
  target_args+=(--trace-json "$trace")
done

anchor_args=()
for trace in \
  /tmp/fresh331_wide_fixed_v1/repeat1_seed1460/seed1460.json \
  /tmp/fresh331_wide_fixed_v1/repeat2_seed1460/seed1460.json \
  /tmp/fresh331_wide_fixed_v1/repeat3_seed1460/seed1460.json \
  /tmp/fresh331_wide_fixed_v1/repeat4_seed1460/seed1460.json \
  /tmp/fresh331_wide_fixed_v1/repeat1_seed1461/seed1461.json \
  /tmp/fresh331_wide_fixed_v1/repeat3_seed1461/seed1461.json \
  /tmp/fresh331_wide_fixed_v1/repeat4_seed1461/seed1461.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1456/seed1456.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1457/seed1457.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1458/seed1458.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1459/seed1459.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1460/seed1460.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1461/seed1461.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1462/seed1462.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1463/seed1463.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1464/seed1464.json \
  /tmp/fresh331_guard_gate_v1/repeat1_seed1458/seed1458.json \
  /tmp/fresh331_guard_gate_v1/repeat2_seed1458/seed1458.json \
  /tmp/fresh331_guard_gate_v1/repeat3_seed1458/seed1458.json \
  /tmp/fresh331_guard_gate_v1/repeat1_seed1463/seed1463.json \
  /tmp/fresh331_guard_gate_v1/repeat2_seed1463/seed1463.json \
  /tmp/fresh331_guard_gate_v1/repeat3_seed1463/seed1463.json; do
  if [[ -f "$trace" ]]; then
    anchor_args+=(--anchor-trace-json "$trace")
  fi
done

mkdir -p "$(dirname "$OUTPUT")"

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  "${target_args[@]}" \
  "${anchor_args[@]}" \
  --output "$OUTPUT" \
  --epochs "$EPOCHS" \
  --batch-size 48 \
  --learning-rate "$LEARNING_RATE" \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --anchor-batch-size 512 \
  --max-grad-norm "$MAX_GRAD_NORM" \
  --min-error-norm "$MIN_ERROR_NORM" \
  --target-priority deconf \
  --target-shape-kinds deconf,guard \
  --deconf-weight "$DECONF_WEIGHT" \
  --deconf-linear-blend "$DECONF_LINEAR_BLEND" \
  --deconf-omega-blend "$DECONF_OMEGA_BLEND" \
  --risk-guard-weight "$RISK_GUARD_WEIGHT" \
  --risk-guard-yield-only \
  --risk-guard-require-threat \
  --risk-guard-min-threat 0.25 \
  --risk-guard-max-separation 1.55 \
  --risk-guard-max-linear "$RISK_GUARD_MAX_LINEAR" \
  --risk-guard-min-starboard-omega 0.0 \
  --risk-guard-min-distance 0.35

echo "$OUTPUT"