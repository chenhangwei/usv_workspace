#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ -f ../install/setup.bash ]]; then
  source ../install/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
set -u
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh331_nearmiss_event_guard_from_fresh329.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh345_role_risk_guard_from_fresh331.pt}"
EPOCHS="${EPOCHS:-36}"
LEARNING_RATE="${LEARNING_RATE:-5.0e-8}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-1800.0}"
ANCHOR_BATCH_SIZE="${ANCHOR_BATCH_SIZE:-1024}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.014}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-999.0}"

TRACE_DIRS="${TRACE_DIRS:-/tmp/fresh331_baseline_seed1456_event_r12_20260518_v1 /tmp/fresh331_seed1456_eventwin_r6_v1}"
ANCHOR_DIRS="${ANCHOR_DIRS:-/tmp/fresh331_baseline_seed1456_event_r12_20260518_v1 /tmp/fresh331_baseline_fixed1460_1461_event_r2_20260518_v1 /tmp/fresh331_fixed_eventwin_r4_v1 /tmp/fresh331_guard_gate_v1 /tmp/fresh331_wide_hard9_v1}"
PRESERVE_DIRS="${PRESERVE_DIRS:-/tmp/fresh331_baseline_fixed1460_1461_event_r2_20260518_v1 /tmp/fresh331_fixed_eventwin_r4_v1}"
PRESERVE_ACTION_WEIGHT="${PRESERVE_ACTION_WEIGHT:-4.0}"
PRESERVE_MIN_ROUTE_PROGRESS="${PRESERVE_MIN_ROUTE_PROGRESS:-0.70}"
PRESERVE_MAX_ROUTE_PROGRESS="${PRESERVE_MAX_ROUTE_PROGRESS:-1.10}"
PRESERVE_MIN_TEAM_SEPARATION="${PRESERVE_MIN_TEAM_SEPARATION:-1.05}"

RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-0.34}"
RISK_GUARD_MIN_THREAT="${RISK_GUARD_MIN_THREAT:-0.65}"
RISK_GUARD_MAX_SEPARATION="${RISK_GUARD_MAX_SEPARATION:-1.60}"
RISK_GUARD_MAX_LINEAR="${RISK_GUARD_MAX_LINEAR:-0.12}"
RISK_GUARD_YIELD_MAX_LINEAR="${RISK_GUARD_YIELD_MAX_LINEAR:-0.0}"
RISK_GUARD_STANDON_MAX_LINEAR="${RISK_GUARD_STANDON_MAX_LINEAR:-0.12}"
RISK_GUARD_OMEGA_BLEND="${RISK_GUARD_OMEGA_BLEND:-1.0}"
RISK_GUARD_YIELD_MIN_OMEGA_ABS="${RISK_GUARD_YIELD_MIN_OMEGA_ABS:-0.36}"
RISK_GUARD_STANDON_MIN_OMEGA_ABS="${RISK_GUARD_STANDON_MIN_OMEGA_ABS:-0.08}"
RISK_GUARD_MIN_DISTANCE="${RISK_GUARD_MIN_DISTANCE:-0.40}"
RISK_GUARD_MIN_ROUTE_PROGRESS="${RISK_GUARD_MIN_ROUTE_PROGRESS:-0.20}"
RISK_GUARD_MAX_ROUTE_PROGRESS="${RISK_GUARD_MAX_ROUTE_PROGRESS:-0.75}"
RISK_GUARD_MIN_ABS_CTE="${RISK_GUARD_MIN_ABS_CTE:-1.20}"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi

append_json_dir() {
  local -n out_array="$1"
  local dir="$2"
  [[ -d "$dir" ]] || return 0
  while IFS= read -r -d '' trace; do
    out_array+=("$trace")
  done < <(find "$dir" -name '*.json' -type f -print0 | sort -z)
}

trace_files=()
for dir in $TRACE_DIRS; do
  append_json_dir trace_files "$dir"
done

anchor_files=()
for dir in $ANCHOR_DIRS; do
  append_json_dir anchor_files "$dir"
done

preserve_files=()
for dir in $PRESERVE_DIRS; do
  append_json_dir preserve_files "$dir"
done

if [[ "${#trace_files[@]}" -eq 0 ]]; then
  echo "missing trace files. Set TRACE_DIRS to event-window JSON directories." >&2
  exit 1
fi

trace_args=()
for trace in "${trace_files[@]}"; do
  trace_args+=(--trace-json "$trace")
done

anchor_args=()
for trace in "${anchor_files[@]}"; do
  anchor_args+=(--anchor-trace-json "$trace")
done

preserve_args=()
for trace in "${preserve_files[@]}"; do
  preserve_args+=(--preserve-trace-json "$trace")
done

mkdir -p "$(dirname "$OUTPUT")"

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  "${trace_args[@]}" \
  "${anchor_args[@]}" \
  "${preserve_args[@]}" \
  --output "$OUTPUT" \
  --epochs "$EPOCHS" \
  --batch-size 64 \
  --learning-rate "$LEARNING_RATE" \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --anchor-batch-size "$ANCHOR_BATCH_SIZE" \
  --max-grad-norm "$MAX_GRAD_NORM" \
  --min-error-norm "$MIN_ERROR_NORM" \
  --target-priority deconf \
  --target-shape-kinds guard \
  --deconf-weight 0.0 \
  --offroute-weight 0.0 \
  --cte-weight 0.0 \
  --finish-weight 0.0 \
  --preserve-action-weight "$PRESERVE_ACTION_WEIGHT" \
  --preserve-min-route-progress "$PRESERVE_MIN_ROUTE_PROGRESS" \
  --preserve-max-route-progress "$PRESERVE_MAX_ROUTE_PROGRESS" \
  --preserve-min-team-separation "$PRESERVE_MIN_TEAM_SEPARATION" \
  --preserve-exclude-collisions \
  --risk-guard-weight "$RISK_GUARD_WEIGHT" \
  --risk-guard-require-threat \
  --risk-guard-min-threat "$RISK_GUARD_MIN_THREAT" \
  --risk-guard-max-separation "$RISK_GUARD_MAX_SEPARATION" \
  --risk-guard-max-linear "$RISK_GUARD_MAX_LINEAR" \
  --risk-guard-yield-max-linear "$RISK_GUARD_YIELD_MAX_LINEAR" \
  --risk-guard-standon-max-linear "$RISK_GUARD_STANDON_MAX_LINEAR" \
  --risk-guard-omega-blend "$RISK_GUARD_OMEGA_BLEND" \
  --risk-guard-yield-min-omega-abs "$RISK_GUARD_YIELD_MIN_OMEGA_ABS" \
  --risk-guard-standon-min-omega-abs "$RISK_GUARD_STANDON_MIN_OMEGA_ABS" \
  --risk-guard-min-distance "$RISK_GUARD_MIN_DISTANCE" \
  --risk-guard-min-route-progress "$RISK_GUARD_MIN_ROUTE_PROGRESS" \
  --risk-guard-max-route-progress "$RISK_GUARD_MAX_ROUTE_PROGRESS" \
  --risk-guard-min-abs-cte "$RISK_GUARD_MIN_ABS_CTE"

echo "$OUTPUT"