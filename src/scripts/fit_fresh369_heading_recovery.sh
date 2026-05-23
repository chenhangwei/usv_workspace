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

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh367_navigation_aware_avoidance_from_fresh352.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh369_heading_recovery_from_fresh367.pt}"
EPOCHS="${EPOCHS:-120}"
LEARNING_RATE="${LEARNING_RATE:-1.0e-4}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-130.0}"
ANCHOR_BATCH_SIZE="${ANCHOR_BATCH_SIZE:-1024}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.24}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-999.0}"
TRAIN_ALL_ACTOR="${TRAIN_ALL_ACTOR:-1}"
TARGET_LINEAR_BLEND="${TARGET_LINEAR_BLEND:-0.85}"
TARGET_OMEGA_BLEND="${TARGET_OMEGA_BLEND:-0.98}"

TRACE_DIRS="${TRACE_DIRS:-/tmp/fresh367_guard1463_stride20_700 /tmp/fresh368_guard1463_stride20_700 /tmp/fresh331_baseline_seed1456_event_r12_20260518_v1 /tmp/fresh331_baseline_fixed1460_1461_event_r2_20260518_v1 /tmp/fresh345_seed1456_domain220_r12_v1 /tmp/fresh346_seed1456_domain220_r6_v1}"
ANCHOR_DIRS="${ANCHOR_DIRS:-/tmp/fresh367_guard1463_stride20_700 /tmp/fresh368_guard1463_stride20_700 /tmp/fresh331_baseline_seed1456_event_r12_20260518_v1 /tmp/fresh331_baseline_fixed1460_1461_event_r2_20260518_v1 /tmp/fresh331_baseline_guard1458_1463_event_r3_20260518_v1 /tmp/fresh331_fixed_eventwin_r4_v1}"
PRESERVE_DIRS="${PRESERVE_DIRS:-/tmp/fresh331_baseline_fixed1460_1461_event_r2_20260518_v1 /tmp/fresh331_baseline_guard1458_1463_event_r3_20260518_v1 /tmp/fresh331_fixed_eventwin_r4_v1 /tmp/fresh367_seed1456_domain220_r3}"

PRESERVE_ACTION_WEIGHT="${PRESERVE_ACTION_WEIGHT:-14.0}"
PRESERVE_MIN_ROUTE_PROGRESS="${PRESERVE_MIN_ROUTE_PROGRESS:-0.05}"
PRESERVE_MAX_ROUTE_PROGRESS="${PRESERVE_MAX_ROUTE_PROGRESS:-1.10}"
PRESERVE_MIN_TEAM_SEPARATION="${PRESERVE_MIN_TEAM_SEPARATION:-1.15}"
PRESERVE_MIN_EPISODE_PROGRESS="${PRESERVE_MIN_EPISODE_PROGRESS:-0.05}"

RECOVERY_TURN_WEIGHT="${RECOVERY_TURN_WEIGHT:-14.0}"
RECOVERY_TURN_TARGET_LINEAR="${RECOVERY_TURN_TARGET_LINEAR:-0.04}"
RECOVERY_TURN_MIN_DISTANCE="${RECOVERY_TURN_MIN_DISTANCE:-2.0}"
RECOVERY_TURN_MIN_ABS_CTE="${RECOVERY_TURN_MIN_ABS_CTE:-1.10}"
RECOVERY_TURN_MIN_HEADING_ERROR="${RECOVERY_TURN_MIN_HEADING_ERROR:-1.10}"
RECOVERY_TURN_MIN_SEPARATION="${RECOVERY_TURN_MIN_SEPARATION:-2.4}"
RECOVERY_TURN_MAX_THREAT="${RECOVERY_TURN_MAX_THREAT:-0.22}"
RECOVERY_TURN_MIN_STEP="${RECOVERY_TURN_MIN_STEP:-120}"
RECOVERY_TURN_MIN_ROUTE_PROGRESS="${RECOVERY_TURN_MIN_ROUTE_PROGRESS:-0.0}"
RECOVERY_TURN_MAX_ROUTE_PROGRESS="${RECOVERY_TURN_MAX_ROUTE_PROGRESS:-0.98}"
RECOVERY_TURN_OMEGA_BLEND="${RECOVERY_TURN_OMEGA_BLEND:-1.0}"
RECOVERY_TURN_MIN_OMEGA_ABS="${RECOVERY_TURN_MIN_OMEGA_ABS:-0.22}"
RECOVERY_TURN_HEADING_GAIN="${RECOVERY_TURN_HEADING_GAIN:-0.0}"
RECOVERY_TURN_MAX_OMEGA="${RECOVERY_TURN_MAX_OMEGA:-0.45}"

RECOVERY_SPEEDUP_WEIGHT="${RECOVERY_SPEEDUP_WEIGHT:-3.0}"
RECOVERY_SPEEDUP_SOURCE_THRESHOLD="${RECOVERY_SPEEDUP_SOURCE_THRESHOLD:-0.30}"
RECOVERY_SPEEDUP_TARGET_LINEAR="${RECOVERY_SPEEDUP_TARGET_LINEAR:-0.30}"
RECOVERY_SPEEDUP_MIN_DISTANCE="${RECOVERY_SPEEDUP_MIN_DISTANCE:-2.0}"
RECOVERY_SPEEDUP_MAX_CTE="${RECOVERY_SPEEDUP_MAX_CTE:-1.10}"
RECOVERY_SPEEDUP_MIN_SEPARATION="${RECOVERY_SPEEDUP_MIN_SEPARATION:-2.4}"
RECOVERY_SPEEDUP_MAX_THREAT="${RECOVERY_SPEEDUP_MAX_THREAT:-0.16}"
RECOVERY_SPEEDUP_MIN_STEP="${RECOVERY_SPEEDUP_MIN_STEP:-120}"
RECOVERY_SPEEDUP_MIN_ROUTE_PROGRESS="${RECOVERY_SPEEDUP_MIN_ROUTE_PROGRESS:-0.0}"
RECOVERY_SPEEDUP_MAX_ROUTE_PROGRESS="${RECOVERY_SPEEDUP_MAX_ROUTE_PROGRESS:-1.10}"
RECOVERY_SPEEDUP_MAX_HEADING_ERROR="${RECOVERY_SPEEDUP_MAX_HEADING_ERROR:-0.90}"
RECOVERY_SPEEDUP_CTE_OMEGA_BLEND="${RECOVERY_SPEEDUP_CTE_OMEGA_BLEND:-0.85}"

RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-2.4}"
RISK_GUARD_REQUIRE_THREAT="${RISK_GUARD_REQUIRE_THREAT:-0}"
RISK_GUARD_MIN_THREAT="${RISK_GUARD_MIN_THREAT:-0.20}"
RISK_GUARD_MAX_SEPARATION="${RISK_GUARD_MAX_SEPARATION:-2.40}"
RISK_GUARD_MAX_LINEAR="${RISK_GUARD_MAX_LINEAR:-0.13}"
RISK_GUARD_YIELD_MAX_LINEAR="${RISK_GUARD_YIELD_MAX_LINEAR:-0.05}"
RISK_GUARD_STANDON_MAX_LINEAR="${RISK_GUARD_STANDON_MAX_LINEAR:-0.15}"
RISK_GUARD_OMEGA_BLEND="${RISK_GUARD_OMEGA_BLEND:-0.80}"
RISK_GUARD_YIELD_MIN_OMEGA_ABS="${RISK_GUARD_YIELD_MIN_OMEGA_ABS:-0.30}"
RISK_GUARD_STANDON_MIN_OMEGA_ABS="${RISK_GUARD_STANDON_MIN_OMEGA_ABS:-0.08}"
RISK_GUARD_MIN_DISTANCE="${RISK_GUARD_MIN_DISTANCE:-0.35}"
RISK_GUARD_MIN_ROUTE_PROGRESS="${RISK_GUARD_MIN_ROUTE_PROGRESS:-0.0}"
RISK_GUARD_MAX_ROUTE_PROGRESS="${RISK_GUARD_MAX_ROUTE_PROGRESS:-1.08}"
RISK_GUARD_MIN_ABS_CTE="${RISK_GUARD_MIN_ABS_CTE:-0.0}"

CLEAR_SPEEDUP_WEIGHT="${CLEAR_SPEEDUP_WEIGHT:-0.4}"
LATE_CLEAR_WEIGHT="${LATE_CLEAR_WEIGHT:-0.8}"

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

require_threat_args=()
if [[ "$RISK_GUARD_REQUIRE_THREAT" == "1" || "$RISK_GUARD_REQUIRE_THREAT" == "true" ]]; then
  require_threat_args+=(--risk-guard-require-threat)
fi

train_all_args=()
if [[ "$TRAIN_ALL_ACTOR" == "1" || "$TRAIN_ALL_ACTOR" == "true" ]]; then
  train_all_args+=(--train-all-actor)
fi

mkdir -p "$(dirname "$OUTPUT")"

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  "${trace_args[@]}" \
  "${anchor_args[@]}" \
  "${preserve_args[@]}" \
  --output "$OUTPUT" \
  --epochs "$EPOCHS" \
  --batch-size 96 \
  --learning-rate "$LEARNING_RATE" \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --anchor-batch-size "$ANCHOR_BATCH_SIZE" \
  --max-grad-norm "$MAX_GRAD_NORM" \
  --min-error-norm "$MIN_ERROR_NORM" \
  "${train_all_args[@]}" \
  --target-priority deconf \
  --target-shape-kinds guard,recovery_turn,recovery,late_clear \
  --target-linear-blend "$TARGET_LINEAR_BLEND" \
  --target-omega-blend "$TARGET_OMEGA_BLEND" \
  --deconf-weight 0.0 \
  --offroute-weight 0.0 \
  --cte-weight 0.0 \
  --finish-weight 0.0 \
  --preserve-action-weight "$PRESERVE_ACTION_WEIGHT" \
  --preserve-min-route-progress "$PRESERVE_MIN_ROUTE_PROGRESS" \
  --preserve-max-route-progress "$PRESERVE_MAX_ROUTE_PROGRESS" \
  --preserve-min-team-separation "$PRESERVE_MIN_TEAM_SEPARATION" \
  --preserve-min-episode-progress "$PRESERVE_MIN_EPISODE_PROGRESS" \
  --preserve-exclude-collisions \
  --recovery-turn-weight "$RECOVERY_TURN_WEIGHT" \
  --recovery-turn-target-linear "$RECOVERY_TURN_TARGET_LINEAR" \
  --recovery-turn-min-distance "$RECOVERY_TURN_MIN_DISTANCE" \
  --recovery-turn-min-abs-cte "$RECOVERY_TURN_MIN_ABS_CTE" \
  --recovery-turn-min-heading-error "$RECOVERY_TURN_MIN_HEADING_ERROR" \
  --recovery-turn-min-separation "$RECOVERY_TURN_MIN_SEPARATION" \
  --recovery-turn-max-threat "$RECOVERY_TURN_MAX_THREAT" \
  --recovery-turn-min-step "$RECOVERY_TURN_MIN_STEP" \
  --recovery-turn-min-route-progress "$RECOVERY_TURN_MIN_ROUTE_PROGRESS" \
  --recovery-turn-max-route-progress "$RECOVERY_TURN_MAX_ROUTE_PROGRESS" \
  --recovery-turn-omega-blend "$RECOVERY_TURN_OMEGA_BLEND" \
  --recovery-turn-min-omega-abs "$RECOVERY_TURN_MIN_OMEGA_ABS" \
  --recovery-turn-heading-gain "$RECOVERY_TURN_HEADING_GAIN" \
  --recovery-turn-max-omega "$RECOVERY_TURN_MAX_OMEGA" \
  --recovery-speedup-weight "$RECOVERY_SPEEDUP_WEIGHT" \
  --recovery-speedup-source-threshold "$RECOVERY_SPEEDUP_SOURCE_THRESHOLD" \
  --recovery-speedup-target-linear "$RECOVERY_SPEEDUP_TARGET_LINEAR" \
  --recovery-speedup-min-distance "$RECOVERY_SPEEDUP_MIN_DISTANCE" \
  --recovery-speedup-max-cte "$RECOVERY_SPEEDUP_MAX_CTE" \
  --recovery-speedup-min-separation "$RECOVERY_SPEEDUP_MIN_SEPARATION" \
  --recovery-speedup-max-threat "$RECOVERY_SPEEDUP_MAX_THREAT" \
  --recovery-speedup-min-step "$RECOVERY_SPEEDUP_MIN_STEP" \
  --recovery-speedup-min-route-progress "$RECOVERY_SPEEDUP_MIN_ROUTE_PROGRESS" \
  --recovery-speedup-max-route-progress "$RECOVERY_SPEEDUP_MAX_ROUTE_PROGRESS" \
  --recovery-speedup-max-heading-error "$RECOVERY_SPEEDUP_MAX_HEADING_ERROR" \
  --recovery-speedup-cte-omega-blend "$RECOVERY_SPEEDUP_CTE_OMEGA_BLEND" \
  --clear-speedup-weight "$CLEAR_SPEEDUP_WEIGHT" \
  --late-clear-weight "$LATE_CLEAR_WEIGHT" \
  --late-clear-allow-deconf \
  --risk-guard-weight "$RISK_GUARD_WEIGHT" \
  "${require_threat_args[@]}" \
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
