#!/bin/bash
# Fit fresh186 with cautious train-all actor active-slice replay.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh181_checkpoints/fresh181_pairwise_priority_delta_from_fresh172_step_0001260.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh186_checkpoints/fresh186_train_all_release_recovery_from_fresh181_step1260.pt}"
TRACE_DIR="${TRACE_DIR:-/tmp/fresh182_active_trace}"
SEEDS="${SEEDS:-1460 1462}"
EXTRA_TRACE_JSONS="${EXTRA_TRACE_JSONS:-}"
EPOCHS="${EPOCHS:-220}"
LEARNING_RATE="${LEARNING_RATE:-5.0e-6}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-12.0}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.02}"
DECONF_WEIGHT="${DECONF_WEIGHT:-1.35}"
OFFROUTE_WEIGHT="${OFFROUTE_WEIGHT:-1.20}"
CTE_WEIGHT="${CTE_WEIGHT:-1.35}"
FINISH_WEIGHT="${FINISH_WEIGHT:-0.85}"
TARGET_MIN_LINEAR="${TARGET_MIN_LINEAR:-0.20}"
TARGET_MAX_LINEAR_DROP="${TARGET_MAX_LINEAR_DROP:-0.06}"
TARGET_LINEAR_BLEND="${TARGET_LINEAR_BLEND:-0.50}"
TARGET_OMEGA_BLEND="${TARGET_OMEGA_BLEND:-0.75}"
TARGET_SHAPE_KINDS="${TARGET_SHAPE_KINDS:-deconf,offroute,cte,finish}"
DECONF_MIN_LINEAR="${DECONF_MIN_LINEAR:-0.20}"
DECONF_MAX_LINEAR_DROP="${DECONF_MAX_LINEAR_DROP:-0.06}"
DECONF_LINEAR_BLEND="${DECONF_LINEAR_BLEND:-0.85}"
DECONF_OMEGA_BLEND="${DECONF_OMEGA_BLEND:-0.65}"
LOW_SPEED_SOURCE_THRESHOLD="${LOW_SPEED_SOURCE_THRESHOLD:-0.12}"
LOW_SPEED_WEIGHT="${LOW_SPEED_WEIGHT:-1.0}"
LOW_SPEED_WEIGHT_KINDS="${LOW_SPEED_WEIGHT_KINDS:-deconf,offroute,cte,finish}"
CLEAR_SPEEDUP_WEIGHT="${CLEAR_SPEEDUP_WEIGHT:-0.0}"
CLEAR_SPEEDUP_SOURCE_THRESHOLD="${CLEAR_SPEEDUP_SOURCE_THRESHOLD:-0.13}"
CLEAR_SPEEDUP_TARGET_LINEAR="${CLEAR_SPEEDUP_TARGET_LINEAR:-0.26}"
CLEAR_SPEEDUP_MIN_DISTANCE="${CLEAR_SPEEDUP_MIN_DISTANCE:-2.0}"
CLEAR_SPEEDUP_MAX_CTE="${CLEAR_SPEEDUP_MAX_CTE:-0.8}"
CLEAR_SPEEDUP_MIN_SEPARATION="${CLEAR_SPEEDUP_MIN_SEPARATION:-3.0}"
CLEAR_SPEEDUP_MAX_THREAT="${CLEAR_SPEEDUP_MAX_THREAT:-0.08}"
CLEAR_SPEEDUP_MIN_STEP="${CLEAR_SPEEDUP_MIN_STEP:-0}"
LATE_CLEAR_WEIGHT="${LATE_CLEAR_WEIGHT:-0.0}"
LATE_CLEAR_SOURCE_THRESHOLD="${LATE_CLEAR_SOURCE_THRESHOLD:-0.17}"
LATE_CLEAR_TARGET_LINEAR="${LATE_CLEAR_TARGET_LINEAR:-0.28}"
LATE_CLEAR_MIN_DISTANCE="${LATE_CLEAR_MIN_DISTANCE:-2.5}"
LATE_CLEAR_MAX_CTE="${LATE_CLEAR_MAX_CTE:-2.8}"
LATE_CLEAR_MIN_SEPARATION="${LATE_CLEAR_MIN_SEPARATION:-3.0}"
LATE_CLEAR_MAX_THREAT="${LATE_CLEAR_MAX_THREAT:-0.08}"
LATE_CLEAR_MIN_STEP="${LATE_CLEAR_MIN_STEP:-140}"
LATE_CLEAR_CTE_OMEGA_BLEND="${LATE_CLEAR_CTE_OMEGA_BLEND:-0.6}"
LATE_CLEAR_CTE_SLOW_THRESHOLD="${LATE_CLEAR_CTE_SLOW_THRESHOLD:--1.0}"
LATE_CLEAR_CTE_RECOVERY_LINEAR="${LATE_CLEAR_CTE_RECOVERY_LINEAR:-0.16}"
LATE_CLEAR_ALLOW_DECONF="${LATE_CLEAR_ALLOW_DECONF:-0}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-0.0}"
RISK_GUARD_MIN_THREAT="${RISK_GUARD_MIN_THREAT:-0.08}"
RISK_GUARD_MAX_SEPARATION="${RISK_GUARD_MAX_SEPARATION:-2.8}"
RISK_GUARD_MAX_LINEAR="${RISK_GUARD_MAX_LINEAR:-0.12}"
RISK_GUARD_MIN_STARBOARD_OMEGA="${RISK_GUARD_MIN_STARBOARD_OMEGA:-0.0}"
RISK_GUARD_MIN_STARBOARD_THREAT="${RISK_GUARD_MIN_STARBOARD_THREAT:-0.0}"
RISK_GUARD_MIN_DISTANCE="${RISK_GUARD_MIN_DISTANCE:-2.0}"
RISK_GUARD_MAX_STEP="${RISK_GUARD_MAX_STEP:--1}"
RISK_GUARD_YIELD_ONLY="${RISK_GUARD_YIELD_ONLY:-0}"
RISK_GUARD_REQUIRE_THREAT="${RISK_GUARD_REQUIRE_THREAT:-0}"
GOAL_HOLD_WEIGHT="${GOAL_HOLD_WEIGHT:-0.0}"
GOAL_HOLD_SOURCE_THRESHOLD="${GOAL_HOLD_SOURCE_THRESHOLD:-0.08}"
GOAL_HOLD_MAX_DISTANCE="${GOAL_HOLD_MAX_DISTANCE:-1.8}"
GOAL_HOLD_MAX_CTE="${GOAL_HOLD_MAX_CTE:-1.2}"
GOAL_HOLD_MAX_SEPARATION="${GOAL_HOLD_MAX_SEPARATION:-2.4}"
GOAL_HOLD_MIN_THREAT="${GOAL_HOLD_MIN_THREAT:-0.15}"
GOAL_HOLD_TARGET_LINEAR="${GOAL_HOLD_TARGET_LINEAR:-0.04}"
GOAL_HOLD_OMEGA_BLEND="${GOAL_HOLD_OMEGA_BLEND:-1.0}"

risk_guard_yield_args=()
if [[ "$RISK_GUARD_YIELD_ONLY" == "1" || "$RISK_GUARD_YIELD_ONLY" == "true" ]]; then
  risk_guard_yield_args+=(--risk-guard-yield-only)
fi
if [[ "$RISK_GUARD_REQUIRE_THREAT" == "1" || "$RISK_GUARD_REQUIRE_THREAT" == "true" ]]; then
  risk_guard_yield_args+=(--risk-guard-require-threat)
fi

late_clear_deconf_args=()
if [[ "$LATE_CLEAR_ALLOW_DECONF" == "1" || "$LATE_CLEAR_ALLOW_DECONF" == "true" ]]; then
  late_clear_deconf_args+=(--late-clear-allow-deconf)
fi

trace_args=()
anchor_args=()
for seed in $SEEDS; do
  trace_json="$TRACE_DIR/fresh181_step1260_raw_trace_seed${seed}.json"
  if [[ ! -s "$trace_json" ]]; then
    echo "missing trace json: $trace_json" >&2
    exit 1
  fi
  trace_args+=(--trace-json "$trace_json")
  anchor_args+=(--anchor-trace-json "$trace_json")
done

for trace_json in $EXTRA_TRACE_JSONS; do
  if [[ ! -s "$trace_json" ]]; then
    echo "missing extra trace json: $trace_json" >&2
    exit 1
  fi
  trace_args+=(--trace-json "$trace_json")
  anchor_args+=(--anchor-trace-json "$trace_json")
done

mkdir -p "$(dirname "$OUTPUT")"
/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  "${trace_args[@]}" \
  "${anchor_args[@]}" \
  --output "$OUTPUT" \
  --target-priority deconf,offroute,cte,finish \
  --min-error-norm "$MIN_ERROR_NORM" \
  --epochs "$EPOCHS" \
  --learning-rate "$LEARNING_RATE" \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --deconf-weight "$DECONF_WEIGHT" \
  --offroute-weight "$OFFROUTE_WEIGHT" \
  --cte-weight "$CTE_WEIGHT" \
  --finish-weight "$FINISH_WEIGHT" \
  --target-min-linear "$TARGET_MIN_LINEAR" \
  --target-max-linear-drop "$TARGET_MAX_LINEAR_DROP" \
  --target-linear-blend "$TARGET_LINEAR_BLEND" \
  --target-omega-blend "$TARGET_OMEGA_BLEND" \
  --target-shape-kinds "$TARGET_SHAPE_KINDS" \
  --deconf-min-linear "$DECONF_MIN_LINEAR" \
  --deconf-max-linear-drop "$DECONF_MAX_LINEAR_DROP" \
  --deconf-linear-blend "$DECONF_LINEAR_BLEND" \
  --deconf-omega-blend "$DECONF_OMEGA_BLEND" \
  --low-speed-source-threshold "$LOW_SPEED_SOURCE_THRESHOLD" \
  --low-speed-weight "$LOW_SPEED_WEIGHT" \
  --low-speed-weight-kinds "$LOW_SPEED_WEIGHT_KINDS" \
  --clear-speedup-weight "$CLEAR_SPEEDUP_WEIGHT" \
  --clear-speedup-source-threshold "$CLEAR_SPEEDUP_SOURCE_THRESHOLD" \
  --clear-speedup-target-linear "$CLEAR_SPEEDUP_TARGET_LINEAR" \
  --clear-speedup-min-distance "$CLEAR_SPEEDUP_MIN_DISTANCE" \
  --clear-speedup-max-cte "$CLEAR_SPEEDUP_MAX_CTE" \
  --clear-speedup-min-separation "$CLEAR_SPEEDUP_MIN_SEPARATION" \
  --clear-speedup-max-threat "$CLEAR_SPEEDUP_MAX_THREAT" \
  --clear-speedup-min-step "$CLEAR_SPEEDUP_MIN_STEP" \
  --late-clear-weight "$LATE_CLEAR_WEIGHT" \
  --late-clear-source-threshold "$LATE_CLEAR_SOURCE_THRESHOLD" \
  --late-clear-target-linear "$LATE_CLEAR_TARGET_LINEAR" \
  --late-clear-min-distance "$LATE_CLEAR_MIN_DISTANCE" \
  --late-clear-max-cte "$LATE_CLEAR_MAX_CTE" \
  --late-clear-min-separation "$LATE_CLEAR_MIN_SEPARATION" \
  --late-clear-max-threat "$LATE_CLEAR_MAX_THREAT" \
  --late-clear-min-step "$LATE_CLEAR_MIN_STEP" \
  --late-clear-cte-omega-blend "$LATE_CLEAR_CTE_OMEGA_BLEND" \
  --late-clear-cte-slow-threshold "$LATE_CLEAR_CTE_SLOW_THRESHOLD" \
  --late-clear-cte-recovery-linear "$LATE_CLEAR_CTE_RECOVERY_LINEAR" \
  "${late_clear_deconf_args[@]}" \
  --risk-guard-weight "$RISK_GUARD_WEIGHT" \
  --risk-guard-min-threat "$RISK_GUARD_MIN_THREAT" \
  --risk-guard-max-separation "$RISK_GUARD_MAX_SEPARATION" \
  --risk-guard-max-linear "$RISK_GUARD_MAX_LINEAR" \
  --risk-guard-min-starboard-omega "$RISK_GUARD_MIN_STARBOARD_OMEGA" \
  --risk-guard-min-starboard-threat "$RISK_GUARD_MIN_STARBOARD_THREAT" \
  --risk-guard-min-distance "$RISK_GUARD_MIN_DISTANCE" \
  --risk-guard-max-step "$RISK_GUARD_MAX_STEP" \
  "${risk_guard_yield_args[@]}" \
  --goal-hold-weight "$GOAL_HOLD_WEIGHT" \
  --goal-hold-source-threshold "$GOAL_HOLD_SOURCE_THRESHOLD" \
  --goal-hold-max-distance "$GOAL_HOLD_MAX_DISTANCE" \
  --goal-hold-max-cte "$GOAL_HOLD_MAX_CTE" \
  --goal-hold-max-separation "$GOAL_HOLD_MAX_SEPARATION" \
  --goal-hold-min-threat "$GOAL_HOLD_MIN_THREAT" \
  --goal-hold-target-linear "$GOAL_HOLD_TARGET_LINEAR" \
  --goal-hold-omega-blend "$GOAL_HOLD_OMEGA_BLEND" \
  --train-all-actor \
  --device cpu
