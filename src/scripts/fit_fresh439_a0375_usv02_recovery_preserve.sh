#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ -f ../install/setup.bash ]]; then
  source ../install/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh437_a0375_usv02_stall_blend_from_fresh432.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh439_a0375_usv02_recovery_preserve.pt}"

TRACE_JSONS="${TRACE_JSONS:-/tmp/fresh437_a0375_hard4_combined_r3/repeat1_seed1461/seed1461.json /tmp/fresh437_a0375_hard4_combined_r3/repeat2_seed1461/seed1461.json /tmp/fresh437_a0375_hard4_combined_r3/repeat3_seed1461/seed1461.json}"
ANCHOR_JSONS="${ANCHOR_JSONS:-/tmp/fresh437_a0375_hard4_combined_r3/repeat1_seed1458/seed1458.json /tmp/fresh437_a0375_hard4_combined_r3/repeat1_seed1460/seed1460.json /tmp/fresh437_a0375_hard4_combined_r3/repeat1_seed1461/seed1461.json /tmp/fresh437_a0375_hard4_combined_r3/repeat1_seed1463/seed1463.json /tmp/fresh437_a0375_hard4_combined_r3/repeat2_seed1458/seed1458.json /tmp/fresh437_a0375_hard4_combined_r3/repeat2_seed1460/seed1460.json /tmp/fresh437_a0375_hard4_combined_r3/repeat2_seed1461/seed1461.json /tmp/fresh437_a0375_hard4_combined_r3/repeat2_seed1463/seed1463.json /tmp/fresh437_a0375_hard4_combined_r3/repeat3_seed1458/seed1458.json /tmp/fresh437_a0375_hard4_combined_r3/repeat3_seed1460/seed1460.json /tmp/fresh437_a0375_hard4_combined_r3/repeat3_seed1461/seed1461.json /tmp/fresh437_a0375_hard4_combined_r3/repeat3_seed1463/seed1463.json}"
PRESERVE_JSONS="${PRESERVE_JSONS:-/tmp/fresh432_seed1460_domain220_pair_r3/repeat1_seed1460/seed1460.json /tmp/fresh432_seed1460_domain220_pair_r3/repeat2_seed1460/seed1460.json /tmp/fresh432_seed1460_domain220_pair_r3/repeat3_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat1_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat2_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat3_seed1460/seed1460.json /tmp/fresh437_a0375_hard4_combined_r3/repeat1_seed1460/seed1460.json /tmp/fresh437_a0375_hard4_combined_r3/repeat2_seed1460/seed1460.json /tmp/fresh437_a0375_hard4_combined_r3/repeat3_seed1460/seed1460.json}"

EPOCHS="${EPOCHS:-85}"
LEARNING_RATE="${LEARNING_RATE:-5.0e-6}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-300.0}"
ANCHOR_BATCH_SIZE="${ANCHOR_BATCH_SIZE:-768}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.035}"
PRESERVE_ACTION_WEIGHT="${PRESERVE_ACTION_WEIGHT:-34.0}"
PRESERVE_MIN_TEAM_SEPARATION="${PRESERVE_MIN_TEAM_SEPARATION:-1.25}"
PRESERVE_MIN_EPISODE_PROGRESS="${PRESERVE_MIN_EPISODE_PROGRESS:-0.58}"
TARGET_LINEAR_BLEND="${TARGET_LINEAR_BLEND:-0.50}"
TARGET_OMEGA_BLEND="${TARGET_OMEGA_BLEND:-0.50}"
TARGET_SHAPE_KINDS="${TARGET_SHAPE_KINDS:-stall_start,late_clear,recovery}"

STALL_START_WEIGHT="${STALL_START_WEIGHT:-10.0}"
STALL_START_TARGET_LINEAR="${STALL_START_TARGET_LINEAR:-0.14}"
STALL_START_SOURCE_THRESHOLD="${STALL_START_SOURCE_THRESHOLD:-0.065}"
STALL_START_MAX_ROUTE_PROGRESS="${STALL_START_MAX_ROUTE_PROGRESS:-0.04}"
STALL_START_MAX_CTE="${STALL_START_MAX_CTE:-1.15}"
STALL_START_HEADING_SLOW_THRESHOLD="${STALL_START_HEADING_SLOW_THRESHOLD:-2.45}"
STALL_START_HEADING_RECOVERY_LINEAR="${STALL_START_HEADING_RECOVERY_LINEAR:-0.08}"

LATE_CLEAR_WEIGHT="${LATE_CLEAR_WEIGHT:-1.6}"
LATE_CLEAR_SOURCE_THRESHOLD="${LATE_CLEAR_SOURCE_THRESHOLD:-0.10}"
LATE_CLEAR_TARGET_LINEAR="${LATE_CLEAR_TARGET_LINEAR:-0.13}"
LATE_CLEAR_MIN_DISTANCE="${LATE_CLEAR_MIN_DISTANCE:-5.0}"
LATE_CLEAR_MAX_CTE="${LATE_CLEAR_MAX_CTE:-2.40}"
LATE_CLEAR_MIN_SEPARATION="${LATE_CLEAR_MIN_SEPARATION:-2.75}"
LATE_CLEAR_MAX_THREAT="${LATE_CLEAR_MAX_THREAT:-0.08}"
LATE_CLEAR_MIN_STEP="${LATE_CLEAR_MIN_STEP:-200}"
LATE_CLEAR_CTE_OMEGA_BLEND="${LATE_CLEAR_CTE_OMEGA_BLEND:-0.70}"
LATE_CLEAR_CTE_SLOW_THRESHOLD="${LATE_CLEAR_CTE_SLOW_THRESHOLD:-1.55}"
LATE_CLEAR_CTE_RECOVERY_LINEAR="${LATE_CLEAR_CTE_RECOVERY_LINEAR:-0.08}"

RECOVERY_SPEEDUP_WEIGHT="${RECOVERY_SPEEDUP_WEIGHT:-4.2}"
RECOVERY_SPEEDUP_SOURCE_THRESHOLD="${RECOVERY_SPEEDUP_SOURCE_THRESHOLD:-0.10}"
RECOVERY_SPEEDUP_TARGET_LINEAR="${RECOVERY_SPEEDUP_TARGET_LINEAR:-0.13}"
RECOVERY_SPEEDUP_MIN_DISTANCE="${RECOVERY_SPEEDUP_MIN_DISTANCE:-5.0}"
RECOVERY_SPEEDUP_MAX_CTE="${RECOVERY_SPEEDUP_MAX_CTE:-2.80}"
RECOVERY_SPEEDUP_MIN_SEPARATION="${RECOVERY_SPEEDUP_MIN_SEPARATION:-2.55}"
RECOVERY_SPEEDUP_MAX_THREAT="${RECOVERY_SPEEDUP_MAX_THREAT:-0.12}"
RECOVERY_SPEEDUP_MIN_STEP="${RECOVERY_SPEEDUP_MIN_STEP:-180}"
RECOVERY_SPEEDUP_MAX_ROUTE_PROGRESS="${RECOVERY_SPEEDUP_MAX_ROUTE_PROGRESS:-0.12}"
RECOVERY_SPEEDUP_MAX_HEADING_ERROR="${RECOVERY_SPEEDUP_MAX_HEADING_ERROR:-2.80}"
RECOVERY_SPEEDUP_CTE_OMEGA_BLEND="${RECOVERY_SPEEDUP_CTE_OMEGA_BLEND:-0.70}"

EDGE_COAST_WEIGHT="${EDGE_COAST_WEIGHT:-0.0}"
EDGE_COAST_SOURCE_THRESHOLD="${EDGE_COAST_SOURCE_THRESHOLD:-0.12}"
EDGE_COAST_TARGET_LINEAR="${EDGE_COAST_TARGET_LINEAR:-0.08}"
EDGE_COAST_MIN_DISTANCE="${EDGE_COAST_MIN_DISTANCE:-5.0}"
EDGE_COAST_MIN_ABS_CTE="${EDGE_COAST_MIN_ABS_CTE:-2.60}"
EDGE_COAST_MAX_ABS_CTE="${EDGE_COAST_MAX_ABS_CTE:-3.05}"
EDGE_COAST_MIN_SEPARATION="${EDGE_COAST_MIN_SEPARATION:-2.55}"
EDGE_COAST_MAX_THREAT="${EDGE_COAST_MAX_THREAT:-0.12}"
EDGE_COAST_MIN_STEP="${EDGE_COAST_MIN_STEP:-160}"
EDGE_COAST_MIN_ROUTE_PROGRESS="${EDGE_COAST_MIN_ROUTE_PROGRESS:-0.0}"
EDGE_COAST_MAX_ROUTE_PROGRESS="${EDGE_COAST_MAX_ROUTE_PROGRESS:-0.18}"
EDGE_COAST_MAX_HEADING_ERROR="${EDGE_COAST_MAX_HEADING_ERROR:-1.10}"
EDGE_COAST_OMEGA_ZERO_BLEND="${EDGE_COAST_OMEGA_ZERO_BLEND:-0.80}"
EDGE_COAST_MAX_OMEGA_ABS="${EDGE_COAST_MAX_OMEGA_ABS:-0.035}"

RECOVERY_TURN_WEIGHT="${RECOVERY_TURN_WEIGHT:-0.0}"
RECOVERY_TURN_TARGET_LINEAR="${RECOVERY_TURN_TARGET_LINEAR:-0.05}"
RECOVERY_TURN_MIN_DISTANCE="${RECOVERY_TURN_MIN_DISTANCE:-5.0}"
RECOVERY_TURN_MIN_ABS_CTE="${RECOVERY_TURN_MIN_ABS_CTE:-1.20}"
RECOVERY_TURN_MIN_HEADING_ERROR="${RECOVERY_TURN_MIN_HEADING_ERROR:-1.00}"
RECOVERY_TURN_MAX_HEADING_ERROR="${RECOVERY_TURN_MAX_HEADING_ERROR:-3.20}"
RECOVERY_TURN_HEADING_SIGN="${RECOVERY_TURN_HEADING_SIGN:-any}"
RECOVERY_TURN_MIN_SEPARATION="${RECOVERY_TURN_MIN_SEPARATION:-2.55}"
RECOVERY_TURN_MAX_THREAT="${RECOVERY_TURN_MAX_THREAT:-0.12}"
RECOVERY_TURN_MIN_STEP="${RECOVERY_TURN_MIN_STEP:-160}"
RECOVERY_TURN_MAX_STEP="${RECOVERY_TURN_MAX_STEP:--1}"
RECOVERY_TURN_MAX_ROUTE_PROGRESS="${RECOVERY_TURN_MAX_ROUTE_PROGRESS:-0.18}"
RECOVERY_TURN_OMEGA_BLEND="${RECOVERY_TURN_OMEGA_BLEND:-0.85}"
RECOVERY_TURN_MIN_OMEGA_ABS="${RECOVERY_TURN_MIN_OMEGA_ABS:-0.08}"
RECOVERY_TURN_HEADING_GAIN="${RECOVERY_TURN_HEADING_GAIN:-0.35}"
RECOVERY_TURN_HEADING_OMEGA_SIGN="${RECOVERY_TURN_HEADING_OMEGA_SIGN:-1.0}"
RECOVERY_TURN_MAX_OMEGA="${RECOVERY_TURN_MAX_OMEGA:-0.42}"
RECOVERY_TURN_TARGET_OMEGA_CAP="${RECOVERY_TURN_TARGET_OMEGA_CAP:--1.0}"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi

trace_args=()
for trace in $TRACE_JSONS; do
  [[ -f "$trace" ]] || { echo "missing trace json: $trace" >&2; exit 1; }
  trace_args+=(--trace-json "$trace")
done

anchor_args=()
for trace in $ANCHOR_JSONS; do
  [[ -f "$trace" ]] || { echo "missing anchor json: $trace" >&2; exit 1; }
  anchor_args+=(--anchor-trace-json "$trace")
done

preserve_args=()
for trace in $PRESERVE_JSONS; do
  [[ -f "$trace" ]] || { echo "missing preserve json: $trace" >&2; exit 1; }
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
  --min-error-norm 999.0 \
  --target-priority deconf \
  --target-shape-kinds "$TARGET_SHAPE_KINDS" \
  --target-linear-blend "$TARGET_LINEAR_BLEND" \
  --target-omega-blend "$TARGET_OMEGA_BLEND" \
  --deconf-weight 0.0 \
  --offroute-weight 0.0 \
  --cte-weight 0.0 \
  --finish-weight 0.0 \
  --preserve-action-weight "$PRESERVE_ACTION_WEIGHT" \
  --preserve-min-route-progress 0.0 \
  --preserve-max-route-progress 1.10 \
  --preserve-min-team-separation "$PRESERVE_MIN_TEAM_SEPARATION" \
  --preserve-min-episode-progress "$PRESERVE_MIN_EPISODE_PROGRESS" \
  --preserve-exclude-collisions \
  --stall-start-weight "$STALL_START_WEIGHT" \
  --stall-start-agent usv_02 \
  --stall-start-source-threshold "$STALL_START_SOURCE_THRESHOLD" \
  --stall-start-target-linear "$STALL_START_TARGET_LINEAR" \
  --stall-start-min-distance 5.0 \
  --stall-start-max-route-progress "$STALL_START_MAX_ROUTE_PROGRESS" \
  --stall-start-max-cte "$STALL_START_MAX_CTE" \
  --stall-start-min-separation 2.75 \
  --stall-start-max-threat 0.08 \
  --stall-start-min-step 40 \
  --stall-start-max-step 820 \
  --stall-start-cte-omega-blend 0.50 \
  --stall-start-heading-slow-threshold "$STALL_START_HEADING_SLOW_THRESHOLD" \
  --stall-start-heading-recovery-linear "$STALL_START_HEADING_RECOVERY_LINEAR" \
  --late-clear-weight "$LATE_CLEAR_WEIGHT" \
  --late-clear-agent usv_02 \
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
  --recovery-turn-weight "$RECOVERY_TURN_WEIGHT" \
  --recovery-turn-agent usv_02 \
  --recovery-turn-target-linear "$RECOVERY_TURN_TARGET_LINEAR" \
  --recovery-turn-min-distance "$RECOVERY_TURN_MIN_DISTANCE" \
  --recovery-turn-min-abs-cte "$RECOVERY_TURN_MIN_ABS_CTE" \
  --recovery-turn-min-heading-error "$RECOVERY_TURN_MIN_HEADING_ERROR" \
  --recovery-turn-max-heading-error "$RECOVERY_TURN_MAX_HEADING_ERROR" \
  --recovery-turn-heading-sign "$RECOVERY_TURN_HEADING_SIGN" \
  --recovery-turn-min-separation "$RECOVERY_TURN_MIN_SEPARATION" \
  --recovery-turn-max-threat "$RECOVERY_TURN_MAX_THREAT" \
  --recovery-turn-min-step "$RECOVERY_TURN_MIN_STEP" \
  --recovery-turn-max-step "$RECOVERY_TURN_MAX_STEP" \
  --recovery-turn-min-route-progress 0.0 \
  --recovery-turn-max-route-progress "$RECOVERY_TURN_MAX_ROUTE_PROGRESS" \
  --recovery-turn-omega-blend "$RECOVERY_TURN_OMEGA_BLEND" \
  --recovery-turn-min-omega-abs "$RECOVERY_TURN_MIN_OMEGA_ABS" \
  --recovery-turn-heading-gain "$RECOVERY_TURN_HEADING_GAIN" \
  --recovery-turn-heading-omega-sign "$RECOVERY_TURN_HEADING_OMEGA_SIGN" \
  --recovery-turn-max-omega "$RECOVERY_TURN_MAX_OMEGA" \
  --recovery-turn-target-omega-cap "$RECOVERY_TURN_TARGET_OMEGA_CAP" \
  --recovery-speedup-weight "$RECOVERY_SPEEDUP_WEIGHT" \
  --recovery-speedup-agent usv_02 \
  --recovery-speedup-source-threshold "$RECOVERY_SPEEDUP_SOURCE_THRESHOLD" \
  --recovery-speedup-target-linear "$RECOVERY_SPEEDUP_TARGET_LINEAR" \
  --recovery-speedup-min-distance "$RECOVERY_SPEEDUP_MIN_DISTANCE" \
  --recovery-speedup-max-cte "$RECOVERY_SPEEDUP_MAX_CTE" \
  --recovery-speedup-min-separation "$RECOVERY_SPEEDUP_MIN_SEPARATION" \
  --recovery-speedup-max-threat "$RECOVERY_SPEEDUP_MAX_THREAT" \
  --recovery-speedup-min-step "$RECOVERY_SPEEDUP_MIN_STEP" \
  --recovery-speedup-min-route-progress 0.0 \
  --recovery-speedup-max-route-progress "$RECOVERY_SPEEDUP_MAX_ROUTE_PROGRESS" \
  --recovery-speedup-max-heading-error "$RECOVERY_SPEEDUP_MAX_HEADING_ERROR" \
  --recovery-speedup-cte-omega-blend "$RECOVERY_SPEEDUP_CTE_OMEGA_BLEND" \
  --edge-coast-weight "$EDGE_COAST_WEIGHT" \
  --edge-coast-agent usv_02 \
  --edge-coast-source-threshold "$EDGE_COAST_SOURCE_THRESHOLD" \
  --edge-coast-target-linear "$EDGE_COAST_TARGET_LINEAR" \
  --edge-coast-min-distance "$EDGE_COAST_MIN_DISTANCE" \
  --edge-coast-min-abs-cte "$EDGE_COAST_MIN_ABS_CTE" \
  --edge-coast-max-abs-cte "$EDGE_COAST_MAX_ABS_CTE" \
  --edge-coast-min-separation "$EDGE_COAST_MIN_SEPARATION" \
  --edge-coast-max-threat "$EDGE_COAST_MAX_THREAT" \
  --edge-coast-min-step "$EDGE_COAST_MIN_STEP" \
  --edge-coast-min-route-progress "$EDGE_COAST_MIN_ROUTE_PROGRESS" \
  --edge-coast-max-route-progress "$EDGE_COAST_MAX_ROUTE_PROGRESS" \
  --edge-coast-max-heading-error "$EDGE_COAST_MAX_HEADING_ERROR" \
  --edge-coast-omega-zero-blend "$EDGE_COAST_OMEGA_ZERO_BLEND" \
  --edge-coast-max-omega-abs "$EDGE_COAST_MAX_OMEGA_ABS" \
  --goal-return-weight 0.0

echo "$OUTPUT"