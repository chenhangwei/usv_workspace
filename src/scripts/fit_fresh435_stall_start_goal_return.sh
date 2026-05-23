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

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh432_agent23_late_clear_tracefit_stronger_from_fresh426.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh435_stall_start_goal_return_from_fresh432.pt}"

TRACE_JSONS="${TRACE_JSONS:-/tmp/fresh432_hard4_r3/repeat1_seed1461/seed1461.json /tmp/fresh432_hard4_r3/repeat2_seed1461/seed1461.json /tmp/fresh432_hard4_r3/repeat3_seed1461/seed1461.json}"
ANCHOR_JSONS="${ANCHOR_JSONS:-/tmp/fresh432_hard4_r3/repeat1_seed1458/seed1458.json /tmp/fresh432_hard4_r3/repeat1_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat1_seed1461/seed1461.json /tmp/fresh432_hard4_r3/repeat1_seed1463/seed1463.json /tmp/fresh432_hard4_r3/repeat2_seed1458/seed1458.json /tmp/fresh432_hard4_r3/repeat2_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat2_seed1461/seed1461.json /tmp/fresh432_hard4_r3/repeat2_seed1463/seed1463.json /tmp/fresh432_hard4_r3/repeat3_seed1458/seed1458.json /tmp/fresh432_hard4_r3/repeat3_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat3_seed1461/seed1461.json /tmp/fresh432_hard4_r3/repeat3_seed1463/seed1463.json}"
PRESERVE_JSONS="${PRESERVE_JSONS:-/tmp/fresh432_hard4_r3/repeat1_seed1458/seed1458.json /tmp/fresh432_hard4_r3/repeat1_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat1_seed1463/seed1463.json /tmp/fresh432_hard4_r3/repeat2_seed1458/seed1458.json /tmp/fresh432_hard4_r3/repeat2_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat2_seed1463/seed1463.json /tmp/fresh432_hard4_r3/repeat3_seed1458/seed1458.json /tmp/fresh432_hard4_r3/repeat3_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat3_seed1463/seed1463.json}"

EPOCHS="${EPOCHS:-90}"
LEARNING_RATE="${LEARNING_RATE:-1.2e-5}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-120.0}"
ANCHOR_BATCH_SIZE="${ANCHOR_BATCH_SIZE:-768}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.050}"
PRESERVE_ACTION_WEIGHT="${PRESERVE_ACTION_WEIGHT:-10.0}"
PRESERVE_MIN_TEAM_SEPARATION="${PRESERVE_MIN_TEAM_SEPARATION:-1.20}"
TARGET_LINEAR_BLEND="${TARGET_LINEAR_BLEND:-0.90}"
TARGET_OMEGA_BLEND="${TARGET_OMEGA_BLEND:-0.75}"

STALL_START_WEIGHT="${STALL_START_WEIGHT:-12.0}"
STALL_START_TARGET_LINEAR="${STALL_START_TARGET_LINEAR:-0.15}"
STALL_START_SOURCE_THRESHOLD="${STALL_START_SOURCE_THRESHOLD:-0.045}"
STALL_START_MAX_ROUTE_PROGRESS="${STALL_START_MAX_ROUTE_PROGRESS:-0.020}"
STALL_START_MAX_CTE="${STALL_START_MAX_CTE:-0.85}"
STALL_START_HEADING_SLOW_THRESHOLD="${STALL_START_HEADING_SLOW_THRESHOLD:-2.35}"
STALL_START_HEADING_RECOVERY_LINEAR="${STALL_START_HEADING_RECOVERY_LINEAR:-0.08}"

GOAL_RETURN_WEIGHT="${GOAL_RETURN_WEIGHT:-7.0}"
GOAL_RETURN_TARGET_LINEAR="${GOAL_RETURN_TARGET_LINEAR:-0.11}"
GOAL_RETURN_SOURCE_THRESHOLD="${GOAL_RETURN_SOURCE_THRESHOLD:-0.12}"
GOAL_RETURN_MIN_DISTANCE="${GOAL_RETURN_MIN_DISTANCE:-2.50}"
GOAL_RETURN_MIN_ROUTE_PROGRESS="${GOAL_RETURN_MIN_ROUTE_PROGRESS:-0.92}"
GOAL_RETURN_MAX_CTE="${GOAL_RETURN_MAX_CTE:-3.05}"
GOAL_RETURN_SAFE_OMEGA_BLEND="${GOAL_RETURN_SAFE_OMEGA_BLEND:-0.80}"

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
  --target-shape-kinds stall_start,goal_return \
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
  --stall-start-cte-omega-blend 0.55 \
  --stall-start-heading-slow-threshold "$STALL_START_HEADING_SLOW_THRESHOLD" \
  --stall-start-heading-recovery-linear "$STALL_START_HEADING_RECOVERY_LINEAR" \
  --goal-return-weight "$GOAL_RETURN_WEIGHT" \
  --goal-return-agent usv_03 \
  --goal-return-source-threshold "$GOAL_RETURN_SOURCE_THRESHOLD" \
  --goal-return-target-linear "$GOAL_RETURN_TARGET_LINEAR" \
  --goal-return-min-distance "$GOAL_RETURN_MIN_DISTANCE" \
  --goal-return-min-route-progress "$GOAL_RETURN_MIN_ROUTE_PROGRESS" \
  --goal-return-max-cte "$GOAL_RETURN_MAX_CTE" \
  --goal-return-min-separation 2.75 \
  --goal-return-max-threat 0.10 \
  --goal-return-min-step 320 \
  --goal-return-safe-omega-blend "$GOAL_RETURN_SAFE_OMEGA_BLEND"

echo "$OUTPUT"