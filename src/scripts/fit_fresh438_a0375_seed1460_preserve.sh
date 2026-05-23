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
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh438_a0375_seed1460_preserve_usv02_stall.pt}"

TRACE_JSONS="${TRACE_JSONS:-/tmp/fresh437_a0375_hard4_combined_r3/repeat1_seed1461/seed1461.json /tmp/fresh437_a0375_hard4_combined_r3/repeat2_seed1461/seed1461.json /tmp/fresh437_a0375_hard4_combined_r3/repeat3_seed1461/seed1461.json}"
ANCHOR_JSONS="${ANCHOR_JSONS:-/tmp/fresh437_a0375_hard4_combined_r3/repeat1_seed1458/seed1458.json /tmp/fresh437_a0375_hard4_combined_r3/repeat1_seed1460/seed1460.json /tmp/fresh437_a0375_hard4_combined_r3/repeat1_seed1461/seed1461.json /tmp/fresh437_a0375_hard4_combined_r3/repeat1_seed1463/seed1463.json /tmp/fresh437_a0375_hard4_combined_r3/repeat2_seed1458/seed1458.json /tmp/fresh437_a0375_hard4_combined_r3/repeat2_seed1460/seed1460.json /tmp/fresh437_a0375_hard4_combined_r3/repeat2_seed1461/seed1461.json /tmp/fresh437_a0375_hard4_combined_r3/repeat2_seed1463/seed1463.json /tmp/fresh437_a0375_hard4_combined_r3/repeat3_seed1458/seed1458.json /tmp/fresh437_a0375_hard4_combined_r3/repeat3_seed1460/seed1460.json /tmp/fresh437_a0375_hard4_combined_r3/repeat3_seed1461/seed1461.json /tmp/fresh437_a0375_hard4_combined_r3/repeat3_seed1463/seed1463.json}"
PRESERVE_JSONS="${PRESERVE_JSONS:-/tmp/fresh432_seed1460_domain220_pair_r3/repeat1_seed1460/seed1460.json /tmp/fresh432_seed1460_domain220_pair_r3/repeat2_seed1460/seed1460.json /tmp/fresh432_seed1460_domain220_pair_r3/repeat3_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat1_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat2_seed1460/seed1460.json /tmp/fresh432_hard4_r3/repeat3_seed1460/seed1460.json}"

EPOCHS="${EPOCHS:-80}"
LEARNING_RATE="${LEARNING_RATE:-8.0e-6}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-220.0}"
ANCHOR_BATCH_SIZE="${ANCHOR_BATCH_SIZE:-768}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.040}"
PRESERVE_ACTION_WEIGHT="${PRESERVE_ACTION_WEIGHT:-18.0}"
PRESERVE_MIN_TEAM_SEPARATION="${PRESERVE_MIN_TEAM_SEPARATION:-1.25}"
PRESERVE_MIN_EPISODE_PROGRESS="${PRESERVE_MIN_EPISODE_PROGRESS:-0.58}"
TARGET_LINEAR_BLEND="${TARGET_LINEAR_BLEND:-0.60}"
TARGET_OMEGA_BLEND="${TARGET_OMEGA_BLEND:-0.45}"

STALL_START_WEIGHT="${STALL_START_WEIGHT:-22.0}"
STALL_START_TARGET_LINEAR="${STALL_START_TARGET_LINEAR:-0.17}"
STALL_START_SOURCE_THRESHOLD="${STALL_START_SOURCE_THRESHOLD:-0.065}"
STALL_START_MAX_ROUTE_PROGRESS="${STALL_START_MAX_ROUTE_PROGRESS:-0.035}"
STALL_START_MAX_CTE="${STALL_START_MAX_CTE:-1.05}"
STALL_START_HEADING_SLOW_THRESHOLD="${STALL_START_HEADING_SLOW_THRESHOLD:-2.45}"
STALL_START_HEADING_RECOVERY_LINEAR="${STALL_START_HEADING_RECOVERY_LINEAR:-0.09}"

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
  --target-shape-kinds stall_start \
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
  --goal-return-weight 0.0

echo "$OUTPUT"