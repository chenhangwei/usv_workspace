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

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh426_usv03_clear_short_probe.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh431_agent23_late_clear_tracefit_from_fresh426.pt}"
EPOCHS="${EPOCHS:-70}"
LEARNING_RATE="${LEARNING_RATE:-2.0e-6}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-650.0}"
ANCHOR_BATCH_SIZE="${ANCHOR_BATCH_SIZE:-512}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.050}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-999.0}"
TARGET_LINEAR_BLEND="${TARGET_LINEAR_BLEND:-0.75}"
TARGET_OMEGA_BLEND="${TARGET_OMEGA_BLEND:-0.85}"

TRACE_JSONS="${TRACE_JSONS:-/tmp/fresh426_hard4_r3/repeat1_seed1458/seed1458.json /tmp/fresh426_hard4_r3/repeat1_seed1461/seed1461.json}"
ANCHOR_JSONS="${ANCHOR_JSONS:-/tmp/fresh426_hard4_r3/repeat1_seed1458/seed1458.json /tmp/fresh426_hard4_r3/repeat1_seed1460/seed1460.json /tmp/fresh426_hard4_r3/repeat1_seed1461/seed1461.json /tmp/fresh426_hard4_r3/repeat1_seed1463/seed1463.json}"
PRESERVE_JSONS="${PRESERVE_JSONS:-/tmp/fresh426_hard4_r3/repeat1_seed1460/seed1460.json /tmp/fresh426_hard4_r3/repeat1_seed1463/seed1463.json}"

PRESERVE_ACTION_WEIGHT="${PRESERVE_ACTION_WEIGHT:-6.0}"
PRESERVE_MIN_TEAM_SEPARATION="${PRESERVE_MIN_TEAM_SEPARATION:-1.20}"

LATE_CLEAR_WEIGHT="${LATE_CLEAR_WEIGHT:-3.0}"
LATE_CLEAR_SOURCE_THRESHOLD="${LATE_CLEAR_SOURCE_THRESHOLD:-0.13}"
LATE_CLEAR_TARGET_LINEAR="${LATE_CLEAR_TARGET_LINEAR:-0.16}"
LATE_CLEAR_MIN_DISTANCE="${LATE_CLEAR_MIN_DISTANCE:-5.0}"
LATE_CLEAR_MAX_CTE="${LATE_CLEAR_MAX_CTE:-2.40}"
LATE_CLEAR_MIN_SEPARATION="${LATE_CLEAR_MIN_SEPARATION:-2.75}"
LATE_CLEAR_MAX_THREAT="${LATE_CLEAR_MAX_THREAT:-0.08}"
LATE_CLEAR_MIN_STEP="${LATE_CLEAR_MIN_STEP:-200}"
LATE_CLEAR_CTE_OMEGA_BLEND="${LATE_CLEAR_CTE_OMEGA_BLEND:-0.70}"
LATE_CLEAR_CTE_SLOW_THRESHOLD="${LATE_CLEAR_CTE_SLOW_THRESHOLD:-1.45}"
LATE_CLEAR_CTE_RECOVERY_LINEAR="${LATE_CLEAR_CTE_RECOVERY_LINEAR:-0.11}"

RECOVERY_SPEEDUP_WEIGHT="${RECOVERY_SPEEDUP_WEIGHT:-1.1}"
RECOVERY_SPEEDUP_SOURCE_THRESHOLD="${RECOVERY_SPEEDUP_SOURCE_THRESHOLD:-0.12}"
RECOVERY_SPEEDUP_TARGET_LINEAR="${RECOVERY_SPEEDUP_TARGET_LINEAR:-0.15}"
RECOVERY_SPEEDUP_MIN_DISTANCE="${RECOVERY_SPEEDUP_MIN_DISTANCE:-5.0}"
RECOVERY_SPEEDUP_MAX_CTE="${RECOVERY_SPEEDUP_MAX_CTE:-2.50}"
RECOVERY_SPEEDUP_MIN_SEPARATION="${RECOVERY_SPEEDUP_MIN_SEPARATION:-2.75}"
RECOVERY_SPEEDUP_MAX_THREAT="${RECOVERY_SPEEDUP_MAX_THREAT:-0.10}"
RECOVERY_SPEEDUP_MIN_STEP="${RECOVERY_SPEEDUP_MIN_STEP:-200}"
RECOVERY_SPEEDUP_MAX_ROUTE_PROGRESS="${RECOVERY_SPEEDUP_MAX_ROUTE_PROGRESS:-0.58}"
RECOVERY_SPEEDUP_MAX_HEADING_ERROR="${RECOVERY_SPEEDUP_MAX_HEADING_ERROR:-2.50}"
RECOVERY_SPEEDUP_CTE_OMEGA_BLEND="${RECOVERY_SPEEDUP_CTE_OMEGA_BLEND:-0.70}"

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
  --min-error-norm "$MIN_ERROR_NORM" \
  --target-priority deconf \
  --target-shape-kinds late_clear,recovery \
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
  --late-clear-weight "$LATE_CLEAR_WEIGHT" \
  --late-clear-agent usv_02 \
  --late-clear-agent usv_03 \
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
  --recovery-speedup-weight "$RECOVERY_SPEEDUP_WEIGHT" \
  --recovery-speedup-agent usv_02 \
  --recovery-speedup-agent usv_03 \
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
  --recovery-speedup-cte-omega-blend "$RECOVERY_SPEEDUP_CTE_OMEGA_BLEND"

echo "$OUTPUT"