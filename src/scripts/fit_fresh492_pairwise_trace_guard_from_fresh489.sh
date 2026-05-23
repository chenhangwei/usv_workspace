#!/usr/bin/env bash
set -eo pipefail

# fresh492: offline pairwise trace guard from fresh489.
#
# fresh491's PPO pairwise guard saw rand_pair_active=0.000, so the local
# close-range brake never reached the actor.  This script fits the same guard
# directly on the raw collision windows, split by collision pair to avoid
# applying team-min-separation samples to uninvolved agents.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ -f ../install/setup.bash ]]; then
  source ../install/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
set -u
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh489_raw_cte_signed_return_from_fresh486.pt}"
STAGE1_OUTPUT="${STAGE1_OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh492_pairwise_trace_guard_stage1_from_fresh489.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh492_pairwise_trace_guard_from_fresh489.pt}"

STAGE1_TRACE_JSONS="${STAGE1_TRACE_JSONS:-/tmp/fresh490_1460_1461_r1/repeat1_seed1461/seed1461.json}"
STAGE2_TRACE_JSONS="${STAGE2_TRACE_JSONS:-/tmp/fresh491_1460_1461_r1/repeat1_seed1461/seed1461.json}"
ANCHOR_JSONS="${ANCHOR_JSONS:-/tmp/fresh489_1460_1461_r1/repeat1_seed1460/seed1460.json /tmp/fresh489_1461_short/repeat1_seed1461/seed1461.json /tmp/fresh490_1460_1461_r1/repeat1_seed1460/seed1460.json /tmp/fresh490_1460_1461_r1/repeat1_seed1461/seed1461.json /tmp/fresh491_1460_1461_r1/repeat1_seed1460/seed1460.json /tmp/fresh491_1460_1461_r1/repeat1_seed1461/seed1461.json}"

EPOCHS="${EPOCHS:-90}"
LEARNING_RATE="${LEARNING_RATE:-3.0e-6}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-420.0}"
ANCHOR_BATCH_SIZE="${ANCHOR_BATCH_SIZE:-768}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.035}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-18.0}"
RISK_GUARD_MAX_SEPARATION="${RISK_GUARD_MAX_SEPARATION:-2.60}"
RISK_GUARD_MAX_LINEAR="${RISK_GUARD_MAX_LINEAR:-0.085}"
RISK_GUARD_YIELD_MAX_LINEAR="${RISK_GUARD_YIELD_MAX_LINEAR:-0.015}"
RISK_GUARD_STANDON_MAX_LINEAR="${RISK_GUARD_STANDON_MAX_LINEAR:-0.085}"
RISK_GUARD_YIELD_MIN_OMEGA_ABS="${RISK_GUARD_YIELD_MIN_OMEGA_ABS:-0.34}"
RISK_GUARD_STANDON_MIN_OMEGA_ABS="${RISK_GUARD_STANDON_MIN_OMEGA_ABS:-0.10}"
RISK_GUARD_MIN_STARBOARD_OMEGA="${RISK_GUARD_MIN_STARBOARD_OMEGA:-0.34}"
RISK_GUARD_MIN_STARBOARD_THREAT="${RISK_GUARD_MIN_STARBOARD_THREAT:-0.08}"
RISK_GUARD_MIN_DISTANCE="${RISK_GUARD_MIN_DISTANCE:-1.20}"
RISK_GUARD_MIN_ROUTE_PROGRESS="${RISK_GUARD_MIN_ROUTE_PROGRESS:-0.04}"
RISK_GUARD_MAX_ROUTE_PROGRESS="${RISK_GUARD_MAX_ROUTE_PROGRESS:-0.98}"
RISK_GUARD_MIN_ABS_CTE="${RISK_GUARD_MIN_ABS_CTE:-0.50}"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi

append_trace_args() {
  local -n out_array="$1"
  shift
  local trace
  for trace in "$@"; do
    [[ -f "$trace" ]] || { echo "missing trace json: $trace" >&2; exit 1; }
    out_array+=(--trace-json "$trace")
  done
}

append_anchor_args() {
  local -n out_array="$1"
  shift
  local trace
  for trace in "$@"; do
    [[ -f "$trace" ]] || { echo "missing anchor json: $trace" >&2; exit 1; }
    out_array+=(--anchor-trace-json "$trace")
  done
}

stage1_traces=()
# shellcheck disable=SC2206
stage1_trace_list=($STAGE1_TRACE_JSONS)
append_trace_args stage1_traces "${stage1_trace_list[@]}"

stage2_traces=()
# shellcheck disable=SC2206
stage2_trace_list=($STAGE2_TRACE_JSONS)
append_trace_args stage2_traces "${stage2_trace_list[@]}"

anchor_args=()
# shellcheck disable=SC2206
anchor_list=($ANCHOR_JSONS)
append_anchor_args anchor_args "${anchor_list[@]}"

common_args=(
  --epochs "$EPOCHS"
  --batch-size 64
  --learning-rate "$LEARNING_RATE"
  --anchor-weight "$ANCHOR_WEIGHT"
  --anchor-batch-size "$ANCHOR_BATCH_SIZE"
  --max-grad-norm "$MAX_GRAD_NORM"
  --min-error-norm 999.0
  --target-priority deconf
  --target-shape-kinds guard
  --deconf-weight 0.0
  --offroute-weight 0.0
  --cte-weight 0.0
  --finish-weight 0.0
  --risk-guard-weight "$RISK_GUARD_WEIGHT"
  --risk-guard-min-threat 0.08
  --risk-guard-max-separation "$RISK_GUARD_MAX_SEPARATION"
  --risk-guard-max-linear "$RISK_GUARD_MAX_LINEAR"
  --risk-guard-yield-max-linear "$RISK_GUARD_YIELD_MAX_LINEAR"
  --risk-guard-standon-max-linear "$RISK_GUARD_STANDON_MAX_LINEAR"
  --risk-guard-omega-blend 1.0
  --risk-guard-yield-min-omega-abs "$RISK_GUARD_YIELD_MIN_OMEGA_ABS"
  --risk-guard-standon-min-omega-abs "$RISK_GUARD_STANDON_MIN_OMEGA_ABS"
  --risk-guard-min-starboard-omega "$RISK_GUARD_MIN_STARBOARD_OMEGA"
  --risk-guard-min-starboard-threat "$RISK_GUARD_MIN_STARBOARD_THREAT"
  --risk-guard-min-distance "$RISK_GUARD_MIN_DISTANCE"
  --risk-guard-min-route-progress "$RISK_GUARD_MIN_ROUTE_PROGRESS"
  --risk-guard-max-route-progress "$RISK_GUARD_MAX_ROUTE_PROGRESS"
  --risk-guard-min-abs-cte "$RISK_GUARD_MIN_ABS_CTE"
)

mkdir -p "$(dirname "$OUTPUT")"

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  "${stage1_traces[@]}" \
  "${anchor_args[@]}" \
  --output "$STAGE1_OUTPUT" \
  "${common_args[@]}" \
  --risk-guard-agent usv_02 \
  --risk-guard-agent usv_03

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$STAGE1_OUTPUT" \
  "${stage2_traces[@]}" \
  "${anchor_args[@]}" \
  --output "$OUTPUT" \
  "${common_args[@]}" \
  --risk-guard-agent usv_01 \
  --risk-guard-agent usv_02

echo "$OUTPUT"