#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source ../install/setup.bash
set -u
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh331_nearmiss_event_guard_from_fresh329.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh337_yield_speed_guard_from_fresh331.pt}"
EPOCHS="${EPOCHS:-42}"
LEARNING_RATE="${LEARNING_RATE:-7.5e-8}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-1400.0}"
PRESERVE_ACTION_WEIGHT="${PRESERVE_ACTION_WEIGHT:-0.0}"
PRESERVE_MIN_ROUTE_PROGRESS="${PRESERVE_MIN_ROUTE_PROGRESS:-0.70}"
PRESERVE_MAX_ROUTE_PROGRESS="${PRESERVE_MAX_ROUTE_PROGRESS:-1.10}"
PRESERVE_MIN_TEAM_SEPARATION="${PRESERVE_MIN_TEAM_SEPARATION:-0.0}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-0.38}"
RISK_GUARD_MIN_THREAT="${RISK_GUARD_MIN_THREAT:-0.75}"
RISK_GUARD_MAX_SEPARATION="${RISK_GUARD_MAX_SEPARATION:-1.60}"
RISK_GUARD_MAX_LINEAR="${RISK_GUARD_MAX_LINEAR:-0.105}"
RISK_GUARD_OMEGA_BLEND="${RISK_GUARD_OMEGA_BLEND:-0.0}"
RISK_GUARD_MIN_STARBOARD_OMEGA="${RISK_GUARD_MIN_STARBOARD_OMEGA:-0.0}"
RISK_GUARD_MIN_STARBOARD_THREAT="${RISK_GUARD_MIN_STARBOARD_THREAT:-0.0}"
RISK_GUARD_MIN_DISTANCE="${RISK_GUARD_MIN_DISTANCE:-0.40}"
RISK_GUARD_AGENT="${RISK_GUARD_AGENT:-}"
RISK_GUARD_MIN_ROUTE_PROGRESS="${RISK_GUARD_MIN_ROUTE_PROGRESS:-0.0}"
RISK_GUARD_MAX_ROUTE_PROGRESS="${RISK_GUARD_MAX_ROUTE_PROGRESS:-1.10}"
RISK_GUARD_MIN_ABS_CTE="${RISK_GUARD_MIN_ABS_CTE:-0.0}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-999.0}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.018}"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi

trace_args=()
for trace in \
  /tmp/fresh331_seed1456_eventwin_r6_v1/repeat1_seed1456/seed1456.json \
  /tmp/fresh331_seed1456_eventwin_r6_v1/repeat2_seed1456/seed1456.json \
  /tmp/fresh331_seed1456_eventwin_r6_v1/repeat3_seed1456/seed1456.json \
  /tmp/fresh331_seed1456_eventwin_r6_v1/repeat4_seed1456/seed1456.json \
  /tmp/fresh331_seed1456_eventwin_r6_v1/repeat5_seed1456/seed1456.json \
  /tmp/fresh331_seed1456_eventwin_r6_v1/repeat6_seed1456/seed1456.json \
  /tmp/fresh331_seed1456_domain220_repeats_v1/repeat5_seed1456/seed1456.json \
  /tmp/fresh331_seed1456_domain220_repeats_v1/repeat6_seed1456/seed1456.json; do
  if [[ ! -f "$trace" ]]; then
    echo "missing fresh337 target trace: $trace" >&2
    exit 1
  fi
  trace_args+=(--trace-json "$trace")
done

anchor_args=()
for trace in \
  /tmp/fresh331_seed1456_domain220_repeats_v1/repeat1_seed1456/seed1456.json \
  /tmp/fresh331_seed1456_domain220_repeats_v1/repeat2_seed1456/seed1456.json \
  /tmp/fresh331_seed1456_domain220_repeats_v1/repeat3_seed1456/seed1456.json \
  /tmp/fresh331_seed1456_domain220_repeats_v1/repeat4_seed1456/seed1456.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1456/seed1456.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1457/seed1457.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1458/seed1458.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1459/seed1459.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1460/seed1460.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1461/seed1461.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1462/seed1462.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1463/seed1463.json \
  /tmp/fresh331_wide_hard9_v1/repeat1_seed1464/seed1464.json \
  /tmp/fresh331_fixed_gate_v1/repeat1_seed1460/seed1460.json \
  /tmp/fresh331_fixed_gate_v1/repeat2_seed1460/seed1460.json \
  /tmp/fresh331_fixed_gate_v1/repeat3_seed1460/seed1460.json \
  /tmp/fresh331_fixed_gate_v1/repeat1_seed1461/seed1461.json \
  /tmp/fresh331_fixed_gate_v1/repeat2_seed1461/seed1461.json \
  /tmp/fresh331_fixed_gate_v1/repeat3_seed1461/seed1461.json \
  /tmp/fresh331_fixed_gate_v1/repeat4_seed1461/seed1461.json \
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

preserve_args=()
if python3 - <<PY
import sys
sys.exit(0 if float("$PRESERVE_ACTION_WEIGHT") > 0.0 else 1)
PY
then
  for trace in \
    /tmp/fresh331_fixed_eventwin_r4_v1/repeat1_seed1460/seed1460.json \
    /tmp/fresh331_fixed_eventwin_r4_v1/repeat1_seed1461/seed1461.json \
    /tmp/fresh331_fixed_eventwin_r4_v1/repeat2_seed1460/seed1460.json \
    /tmp/fresh331_fixed_eventwin_r4_v1/repeat2_seed1461/seed1461.json \
    /tmp/fresh331_fixed_eventwin_r4_v1/repeat3_seed1460/seed1460.json \
    /tmp/fresh331_fixed_eventwin_r4_v1/repeat3_seed1461/seed1461.json \
    /tmp/fresh331_fixed_eventwin_r4_v1/repeat4_seed1460/seed1460.json \
    /tmp/fresh331_fixed_eventwin_r4_v1/repeat4_seed1461/seed1461.json; do
    if [[ -f "$trace" ]]; then
      preserve_args+=(--preserve-trace-json "$trace")
    fi
  done
fi

risk_agent_args=()
if [[ -n "$RISK_GUARD_AGENT" ]]; then
  IFS=',' read -r -a risk_agents <<< "$RISK_GUARD_AGENT"
  for agent in "${risk_agents[@]}"; do
    if [[ -n "$agent" ]]; then
      risk_agent_args+=(--risk-guard-agent "$agent")
    fi
  done
fi

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
  --anchor-batch-size 768 \
  --preserve-action-weight "$PRESERVE_ACTION_WEIGHT" \
  --preserve-min-route-progress "$PRESERVE_MIN_ROUTE_PROGRESS" \
  --preserve-max-route-progress "$PRESERVE_MAX_ROUTE_PROGRESS" \
  --preserve-min-team-separation "$PRESERVE_MIN_TEAM_SEPARATION" \
  --preserve-exclude-collisions \
  --max-grad-norm "$MAX_GRAD_NORM" \
  --min-error-norm "$MIN_ERROR_NORM" \
  --target-priority deconf \
  --target-shape-kinds guard \
  --deconf-weight 0.0 \
  --offroute-weight 0.0 \
  --cte-weight 0.0 \
  --finish-weight 0.0 \
  --risk-guard-weight "$RISK_GUARD_WEIGHT" \
  --risk-guard-yield-only \
  --risk-guard-require-threat \
  --risk-guard-min-threat "$RISK_GUARD_MIN_THREAT" \
  --risk-guard-max-separation "$RISK_GUARD_MAX_SEPARATION" \
  --risk-guard-max-linear "$RISK_GUARD_MAX_LINEAR" \
  --risk-guard-omega-blend "$RISK_GUARD_OMEGA_BLEND" \
  --risk-guard-min-starboard-omega "$RISK_GUARD_MIN_STARBOARD_OMEGA" \
  --risk-guard-min-starboard-threat "$RISK_GUARD_MIN_STARBOARD_THREAT" \
  --risk-guard-min-distance "$RISK_GUARD_MIN_DISTANCE" \
  "${risk_agent_args[@]}" \
  --risk-guard-min-route-progress "$RISK_GUARD_MIN_ROUTE_PROGRESS" \
  --risk-guard-max-route-progress "$RISK_GUARD_MAX_ROUTE_PROGRESS" \
  --risk-guard-min-abs-cte "$RISK_GUARD_MIN_ABS_CTE"

echo "$OUTPUT"