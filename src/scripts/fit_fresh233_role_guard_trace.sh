#!/bin/bash
# fresh233: guarded role-balance trace fit from fresh228.
#
# Fit the known 1458 role imbalance while turning the other hard seeds into
# guard-only traces. This keeps their risky windows represented without asking
# the actor to chase every traced deconf target, which previously moved
# collisions between hard seeds.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh228_full_seed_yield_omega_from_fresh211.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh233_1458_role_guard_from_fresh228.pt}"
TRACE_DIR="${TRACE_DIR:-/tmp/fresh231_trace_fit}"
WORK_DIR="${WORK_DIR:-/tmp/fresh233_role_guard_trace}"
TARGET_SEED="${TARGET_SEED:-1458}"
ANCHOR_SEEDS="${ANCHOR_SEEDS:-1458 1459 1460 1461 1462 1463 1464}"
GUARD_SEEDS="${GUARD_SEEDS:-1459 1460 1461 1462 1463 1464}"
EPOCHS="${EPOCHS:-170}"
LEARNING_RATE="${LEARNING_RATE:-3.5e-6}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-14.0}"
DECONF_WEIGHT="${DECONF_WEIGHT:-2.15}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-2.40}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.035}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.20}"

mkdir -p "$(dirname "$OUTPUT")" "$WORK_DIR"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi

target_json="$TRACE_DIR/fresh228_final_seed${TARGET_SEED}.json"
if [[ ! -s "$target_json" ]]; then
  echo "missing target trace: $target_json" >&2
  exit 1
fi

trace_args=(--trace-json "$target_json")
anchor_args=()
for seed in $ANCHOR_SEEDS; do
  anchor_json="$TRACE_DIR/fresh228_final_seed${seed}.json"
  if [[ ! -s "$anchor_json" ]]; then
    echo "missing anchor trace: $anchor_json" >&2
    exit 1
  fi
  anchor_args+=(--anchor-trace-json "$anchor_json")
done

/bin/python3 - "$TRACE_DIR" "$WORK_DIR" $GUARD_SEEDS <<'PY'
import json
import sys
from pathlib import Path

trace_dir = Path(sys.argv[1])
work_dir = Path(sys.argv[2])
seeds = sys.argv[3:]
active_keys = {
    'random_deconflict_weighted_active',
    'random_offroute_finish_weighted_active',
    'random_cte_recovery_weighted_active',
    'random_safe_finish_weighted_active',
}
work_dir.mkdir(parents=True, exist_ok=True)
for seed in seeds:
    source = trace_dir / f'fresh228_final_seed{seed}.json'
    target = work_dir / f'fresh228_final_seed{seed}_guard_only.json'
    payload = json.loads(source.read_text(encoding='utf-8'))
    for episode in payload.get('episode_metrics', []):
        for sample in episode.get('trace_samples', []):
            for agent in sample.get('agents', {}).values():
                diagnostics = agent.get('mask_diagnostics') or {}
                for key in active_keys:
                    if key in diagnostics:
                        diagnostics[key] = False
    target.write_text(json.dumps(payload, ensure_ascii=False, separators=(',', ':')), encoding='utf-8')
    print(target)
PY

for seed in $GUARD_SEEDS; do
  guard_json="$WORK_DIR/fresh228_final_seed${seed}_guard_only.json"
  if [[ ! -s "$guard_json" ]]; then
    echo "missing guard trace: $guard_json" >&2
    exit 1
  fi
  trace_args+=(--trace-json "$guard_json")
done

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  "${trace_args[@]}" \
  "${anchor_args[@]}" \
  --output "$OUTPUT" \
  --target-priority deconf \
  --target-shape-kinds deconf \
  --epochs "$EPOCHS" \
  --batch-size 64 \
  --learning-rate "$LEARNING_RATE" \
  --max-grad-norm "$MAX_GRAD_NORM" \
  --min-error-norm "$MIN_ERROR_NORM" \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --anchor-batch-size 512 \
  --deconf-weight "$DECONF_WEIGHT" \
  --deconf-linear-blend 1.0 \
  --deconf-omega-blend 0.45 \
  --low-speed-source-threshold 0.12 \
  --low-speed-weight 1.85 \
  --low-speed-weight-kinds deconf \
  --risk-guard-weight "$RISK_GUARD_WEIGHT" \
  --risk-guard-yield-only \
  --risk-guard-require-threat \
  --risk-guard-min-threat 0.10 \
  --risk-guard-max-separation 2.35 \
  --risk-guard-max-linear 0.10 \
  --risk-guard-min-starboard-omega 0.24 \
  --risk-guard-min-starboard-threat 0.12 \
  --risk-guard-min-distance 1.20 \
  --device cpu

echo "fresh233 guarded role-balance fit saved: $OUTPUT"