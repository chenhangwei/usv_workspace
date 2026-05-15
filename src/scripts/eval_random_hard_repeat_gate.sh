#!/bin/bash
# Repeated hard-seed gate for current deterministic random-encounter geometry.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh320b_pairwise_guard_from_fresh318_step1260.pt}"
OUT="${OUT:-/tmp/random_hard_repeat_gate}"
LABEL="${LABEL:-random_hard_repeat_gate}"
SEEDS="${SEEDS:-1458 1460 1461 1463}"
REPEATS="${REPEATS:-3}"
STEPS="${STEPS:-1200}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-240.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-220.0}"
BASE_DOMAIN="${BASE_DOMAIN:-120}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-1800}"
TRACE_STRIDE="${TRACE_STRIDE:-0}"
TRACE_EVENT_SEPARATION="${TRACE_EVENT_SEPARATION:-0}"
TRACE_EVENT_WINDOW="${TRACE_EVENT_WINDOW:-12}"
TRACE_EVENT_RAW_OBSERVATION="${TRACE_EVENT_RAW_OBSERVATION:-0}"
TRACE_COLLISION_RAW_OBSERVATION="${TRACE_COLLISION_RAW_OBSERVATION:-0}"

mkdir -p "$OUT"
SUMMARY="$OUT/summary.tsv"
AGGREGATE="$OUT/aggregate.tsv"
printf 'repeat\tseed\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tcte\tomega\tsteps\tjson\n' > "$SUMMARY"

if [[ ! -f "$MODEL" ]]; then
  echo "missing model: $MODEL" >&2
  exit 1
fi

domain_index=0
for repeat in $(seq 1 "$REPEATS"); do
  for seed in $SEEDS; do
    run_out="$OUT/repeat${repeat}_seed${seed}"
    run_label="${LABEL}_r${repeat}_s${seed}"
    domain=$((BASE_DOMAIN + domain_index))
    rm -rf "$run_out"
    echo "----- ${run_label} domain=${domain} -----"
    env \
      MODEL="$MODEL" \
      OUT="$run_out" \
      LABEL="$run_label" \
      SEEDS="$seed" \
      STEPS="$STEPS" \
      EPISODE_TIMEOUT="$EPISODE_TIMEOUT" \
      NO_PROGRESS_TIMEOUT="$NO_PROGRESS_TIMEOUT" \
      BASE_DOMAIN="$domain" \
      EVAL_TIMEOUT="$EVAL_TIMEOUT" \
      TRACE_STRIDE="$TRACE_STRIDE" \
      TRACE_EVENT_SEPARATION="$TRACE_EVENT_SEPARATION" \
      TRACE_EVENT_WINDOW="$TRACE_EVENT_WINDOW" \
      TRACE_EVENT_RAW_OBSERVATION="$TRACE_EVENT_RAW_OBSERVATION" \
      TRACE_COLLISION_RAW_OBSERVATION="$TRACE_COLLISION_RAW_OBSERVATION" \
      bash scripts/eval_random_hard_focus.sh
    tail -n +2 "$run_out/summary.tsv" | awk -v repeat="$repeat" 'BEGIN { OFS="\t" } { print repeat, $0 }' >> "$SUMMARY"
    domain_index=$((domain_index + 1))
  done
done

/bin/python3 - "$SUMMARY" "$AGGREGATE" <<'PY'
import csv
import sys
from collections import defaultdict

summary_path, aggregate_path = sys.argv[1:3]
rows = []
with open(summary_path, 'r', encoding='utf-8') as handle:
    reader = csv.DictReader(handle, delimiter='\t')
    for row in reader:
        rows.append(row)

by_seed = defaultdict(list)
for row in rows:
    by_seed[row['seed']].append(row)

fields = [
    'seed',
    'runs',
    'collisions',
    'successes',
    'timeouts',
    'collision_free',
    'min_sep_worst',
    'progress_mean',
    'steps_mean',
]
with open(aggregate_path, 'w', encoding='utf-8', newline='') as handle:
    writer = csv.DictWriter(handle, delimiter='\t', fieldnames=fields)
    writer.writeheader()
    for seed in sorted(by_seed, key=lambda item: int(item)):
        seed_rows = by_seed[seed]
        runs = len(seed_rows)
        collisions = sum(float(row['collision']) > 0.0 for row in seed_rows)
        successes = sum(float(row['success']) > 0.0 for row in seed_rows)
        timeouts = sum(float(row['timeout']) > 0.0 for row in seed_rows)
        min_sep_worst = min(float(row['sep']) for row in seed_rows)
        progress_mean = sum(float(row['progress']) for row in seed_rows) / max(1, runs)
        steps_mean = sum(float(row['steps']) for row in seed_rows) / max(1, runs)
        writer.writerow({
            'seed': seed,
            'runs': runs,
            'collisions': collisions,
            'successes': successes,
            'timeouts': timeouts,
            'collision_free': int(collisions == 0),
            'min_sep_worst': f'{min_sep_worst:.6f}',
            'progress_mean': f'{progress_mean:.6f}',
            'steps_mean': f'{steps_mean:.6f}',
        })

total_collisions = sum(float(row['collision']) > 0.0 for row in rows)
print(f'repeat_gate_runs={len(rows)} collisions={total_collisions}')
PY

echo "summary: $SUMMARY"
cat "$SUMMARY"
echo "aggregate: $AGGREGATE"
cat "$AGGREGATE"