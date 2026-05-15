#!/usr/bin/env bash
set -uo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh329_balanced_collision_raw_guard_from_fresh321.pt}"
OUT="${OUT:-/tmp/collision_raw_gate}"
LABEL="${LABEL:-collision_raw_gate}"
SEEDS="${SEEDS:-1456 1457 1458 1459 1460 1461 1462 1463 1464}"
REPEATS="${REPEATS:-1}"
STEPS="${STEPS:-1200}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-240.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-220.0}"
BASE_DOMAIN="${BASE_DOMAIN:-300}"
MAX_DOMAIN="${MAX_DOMAIN:-232}"
MIN_DOMAIN="${MIN_DOMAIN:-0}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-1200}"
TRACE_STRIDE="${TRACE_STRIDE:-0}"
TRACE_EVENT_SEPARATION="${TRACE_EVENT_SEPARATION:-0}"
TRACE_EVENT_WINDOW="${TRACE_EVENT_WINDOW:-12}"
TRACE_EVENT_RAW_OBSERVATION="${TRACE_EVENT_RAW_OBSERVATION:-0}"
TRACE_COLLISION_RAW_OBSERVATION="${TRACE_COLLISION_RAW_OBSERVATION:-1}"

mkdir -p "$OUT"
SUMMARY="$OUT/summary.tsv"
AGGREGATE="$OUT/aggregate.tsv"
printf 'label\trepeat\tseed\tdomain\tstatus\texit_code\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tcte\tomega\tsteps\tjson\n' > "$SUMMARY"

if [[ ! -f "$MODEL" ]]; then
  echo "missing model: $MODEL" >&2
  exit 1
fi

domain_index=0
domain_span=$((MAX_DOMAIN - MIN_DOMAIN + 1))
if [[ "$domain_span" -le 0 ]]; then
  echo "invalid domain range: MIN_DOMAIN=$MIN_DOMAIN MAX_DOMAIN=$MAX_DOMAIN" >&2
  exit 1
fi
for repeat in $(seq 1 "$REPEATS"); do
  for seed in $SEEDS; do
    domain=$((BASE_DOMAIN + domain_index))
    if [[ "$domain" -gt "$MAX_DOMAIN" || "$domain" -lt "$MIN_DOMAIN" ]]; then
      domain=$((MIN_DOMAIN + ((domain - MIN_DOMAIN) % domain_span)))
      if [[ "$domain" -lt "$MIN_DOMAIN" ]]; then
        domain=$((domain + domain_span))
      fi
    fi
    run_out="$OUT/repeat${repeat}_seed${seed}"
    run_label="${LABEL}_r${repeat}_s${seed}"
    rm -rf "$run_out"
    mkdir -p "$run_out"
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
      bash scripts/eval_random_hard_focus.sh > "$run_out/driver.log" 2>&1
    status=$?
    json="$run_out/seed${seed}.json"
    if [[ -s "$run_out/summary.tsv" && "$(wc -l < "$run_out/summary.tsv")" -gt 1 ]]; then
      tail -n +2 "$run_out/summary.tsv" | awk -v label="$LABEL" -v repeat="$repeat" -v domain="$domain" -v status="ok" -v code="$status" 'BEGIN { OFS="\t" } { print label, repeat, $1, domain, status, code, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11 }' >> "$SUMMARY"
    elif [[ -s "$json" ]]; then
      /bin/python3 - "$json" "$LABEL" "$repeat" "$seed" "$domain" "$status" "$SUMMARY" <<'PY'
import json
import sys

json_path, label, repeat, seed, domain, status, summary_path = sys.argv[1:8]
with open(json_path, 'r', encoding='utf-8') as handle:
    data = json.load(handle)
episode = (data.get('episode_metrics') or [{}])[0]
row = [
    label,
    repeat,
    seed,
    domain,
    'json_only',
    status,
    f"{float(data.get('collision_rate', 0.0)):.6f}",
    f"{float(data.get('success_rate', 0.0)):.6f}",
    f"{float(data.get('timeout_rate', 0.0)):.6f}",
    f"{float(episode.get('team_goal_progress_ratio', 0.0)):.6f}",
    f"{float(episode.get('goal_completion_ratio', 0.0)):.6f}",
    f"{float(episode.get('episode_min_separation', 0.0)):.6f}",
    f"{float(episode.get('mean_cross_track_error', 0.0)):.6f}",
    f"{float(episode.get('omega_flip_count', 0.0)):.6f}",
    f"{float(episode.get('steps', 0.0)):.6f}",
    json_path,
]
with open(summary_path, 'a', encoding='utf-8') as handle:
    handle.write('\t'.join(row) + '\n')
PY
    else
      printf '%s\t%s\t%s\t%s\tno_json\t%s\tNA\tNA\tNA\tNA\tNA\tNA\tNA\tNA\tNA\t%s\n' "$LABEL" "$repeat" "$seed" "$domain" "$status" "$json" >> "$SUMMARY"
    fi
    domain_index=$((domain_index + 1))
  done
done

/bin/python3 - "$SUMMARY" "$AGGREGATE" <<'PY'
import csv
import sys
from collections import defaultdict

summary_path, aggregate_path = sys.argv[1:3]
rows = list(csv.DictReader(open(summary_path, encoding='utf-8'), delimiter='\t'))
by_seed = defaultdict(list)
for row in rows:
    by_seed[row['seed']].append(row)

fields = ['seed', 'runs', 'completed', 'no_json', 'collisions', 'successes', 'timeouts', 'min_sep_worst', 'progress_mean']
with open(aggregate_path, 'w', encoding='utf-8', newline='') as handle:
    writer = csv.DictWriter(handle, delimiter='\t', fieldnames=fields)
    writer.writeheader()
    for seed in sorted(by_seed, key=lambda item: int(item)):
        seed_rows = by_seed[seed]
        numeric = [row for row in seed_rows if row['collision'] != 'NA']
        collisions = sum(float(row['collision']) > 0.0 for row in numeric)
        successes = sum(float(row['success']) > 0.0 for row in numeric)
        timeouts = sum(float(row['timeout']) > 0.0 for row in numeric)
        no_json = sum(row['status'] == 'no_json' for row in seed_rows)
        min_sep = min((float(row['sep']) for row in numeric), default=float('nan'))
        progress = sum(float(row['progress']) for row in numeric) / len(numeric) if numeric else float('nan')
        writer.writerow({
            'seed': seed,
            'runs': len(seed_rows),
            'completed': len(numeric),
            'no_json': no_json,
            'collisions': collisions,
            'successes': successes,
            'timeouts': timeouts,
            'min_sep_worst': f'{min_sep:.6f}',
            'progress_mean': f'{progress:.6f}',
        })

print(f'runs={len(rows)} collisions={sum(float(row["collision"]) > 0.0 for row in rows if row["collision"] != "NA")} no_json={sum(row["status"] == "no_json" for row in rows)}')
PY

echo "summary: $SUMMARY"
cat "$SUMMARY"
echo "aggregate: $AGGREGATE"
cat "$AGGREGATE"