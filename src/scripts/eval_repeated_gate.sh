#!/bin/bash
# Repeated-seed gate (P0 reliable evaluation).
#
# Why: single-run gates are noise-driven on a nondeterministic ROS/sim stack
# (see test_reports/fresh320_pairwise_role_guard_20260512.md). This runs each
# (scenario, seed) REPEATS times on distinct ROS domains and aggregates so that
# decisions are made on distributions, not a single lucky/unlucky run.
#
# Hard gate: 0 collisions across all repeats. Only then compare success rate.
#
# Usage:
#   MODEL=/mnt/data/checkpoints/usv_rl/fresh524_driver.pt \
#   OUT=/tmp/fresh524_gate LABEL=fresh524 REPEATS=3 \
#   bash scripts/eval_repeated_gate.sh

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

MODEL="${MODEL:?need MODEL=}"
OUT="${OUT:-/tmp/repeated_gate}"
LABEL="${LABEL:-repeated}"
REPEATS="${REPEATS:-3}"
STEPS="${STEPS:-400}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-110.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-60.0}"
BASE_DOMAIN="${BASE_DOMAIN:-160}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-1800}"

# (scenario, seed) matrix: default is the fresh523 eval set for direct A/B.
# Override with CASES_OVERRIDE (semicolon-separated "scenario seed" pairs), e.g.
# CASES_OVERRIDE="three_usv_crossing 5231;three_usv_overtaking 5231" for a
# 3-USV-only gate on a 3-agent checkpoint (avoids 5-agent scenario errors).
if [[ -n "${CASES_OVERRIDE:-}" ]]; then
  IFS=';' read -ra CASES <<< "$CASES_OVERRIDE"
else
  CASES=(
    "pentagram_convergence 5211"
    "pentagram_convergence 5212"
    "pentagram_convergence 5213"
    "five_usv_dense_crossing 5221"
    "five_usv_dense_crossing 5222"
    "three_usv_crossing 5231"
    "three_usv_crossing 5232"
    "three_usv_overtaking 5231"
    "three_usv_overtaking 5232"
    "three_usv_random_encounter 5231"
    "three_usv_random_encounter 5232"
  )
fi

mkdir -p "$OUT"
SUMMARY="$OUT/summary.tsv"
printf 'scenario\tseed\trep\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tcte\tomega\tsteps\tjson\n' > "$SUMMARY"

domain_index=0

eval_one() {
  local scenario="$1"
  local seed="$2"
  local rep="$3"
  local domain="$4"
  local label="${scenario}_seed${seed}_r${rep}"
  local json="$OUT/${label}.json"
  local log="$OUT/${label}.log"

  echo "----- ${LABEL} eval scenario=${scenario} seed=${seed} rep=${rep} domain=${domain} -----"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  ROS_DOMAIN_ID="$domain" timeout "$EVAL_TIMEOUT" /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy mappo \
    --model "$MODEL" \
    --episodes 1 \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --seed "$seed" \
    --output-json "$json" \
    --scenario "$scenario" \
    > "$log" 2>&1 || echo "  (run exited non-zero / timed out; recorded as failure)"

  /bin/python3 - "$json" "$scenario" "$seed" "$rep" "$SUMMARY" <<'PY'
import json
import sys

json_path, scenario, seed, rep, summary_path = sys.argv[1:6]
try:
    with open(json_path, 'r', encoding='utf-8') as handle:
        data = json.load(handle)
    episode = data['episode_metrics'][0]
    row = [
        scenario, seed, rep,
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
except Exception as exc:  # missing/corrupt json => count as a hard failure
    row = [scenario, seed, rep, "1.000000", "0.000000", "0.000000",
           "0.000000", "0.000000", "0.000000", "0.000000", "0.000000",
           "0.000000", f"ERROR:{exc}"]
with open(summary_path, 'a', encoding='utf-8') as handle:
    handle.write('\t'.join(row) + '\n')
PY
}

for case in "${CASES[@]}"; do
  read -r scenario seed <<< "$case"
  for rep in $(seq 1 "$REPEATS"); do
    eval_one "$scenario" "$seed" "$rep" "$((BASE_DOMAIN + domain_index))"
    domain_index=$((domain_index + 1))
  done
done

echo
echo "===== ${LABEL} per-run summary ====="
column -t "$SUMMARY"

# Aggregate per (scenario, seed) and emit verdict.
AGG="$OUT/aggregate.tsv"
/bin/python3 - "$SUMMARY" "$AGG" <<'PY'
import sys
from collections import defaultdict

summary_path, agg_path = sys.argv[1:3]
rows = [l.split('\t') for l in open(summary_path).read().splitlines()[1:]]

groups = defaultdict(list)
for r in rows:
    groups[(r[0], r[1])].append(r)

header = ["scenario", "seed", "runs", "collisions", "successes",
          "timeouts", "worst_sep", "mean_sep", "mean_progress",
          "mean_goal", "verdict"]
out_lines = ['\t'.join(header)]
n_pass = 0
fragile = []
failed = []
for (scenario, seed), rs in sorted(groups.items()):
    runs = len(rs)
    collisions = sum(1 for r in rs if float(r[3]) > 0.0)
    successes = sum(1 for r in rs if float(r[4]) > 0.5)
    timeouts = sum(1 for r in rs if float(r[5]) > 0.5)
    seps = [float(r[8]) for r in rs]
    worst_sep = min(seps) if seps else 0.0
    mean_sep = sum(seps) / len(seps) if seps else 0.0
    mean_prog = sum(float(r[6]) for r in rs) / runs
    mean_goal = sum(float(r[7]) for r in rs) / runs
    # Hard gate: any collision across repeats is a fail.
    if collisions > 0:
        verdict = "FAIL_COLLISION"
        failed.append((scenario, seed, f"{collisions}/{runs} coll"))
    elif successes / runs < 0.5:
        verdict = "FAIL_SUCCESS"
        failed.append((scenario, seed, f"{successes}/{runs} succ"))
    else:
        verdict = "PASS"
        n_pass += 1
    out_lines.append('\t'.join([
        scenario, seed, str(runs), str(collisions), str(successes),
        str(timeouts), f"{worst_sep:.3f}", f"{mean_sep:.3f}",
        f"{mean_prog:.3f}", f"{mean_goal:.3f}", verdict,
    ]))

open(agg_path, 'w').write('\n'.join(out_lines) + '\n')

print("\n===== AGGREGATE (per scenario,seed) =====")
import subprocess
print('\n'.join(out_lines))
print(f"\nVERDICT: {n_pass}/{len(groups)} pass (hard gate: 0 collisions across all repeats)")
for s, seed, why in failed:
    print(f"  FAIL {s} seed={seed} ({why})")
PY

echo
echo "Aggregate written to $AGG"
