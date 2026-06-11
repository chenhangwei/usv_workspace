#!/usr/bin/env bash
# Reusable head-on eval: 5 seeds, no shield, single two_usv_head_on scenario.
#
# Usage:
#   MODEL=<path-to-ckpt> OUT_DIR=<dir> LABEL=<label> bash scripts/eval_headon_5seeds.sh
#   SEEDS="2001 2002 2003 2004 2005"    (override seed list if desired)
#
# Outputs:
#   $OUT_DIR/seed<S>.json for each seed
#   $OUT_DIR/summary.tsv with one row per seed + AVG row

set -eo pipefail
cd "$(dirname "$0")/.."

# shellcheck disable=SC1091
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

MODEL="${MODEL:?MODEL=<ckpt> required}"
OUT_DIR="${OUT_DIR:?OUT_DIR=<dir> required}"
LABEL="${LABEL:-headon_eval}"
SCENARIO="${SCENARIO:-two_usv_head_on}"
SEEDS="${SEEDS:-2001 2002 2003 2004 2005}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-90}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-60}"
STEPS="${STEPS:-600}"
NUM_AGENTS="${NUM_AGENTS:-2}"
MAX_AGENTS="${MAX_AGENTS:-2}"
MAX_NEIGHBORS="${MAX_NEIGHBORS:-1}"

mkdir -p "$OUT_DIR"
echo "[eval_headon] LABEL=$LABEL MODEL=$MODEL OUT_DIR=$OUT_DIR SEEDS=$SEEDS"

for seed in $SEEDS; do
  echo "=== $LABEL seed=$seed ==="
  timeout 200 python3 -m usv_rl.evaluate_mappo_policy \
    --policy auto \
    --model "$MODEL" \
    --scenario "$SCENARIO" \
    --num-agents "$NUM_AGENTS" --max-agents "$MAX_AGENTS" --max-neighbors "$MAX_NEIGHBORS" \
    --episodes 1 \
    --steps-per-episode "$STEPS" \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --seed "$seed" \
    --device cpu \
    --output-json "$OUT_DIR/seed${seed}.json" 2>&1 | tail -3
done

echo "---SUMMARY for $LABEL---"
python3 - <<PY
import json, glob, os
files=sorted(glob.glob(os.path.join("$OUT_DIR","seed*.json")))
print(f'{"seed":>5} {"coll":>5} {"succ":>5} {"to":>4} {"min_sep":>8} {"goalP":>6} {"colP":>6} {"effi":>5}')
cs=ss=ts=0; mseps=[]; gps=[]
rows=[]
for f in files:
    d=json.load(open(f))
    seed=int(os.path.basename(f).replace("seed","").replace(".json",""))
    c=d['collision_rate']; s=d['success_rate']; t=d['timeout_rate']
    msep=d['worst_episode_min_separation']; gp=d['mean_team_goal_progress_ratio']
    cP=d['mean_colregs_compliance_ratio']
    eff=d.get('mean_progress_efficiency',-1)
    row=f'{seed:>5} {c:>5.2f} {s:>5.2f} {t:>4.2f} {msep:>8.3f} {gp:>6.3f} {cP:>6.3f} {eff:>5.2f}'
    print(row); rows.append(row)
    cs+=c; ss+=s; ts+=t; mseps.append(msep); gps.append(gp)
n=len(files) or 1
avg=f'  AVG {cs/n:>5.2f} {ss/n:>5.2f} {ts/n:>4.2f} {min(mseps):>8.3f} {sum(gps)/n:>6.3f}'
print('---')
print(avg)
with open(os.path.join("$OUT_DIR","summary.tsv"),"w") as fp:
    fp.write("seed\tcollision\tsuccess\ttimeout\tmin_sep_worst\tgoal_progress\tcolregs_compliance\tprogress_eff\n")
    for row in rows: fp.write(row.replace(" ","\t").strip()+"\n")
    fp.write(avg.replace(" ","\t").strip()+"\n")
PY
