#!/usr/bin/env bash
# fresh646 DAgger round: roll out the BC student in the dense five-USV scenarios,
# label with the (fixed) geometry-PD teacher, aggregate with the base dataset,
# retrain BC, and re-run the 6-scenario gate. No RL.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"
CKPT=/mnt/data/checkpoints/usv_rl
TAG="${TAG:-fresh646}"
ROUND="${ROUND:-1}"
BASE_DATASET="${BASE_DATASET:-$CKPT/${TAG}_teacher_geometry_pd_5usv.npz}"
STUDENT="${STUDENT:-$CKPT/${TAG}_bc_init.pt}"
ARCH_BASE="$CKPT/fresh639_checkpoints/fresh639_graze_pass_5usv_step_0317333.pt"
DAGGER_DS="$CKPT/${TAG}_dagger_r${ROUND}_dense.npz"
MERGED="$CKPT/${TAG}_dagger_r${ROUND}_merged.npz"
OUT="$CKPT/${TAG}_bc_dagger_r${ROUND}.pt"
GATE_JSON="$CKPT/${TAG}_bc_dagger_r${ROUND}_eval.json"

if [[ ! -f "$DAGGER_DS" ]]; then
  python3 -m usv_rl.collect_multi_agent_teacher_dataset \
    --output "$DAGGER_DS" --episodes 36 --steps-per-episode 180 \
    --num-agents 5 --max-agents 5 --max-neighbors 4 \
    --scenario five_usv_dense_head_on --scenario five_usv_dense_crossing \
    --scenario five_usv_dense_overtaking \
    --student "$STUDENT" --student-mix "${STUDENT_MIX:-1.0}" \
    --action-mode speed_scale --action-speed-scale-min 0.30 \
    --cruise-speed 0.34 --max-angular-velocity 0.70 \
    --linear-delta-limit 0.30 --angular-delta-limit 0.70 \
    --sim-tau-linear 0.60 --sim-tau-angular 0.35 \
    --collision-distance 0.75 --near-miss-distance 0.95 \
    --scenario-neighbor-speed 0.34 --teacher-conflict-gate 1 \
    --base-ros-domain-id 330 --seed $((1000 + ROUND * 100)) \
    2>&1 | tee "$CKPT/${TAG}_dagger_r${ROUND}_collect.log"
fi

python3 - "$BASE_DATASET" "$DAGGER_DS" "$MERGED" <<'EOF'
import sys, numpy as np
a=np.load(sys.argv[1],allow_pickle=True); b=np.load(sys.argv[2],allow_pickle=True)
out={k: np.concatenate([a[k], b[k]]) for k in ('observations','actions','scenario_names','engaged')}
for k in ('action_mode','action_speed_scale_min','max_neighbors'): out[k]=a[k]
np.savez_compressed(sys.argv[3], **out)
print(f'[merge] base={len(a["actions"])} dagger={len(b["actions"])} merged={len(out["actions"])}')
EOF

python3 -m usv_rl.bc_pretrain_attention_actor \
  --warmstart "$ARCH_BASE" --dataset "$MERGED" --output "$OUT" \
  --epochs 200 --lr 1e-3 --batch-size 512 --device cpu \
  --input-dropout-slots 8,10 --input-dropout-prob 0.6 \
  --engaged-weight "${ENGAGED_WEIGHT:-1.0}" \
  2>&1 | tee "$CKPT/${TAG}_bc_dagger_r${ROUND}.log"

python3 -m usv_rl.evaluate_mappo_policy \
  --model "$OUT" --episodes 12 --steps-per-episode 150 \
  --scenario two_usv_head_on --scenario two_usv_crossing --scenario cluster_escape \
  --scenario five_usv_dense_head_on --scenario five_usv_dense_crossing \
  --scenario five_usv_dense_overtaking --seed 4342 \
  --output-json "$GATE_JSON" 2>&1 | tee "$CKPT/${TAG}_bc_dagger_r${ROUND}_gate.log"
python3 - "$GATE_JSON" <<'EOF'
import json, sys
d=json.load(open(sys.argv[1]))
print('[gate]', {k: round(float(d[k]), 3) for k in ['collision_rate','success_rate','timeout_rate','worst_pairwise_min_separation','mean_heading_error','mean_cross_track_error']})
for s,v in d['scenario_summaries'].items():
    print(f"  {s:28s} coll={v['collision_rate']:.1f} succ={v['success_rate']:.1f} minsep={v['worst_episode_min_separation']:.2f} HE={v['mean_heading_error']:.2f} CTE={v['mean_cross_track_error']:.2f}")
EOF
echo "[${TAG}] DAgger round $ROUND done: $OUT"
