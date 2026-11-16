#!/bin/bash
# fresh645: collect geometry-PD teacher -> BC with causal-cut -> 50k RL.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"
CKPT=/mnt/data/checkpoints/usv_rl
DATASET="$CKPT/fresh645_teacher_geometry_pd_5usv.npz"
BC_INIT="$CKPT/fresh645_bc_init.pt"
ARCH_BASE="$CKPT/fresh639_checkpoints/fresh639_graze_pass_5usv_step_0317333.pt"
GATE2_JSON="$CKPT/fresh645_bc_init_eval.json"

if [[ ! -f "$DATASET" ]]; then
  python3 -m usv_rl.collect_multi_agent_teacher_dataset \
    --output "$DATASET" --episodes 100 --steps-per-episode 180 \
    --num-agents 5 --max-agents 5 --max-neighbors 4 \
    --scenario two_usv_head_on --scenario two_usv_head_on \
    --scenario two_usv_crossing --scenario two_usv_crossing \
    --scenario two_usv_overtaking --scenario three_usv_crossing \
    --scenario three_usv_overtaking --scenario three_usv_random_encounter \
    --scenario five_usv_dense_head_on --scenario five_usv_dense_crossing \
    --scenario five_usv_dense_overtaking --scenario cluster_escape \
    --scenario pentagram_convergence --scenario same_goal_queue \
    --scenario solo_navigation --scenario waypoint_turn \
    --action-mode speed_scale --action-speed-scale-min 0.30 \
    --cruise-speed 0.34 --max-angular-velocity 0.70 \
    --linear-delta-limit 0.30 --angular-delta-limit 0.70 \
    --sim-tau-linear 0.60 --sim-tau-angular 0.35 \
    --collision-distance 0.75 --near-miss-distance 0.95 \
    --scenario-neighbor-speed 0.34 --teacher-conflict-gate 1 \
    --base-ros-domain-id 310 --seed 43 \
    2>&1 | tee "$CKPT/fresh645_collect.log"
fi
python3 - "$DATASET" <<'EOF'
import sys, numpy as np
d=np.load(sys.argv[1],allow_pickle=True); a=d['actions']; e=d['engaged'].astype(bool); w=a[:,-1]; f=~e
print(f'[gate1] samples={len(a)} engaged={e.mean():.3f} free_omega_median={np.median(w[f]):.4f}')
if len(a)<20000 or not f.any() or abs(np.median(w[f]))>0.05:
    raise SystemExit('[gate1] FAIL: geometry-PD teacher dataset is malformed')
print('[gate1] PASS')
EOF

if [[ ! -f "$BC_INIT" ]]; then
  python3 -m usv_rl.bc_pretrain_attention_actor \
    --warmstart "$ARCH_BASE" --dataset "$DATASET" --output "$BC_INIT" \
    --epochs 200 --lr 1e-3 --batch-size 512 --device cpu \
    --input-dropout-slots 8,10 --input-dropout-prob 0.6 \
    2>&1 | tee "$CKPT/fresh645_bc.log"
fi
python3 -m usv_rl.evaluate_mappo_policy \
  --model "$BC_INIT" --episodes 12 --steps-per-episode 150 \
  --scenario two_usv_head_on --scenario two_usv_crossing --scenario cluster_escape \
  --scenario five_usv_dense_head_on --scenario five_usv_dense_crossing \
  --scenario five_usv_dense_overtaking --seed 4342 \
  --output-json "$GATE2_JSON" 2>&1 | tee "$CKPT/fresh645_gate2.log"
python3 - "$GATE2_JSON" <<'EOF'
import json, sys
s=json.load(open(sys.argv[1])); c=float(s.get('collision_rate',1)); p=float(s.get('mean_team_goal_progress_ratio',0))
print(f'[gate2] coll={c:.3f} timeout={s.get("timeout_rate")} prog={p:.3f}')
if c>0.60: raise SystemExit('[gate2] FAIL: collision regression')
print('[gate2] PASS')
EOF
exec bash "$SCRIPT_DIR/train_fresh645_geometry_pd_full_5usv.sh"
