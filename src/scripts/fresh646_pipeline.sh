#!/usr/bin/env bash
# fresh646: fresh645 pipeline with the teacher CTE sign fixed (+cte_p_gain*CTE).
# Stops after the BC-init closed-loop gate; RL is NOT launched (user decision).
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"
CKPT=/mnt/data/checkpoints/usv_rl
TAG="${TAG:-fresh646}"
DATASET="$CKPT/${TAG}_teacher_geometry_pd_5usv.npz"
BC_INIT="$CKPT/${TAG}_bc_init.pt"
ARCH_BASE="$CKPT/fresh639_checkpoints/fresh639_graze_pass_5usv_step_0317333.pt"
GATE2_JSON="$CKPT/${TAG}_bc_init_eval.json"

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
    2>&1 | tee "$CKPT/${TAG}_collect.log"
fi

# gate1: free-navigation labels must actually steer back to the route.
python3 - "$DATASET" <<'EOF'
import sys, numpy as np
d=np.load(sys.argv[1],allow_pickle=True); a=d['actions']; o=d['observations']; e=d['engaged'].astype(bool)
w=a[:,-1]; f=~e; he=np.degrees(np.arctan2(o[:,5],o[:,6])); cte=o[:,11]
print(f'[gate1] samples={len(a)} engaged={e.mean():.3f} free_omega_median={np.median(w[f]):+.4f} free_|CTE|_p90={np.percentile(np.abs(cte[f]),90):.2f} free_|HE|_med={np.median(np.abs(he[f])):.1f}')
pos=f&(he>20); neg=f&(he<-20)
print(f'[gate1] free HE>+20 omega_med={np.median(w[pos]) if pos.any() else 0:+.3f} (n={pos.sum()}) | HE<-20 omega_med={np.median(w[neg]) if neg.any() else 0:+.3f} (n={neg.sum()})')
ok = len(a)>=20000 and f.any() and abs(np.median(w[f]))<0.05 \
     and np.percentile(np.abs(cte[f]),90)<1.5 \
     and (not pos.any() or np.median(w[pos])>0.15) and (not neg.any() or np.median(w[neg])<-0.15)
if not ok: raise SystemExit('[gate1] FAIL: teacher still not tracking the route')
print('[gate1] PASS')
EOF

if [[ ! -f "$BC_INIT" ]]; then
  python3 -m usv_rl.bc_pretrain_attention_actor \
    --warmstart "$ARCH_BASE" --dataset "$DATASET" --output "$BC_INIT" \
    --epochs 200 --lr 1e-3 --batch-size 512 --device cpu \
    --input-dropout-slots 8,10 --input-dropout-prob 0.6 \
    2>&1 | tee "$CKPT/${TAG}_bc.log"
fi

python3 -m usv_rl.evaluate_mappo_policy \
  --model "$BC_INIT" --episodes 12 --steps-per-episode 150 \
  --scenario two_usv_head_on --scenario two_usv_crossing --scenario cluster_escape \
  --scenario five_usv_dense_head_on --scenario five_usv_dense_crossing \
  --scenario five_usv_dense_overtaking --seed 4342 \
  --output-json "$GATE2_JSON" 2>&1 | tee "$CKPT/${TAG}_gate2.log"
python3 - "$GATE2_JSON" <<'EOF'
import json, sys
s=json.load(open(sys.argv[1]))
print('[gate2]', {k: round(float(s.get(k, float("nan"))), 3) for k in
      ['collision_rate','success_rate','timeout_rate','worst_pairwise_min_separation','mean_heading_error','mean_cross_track_error','mean_team_goal_progress_ratio']})
print('[gate2] fresh645 reference: coll 0.417 succ 0.583 HE 0.635rad CTE 0.85m')
EOF
echo "[${TAG}] BC init ready: $BC_INIT (RL not launched)"
