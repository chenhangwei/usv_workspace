#!/bin/bash
# fresh642 pipeline: v3 teacher collect -> Gate1 dataset stats -> BC -> Gate2
# closed-loop eval -> RL polish. Each stage logs to $CKPT/fresh642_*.log.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

CKPT=/mnt/data/checkpoints/usv_rl
DATASET="$CKPT/fresh642_teacher_v3_5usv.npz"
BC_INIT="$CKPT/fresh642_bc_init.pt"
ARCH_BASE="$CKPT/fresh639_checkpoints/fresh639_graze_pass_5usv_step_0317333.pt"
GATE2_JSON="$CKPT/fresh642_bc_init_eval.json"

# ---------- Stage A: collect (teacher v3, TCPA gate ON) ----------
if [[ ! -f "$DATASET" ]]; then
  echo "[pipeline] Stage A: collecting v3 teacher dataset ..."
  python3 -m usv_rl.collect_multi_agent_teacher_dataset \
    --output "$DATASET" \
    --episodes 100 --steps-per-episode 180 \
    --num-agents 5 --max-agents 5 --max-neighbors 4 \
    --scenario two_usv_head_on --scenario two_usv_head_on \
    --scenario two_usv_crossing --scenario two_usv_crossing \
    --scenario two_usv_overtaking \
    --scenario three_usv_crossing --scenario three_usv_overtaking \
    --scenario three_usv_random_encounter \
    --scenario five_usv_dense_head_on --scenario five_usv_dense_crossing \
    --scenario five_usv_dense_overtaking \
    --scenario cluster_escape --scenario pentagram_convergence \
    --scenario same_goal_queue \
    --scenario solo_navigation --scenario waypoint_turn \
    --action-mode speed_scale --action-speed-scale-min 0.30 \
    --cruise-speed 0.34 --max-angular-velocity 0.70 \
    --linear-delta-limit 0.30 --angular-delta-limit 0.70 \
    --sim-tau-linear 0.60 --sim-tau-angular 0.35 \
    --collision-distance 0.75 --near-miss-distance 0.95 \
    --scenario-neighbor-speed 0.34 \
    --teacher-conflict-gate 1 \
    --base-ros-domain-id 310 --seed 41 \
    2>&1 | tee "$CKPT/fresh642_collect.log"
else
  echo "[pipeline] Stage A skipped: $DATASET exists"
fi

# ---------- Gate 1: dataset sanity ----------
echo "[pipeline] Gate 1: dataset stats ..."
python3 - "$DATASET" <<'EOF'
import sys
import numpy as np
d = np.load(sys.argv[1], allow_pickle=True)
act = d['actions']; eng = d['engaged'].astype(bool)
omega = act[:, -1]; free = ~eng
ef = float(eng.mean())
fm = float(np.median(omega[free])) if free.any() else 0.0
scale_ok = bool((act[free, 0] == 1.0).all()) if free.any() else True
print(f'[gate1] samples={len(act)} engaged_fraction={ef:.3f} '
      f'open_water_omega_median={fm:.4f} open_water_scale_all_1={scale_ok}')
fails = []
# encounter-weighted scenario table: dense five-boat scenes engage ~1.0 by the
# density fallback, so the healthy full-collection range is much higher than
# the smoke mix (verified 2026-08-24: 0.718 with perfect open-water stats)
if not (0.03 <= ef <= 0.85):
    fails.append(f'engaged_fraction {ef:.3f} outside [0.03, 0.85]')
if abs(fm) > 0.05:
    fails.append(f'open-water omega median {fm:.4f} > 0.05 (right drift not fixed)')
if not scale_ok:
    fails.append('open-water scale not uniformly 1.0')
if eng.any():
    right = float((omega[eng] < -0.02).mean()); left = float((omega[eng] > 0.02).mean())
    # latch tail keeps pure goal-tracking samples engaged, so median ~0 is fine;
    # the encounter set as a whole must still lean starboard.
    if right < left:
        fails.append(f'engaged right_turn_fraction {right:.3f} < left {left:.3f}')
if fails:
    print('[gate1] FAIL: ' + '; '.join(fails)); sys.exit(1)
print('[gate1] PASS')
EOF

# ---------- Stage B: BC pretraining ----------
if [[ ! -f "$BC_INIT" ]]; then
  echo "[pipeline] Stage B: BC pretraining ..."
  python3 -m usv_rl.bc_pretrain_attention_actor \
    --warmstart "$ARCH_BASE" \
    --dataset "$DATASET" \
    --output "$BC_INIT" \
    --epochs 200 --lr 1e-3 --batch-size 512 --device cpu \
    2>&1 | tee "$CKPT/fresh642_bc.log"
else
  echo "[pipeline] Stage B skipped: $BC_INIT exists"
fi
[[ -f "$BC_INIT" ]] || { echo "[pipeline] FATAL: BC init missing"; exit 1; }

# ---------- Gate 2: BC closed-loop eval (soft gate, hard-fail only on disaster) ----------
if [[ ! -f "$GATE2_JSON" ]]; then
  echo "[pipeline] Gate 2: closed-loop eval of BC init ..."
  python3 -m usv_rl.evaluate_mappo_policy \
    --model "$BC_INIT" --episodes 12 --steps-per-episode 150 \
    --scenario two_usv_head_on --scenario two_usv_crossing \
    --scenario cluster_escape --scenario five_usv_dense_head_on \
    --scenario five_usv_dense_crossing --scenario five_usv_dense_overtaking \
    --seed 4242 --output-json "$GATE2_JSON" \
    2>&1 | tee "$CKPT/fresh642_gate2.log"
fi
python3 - "$GATE2_JSON" <<'EOF'
import json, sys
s = json.load(open(sys.argv[1]))
coll = float(s.get('collision_rate', 1.0))
prog = float(s.get('mean_team_goal_progress_ratio', 0.0))
print(f'[gate2] BC init: coll={coll:.2f} timeout={s.get("timeout_rate")} prog={prog:.3f} '
      f'(51427 baseline coll=0.50; RL polish still to come)')
if coll > 0.90:
    print('[gate2] FAIL: catastrophic collision rate, teacher/BC broken'); sys.exit(1)
print('[gate2] PASS (soft)')
EOF

# ---------- Stage C: RL polish ----------
echo "[pipeline] Stage C: launching fresh642 RL polish ..."
exec bash "$SCRIPT_DIR/train_fresh642_duck_v3_5usv.sh"
