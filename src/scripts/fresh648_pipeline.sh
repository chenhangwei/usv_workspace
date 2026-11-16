#!/usr/bin/env bash
# fresh648: fix the open-water dead band seen in SITL 2026-09-04 (student holds
# HE ~ +10deg with omega ~ 0, drifts starboard, then saw-tooths at |HE|>40).
# Root cause: teacher rollouts keep |HE|<2deg in open water, so BC never sees
# the corrective labels for HE/CTE offsets. DART: roll the teacher out with
# executed-omega noise + slow bias (labels stay clean), merge with fresh647 base.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"
CKPT=/mnt/data/checkpoints/usv_rl
TAG="${TAG:-fresh648}"
BASE="${BASE_DATASET:-$CKPT/fresh647_teacher_geometry_pd_5usv.npz}"
DART="$CKPT/${TAG}_dart_open_water.npz"
MERGED="$CKPT/${TAG}_merged.npz"
BC_INIT="$CKPT/${TAG}_bc_init.pt"
ARCH_BASE="$CKPT/fresh639_checkpoints/fresh639_graze_pass_5usv_step_0317333.pt"
GATE_JSON="$CKPT/${TAG}_bc_init_eval.json"

if [[ ! -f "$DART" ]]; then
  python3 -m usv_rl.collect_multi_agent_teacher_dataset \
    --output "$DART" --episodes 48 --steps-per-episode 180 \
    --num-agents 5 --max-agents 5 --max-neighbors 4 \
    --scenario solo_navigation --scenario waypoint_turn \
    --scenario solo_navigation --scenario two_usv_overtaking \
    --scenario waypoint_turn --scenario three_usv_crossing \
    --exec-omega-noise-std "${DART_NOISE:-0.25}" --exec-omega-bias-std "${DART_BIAS:-0.30}" --exec-bias-hold 40 \
    --action-mode speed_scale --action-speed-scale-min 0.30 \
    --cruise-speed 0.34 --max-angular-velocity 0.70 \
    --linear-delta-limit 0.30 --angular-delta-limit 0.70 \
    --sim-tau-linear 0.60 --sim-tau-angular 0.35 \
    --collision-distance 0.75 --near-miss-distance 0.95 \
    --scenario-neighbor-speed 0.34 --teacher-conflict-gate 1 \
    --base-ros-domain-id 340 --seed 77 \
    2>&1 | tee "$CKPT/${TAG}_dart_collect.log"
fi

python3 - "$BASE" "$DART" "$MERGED" <<'EOF'
import sys, numpy as np
a=np.load(sys.argv[1],allow_pickle=True); b=np.load(sys.argv[2],allow_pickle=True)
o=b['observations']; w=b['actions'][:,-1]; e=b['engaged'].astype(bool); f=~e
he=np.degrees(np.arctan2(o[:,5],o[:,6])); cte=o[:,11]
print(f'[dart] samples={len(w)} free={f.mean():.2f} free |HE| p50/p90={np.median(np.abs(he[f])):.1f}/{np.percentile(np.abs(he[f]),90):.1f}deg  |CTE| p50/p90={np.median(np.abs(cte[f])):.2f}/{np.percentile(np.abs(cte[f]),90):.2f}m')
for lo,hi in [(-40,-20),(-20,-8),(-8,8),(8,20),(20,40)]:
    m=f&(he>=lo)&(he<hi)
    if m.sum()>20: print(f'[dart]   free HE[{lo:+d},{hi:+d}) n={m.sum():5d} omega med {np.median(w[m]):+.3f}')
cov=(f&(np.abs(he)>8)&(np.abs(he)<40)).sum()
if cov<3000: raise SystemExit(f'[dart] FAIL: only {cov} free samples with 8<|HE|<40deg')
out={k: np.concatenate([a[k], b[k]]) for k in ('observations','actions','scenario_names','engaged')}
for k in ('action_mode','action_speed_scale_min','max_neighbors'): out[k]=a[k]
np.savez_compressed(sys.argv[3], **out)
print(f'[merge] base={len(a["actions"])} dart={len(b["actions"])} merged={len(out["actions"])} engaged={out["engaged"].mean():.3f}')
EOF

python3 -m usv_rl.bc_pretrain_attention_actor \
  --warmstart "$ARCH_BASE" --dataset "$MERGED" --output "$BC_INIT" \
  --epochs 200 --lr 1e-3 --batch-size 512 --device cpu \
  --input-dropout-slots 8,10 --input-dropout-prob 0.6 \
  2>&1 | tee "$CKPT/${TAG}_bc.log"

python3 -m usv_rl.evaluate_mappo_policy \
  --model "$BC_INIT" --episodes 16 --steps-per-episode 150 \
  --scenario solo_navigation --scenario waypoint_turn \
  --scenario two_usv_head_on --scenario two_usv_crossing --scenario cluster_escape \
  --scenario five_usv_dense_head_on --scenario five_usv_dense_crossing \
  --scenario five_usv_dense_overtaking --seed 4342 \
  --output-json "$GATE_JSON" 2>&1 | tee "$CKPT/${TAG}_gate.log"
python3 - "$GATE_JSON" <<'EOF'
import json, sys
d=json.load(open(sys.argv[1]))
print('[gate]', {k: round(float(d[k]), 3) for k in ['collision_rate','success_rate','timeout_rate','worst_pairwise_min_separation','mean_heading_error','mean_cross_track_error']})
for s,v in d['scenario_summaries'].items():
    print(f"  {s:28s} coll={v['collision_rate']:.1f} succ={v['success_rate']:.1f} minsep={v['worst_episode_min_separation']:.2f} HE={v['mean_heading_error']:.2f} CTE={v['mean_cross_track_error']:.2f}")
print('[gate] fresh647 ref (6 scen): coll 0.167 HE 0.615 CTE 0.589; SITL open-water dead band HE~+10deg omega~0')
EOF
echo "[${TAG}] BC init ready: $BC_INIT (RL not launched)"
