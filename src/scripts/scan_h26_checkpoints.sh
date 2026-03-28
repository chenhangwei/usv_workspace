#!/bin/bash
# Batch evaluate headon26 checkpoints
source /home/chenhangwei/usv_workspace/install/setup.bash
export ROS_DOMAIN_ID=112
cd /home/chenhangwei/usv_workspace/src

CKPT_DIR="/mnt/data/checkpoints/usv_rl/headon26_checkpoints"
OUT_DIR="/tmp/h26_scan"
mkdir -p "$OUT_DIR"

# Key checkpoints to evaluate: 10K, 50K, 150K, 200K, 250K, 300K, 350K, 400K, 450K, 500K
STEPS=(0010368 0050112 0150336 0200448 0250560 0300096 0350208 0400320 0450432 0500001)

for STEP in "${STEPS[@]}"; do
    CKPT="${CKPT_DIR}/headon26_step_${STEP}.pt"
    OUTFILE="${OUT_DIR}/h26_step_${STEP}.json"
    
    if [ -f "$OUTFILE" ]; then
        echo "SKIP: ${STEP} (already evaluated)"
        continue
    fi
    
    if [ ! -f "$CKPT" ]; then
        echo "MISSING: ${STEP}"
        continue
    fi
    
    echo "EVAL: step ${STEP} ..."
    python3 -m usv_rl.evaluate_mappo_policy \
        --model "$CKPT" \
        --episodes 3 \
        --steps-per-episode 180 \
        --device cpu \
        --scenario two_usv_head_on \
        --output-json "$OUTFILE" 2>&1 | grep -E "collision_rate|success_rate|mean_team_goal_progress|Saved"
    
    if [ -f "$OUTFILE" ]; then
        echo "  -> saved to $OUTFILE"
    else
        echo "  -> FAILED"
    fi
done

echo ""
echo "=== SCAN RESULTS ==="
for STEP in "${STEPS[@]}"; do
    OUTFILE="${OUT_DIR}/h26_step_${STEP}.json"
    if [ -f "$OUTFILE" ]; then
        python3 -c "
import json
with open('$OUTFILE') as f:
    d = json.load(f)
col = d['collision_rate']
suc = d['success_rate']
prog = d['mean_team_goal_progress_ratio']
sep = d['worst_pairwise_min_separation']
print(f'  step ${STEP}: col={col:.0%} suc={suc:.0%} prog={prog:.1%} worst_sep={sep:.3f}')
"
    fi
done
