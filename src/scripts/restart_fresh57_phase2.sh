#!/bin/bash
# fresh57 Phase 2 恢复脚本 — 从 100K checkpoint 断点续训到 200K
# 使用 --resume-from 恢复训练元数据（步数/LR schedule）
set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh57_checkpoints"
RESUME_CKPT="/mnt/data/checkpoints/usv_rl/fresh57_checkpoints/fresh57_step_0100034.pt"
PHASE2_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh57.pt"

if [[ ! -f "$RESUME_CKPT" ]]; then
  echo "ERROR: Resume checkpoint not found: $RESUME_CKPT"
  exit 1
fi

# 公共参数
COMMON_ARGS=(
  --rollout-steps 192
  --update-epochs 3
  --minibatch-size 128
  --gamma 0.99
  --gae-lambda 0.95
  --clip-range 0.08
  --max-grad-norm 0.5
  --device auto
  --hidden-size 256 --hidden-size 256
  --max-agents 5
  --squash-actions
  --min-forward-speed 0.08
  --angular-delta-limit 0.40
  --neighbor-attention
  --attention-embed-dim 32
  --attention-num-heads 1
  --normalize-observations
  --per-scenario-advantage-norm
  --scenario-balanced-loss
  --domain-randomization
  --dr-position-noise-std 0.10
  --dr-heading-noise-std 0.02
  --dr-velocity-noise-ratio 0.03
  --dr-current-speed-max 0.04
  --dr-velocity-exec-noise 0.05
  --cruise-speed 0.36
  --max-angular-velocity 0.40
  --heading-omega-deadband 0.05
  --heading-omega-reference 0.75
  --angular-authority-power 1.3
  --angular-accel-limit 1.7
  --angular-decel-limit 2.3
  --conflict-turn-relief 0.50
  --collision-distance 0.75
  --collision-penalty -350.0
  --team-reward-weight 0.10
  --team-completion-bonus 30.0
  --goal-bonus 36.0
  --desired-conflict-speed 0.12
  --conflict-distance 8.0
  --anticipation-distance 8.0
  --head-on-guidance-distance 8.0
  --head-on-target-starboard-offset 0.90
  --heading-error-weight 2.0
  --colregs-port-turn-penalty-weight 3.5
  --no-progress-timeout 24.0
  --checkpoint-interval 2000
  --num-sampler-workers 2
  --base-ros-domain-id 150
  --stall-penalty -20.0
  --time-penalty 0.05
  --sim-tau-linear 2.0
  --sim-tau-angular 0.8
  --dr-tau-linear-low 1.5
  --dr-tau-linear-high 2.5
  --dr-tau-angular-low 0.6
  --dr-tau-angular-high 1.0
  --speed-scale-distance 3.0
  --speed-scale-min 0.35
)

echo "========== Phase 2 RESUME: Multi-scenario Avoidance (100K→200K, 3 USVs) =========="
echo "Resuming from: $RESUME_CKPT"
python3 -m usv_rl.train_mappo_policy \
  --output "$PHASE2_OUTPUT" \
  --resume-from "$RESUME_CKPT" \
  --checkpoint-dir "$CKPT_DIR" \
  --num-agents 3 \
  --total-timesteps 200000 \
  --learning-rate 2.5e-5 \
  --learning-rate-end 8e-6 \
  --entropy-coef 0.004 \
  --entropy-coef-end 0.002 \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.15 \
  --scenario-goal-position-std 0.3 \
  --near-miss-distance 5.0 \
  --head-on-near-miss-distance 0.0 \
  --near-miss-weight 14.0 \
  --near-miss-exponent 2.0 \
  --team-progress-weight 1.35 \
  --team-goal-proximity-weight 0.40 \
  --coordination-reward-weight 0.36 \
  --separation-recovery-weight 6.0 \
  --progress-weight 2.6 \
  --conflict-risk-weight 7.0 \
  --conflict-brake-weight 3.5 \
  --conflict-progress-scale 0.50 \
  --conflict-resolution-reward-weight 3.5 \
  --conflict-escalation-penalty-weight 5.5 \
  --unsafe-close-speed-penalty-weight 9.0 \
  --conflict-overspeed-penalty-weight 7.5 \
  --head-on-corridor-reward-weight 3.2 \
  --head-on-centerline-penalty-weight 3.2 \
  --head-on-turn-reward-weight 1.8 \
  --head-on-forward-reward-weight 1.5 \
  --head-on-speed-drop-penalty-weight 1.0 \
  --head-on-close-penalty-weight 5.0 \
  --head-on-no-turn-penalty-weight 3.0 \
  --head-on-phase-gate-strength 0.70 \
  --action-smoothness-weight 5.5 \
  --angular-accel-penalty-weight 4.0 \
  --straight-line-omega-penalty-weight 7.0 \
  --saturated-omega-flip-penalty-weight 9.0 \
  --forward-speed-change-penalty-weight 5.5 \
  --omega-flip-saturation-threshold 0.35 \
  --straight-line-omega-conflict-floor 0.25 \
  --pure-cruise-reward-weight 2.3 \
  --pure-idle-penalty-weight 1.5 \
  --pure-turn-penalty-weight 3.0 \
  --pure-spin-penalty-weight 8.0 \
  --path-deviation-penalty-weight 0.95 \
  --deadlock-penalty-weight 5.5 \
  --stop-go-penalty-weight 3.0 \
  --goal-proximity-reward-weight 2.4 \
  --goal-proximity-relief-distance 2.5 \
  --goal-proximity-heading-relief 0.60 \
  --goal-proximity-smoothness-relief 0.75 \
  --goal-proximity-conflict-relief 0.35 \
  --crossing-starboard-turn-reward-weight 4.8 \
  --crossing-forward-reward-weight 1.6 \
  --overtaking-starboard-turn-reward-weight 2.0 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.2 \
  --overtaking-centerline-penalty-weight 2.0 \
  --overtaking-close-penalty-weight 3.3 \
  --proximity-gradient-penalty-weight 6.0 \
  --proximity-gradient-distance 4.0 \
  --speed-distance-coupling-penalty-weight 7.0 \
  --speed-distance-coupling-threshold 3.0 \
  --heading-convergence-reward-weight 3.0 \
  --heading-convergence-threshold-deg 12.0 \
  "${COMMON_ARGS[@]}"

echo "========== Phase 2 complete =========="
echo "Final model: $PHASE2_OUTPUT"
echo "Next: bash scripts/fresh57_batch_eval.sh"
