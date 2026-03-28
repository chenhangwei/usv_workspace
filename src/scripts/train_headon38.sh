#!/bin/bash
# headon38: Fast adaptation from safe baseline, strong collision deterrent
#
# Key findings from h34-h37:
#   h34/h35 (lr=2e-5, clip=0.08): worst_sep ~2.0-2.25m — lr too slow to change policy
#   h35v2 (lr=1e-4, clip=0.20): head_on 1.05m✓, overtaking 0.82m✓, crossing CRASH
#   h36 (lr=5e-5, clip=0.15, col=-80, from h33): worst_sep=3.15m — more conservative!
#   h37 (lr=3e-5, from h35v2): crossing still 100% crash — h35v2 damage irreversible
#
# Analysis:
#   - h35v2 proved lr=1e-4 + clip=0.20 CAN reduce avoidance distance
#   - But collision_penalty=-30 was too weak → crossing learned unsafe behavior
#   - h36 from h33 with col=-80 went MORE conservative → high collision penalty
#     combined with inherited conservative policy makes it even more cautious
#
# h38: lr=1e-4, clip=0.20 (proven to work) + collision_penalty=-80 (strong deterrent)
# From h33-405K directly (not h35v2 which has corrupted crossing behavior)
# The key difference from h36: use the FAST lr (1e-4) not the slow one (5e-5)
# The higher lr should overcome the conservative policy inheritance quickly,
# while the strong collision penalty should prevent crossing-like crashes.
#
# Reward design:
#   near_miss_distance=1.0, W=1.0 → at 0.75m: ~0.275/step total repulsion
#   collision_penalty=-80 → 290x worse than boundary near-miss
#   team_reward_weight=0.10
#   conflict_risk_weight=0.8
#
# Train to 650K (245K new steps) — should be enough with fast lr

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon38.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon33_checkpoints/headon33_step_0405504.pt \
  --num-agents 3 \
  --total-timesteps 650000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 1e-4 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.20 \
  --entropy-coef 0.04 \
  --max-grad-norm 0.5 \
  --device auto \
  --hidden-size 256 --hidden-size 256 \
  --max-agents 5 \
  --scenario two_usv_head_on --scenario three_usv_crossing --scenario three_usv_overtaking \
  \
  --squash-actions \
  --min-forward-speed 0.12 \
  --normalize-observations \
  \
  --domain-randomization \
  --dr-position-noise-std 0.10 \
  --dr-heading-noise-std 0.02 \
  --dr-velocity-noise-ratio 0.03 \
  --dr-current-speed-max 0.04 \
  --dr-velocity-exec-noise 0.05 \
  \
  --scenario-spawn-position-std 0.3 \
  --scenario-spawn-heading-std 0.15 \
  --scenario-goal-position-std 0.3 \
  \
  --near-miss-distance 1.0 \
  --near-miss-weight 1.0 \
  --near-miss-exponent 2.0 \
  --collision-distance 0.75 \
  --collision-penalty -80.0 \
  --team-reward-weight 0.10 \
  --progress-weight 3.0 \
  --goal-bonus 30.0 \
  --conflict-risk-weight 0.8 \
  --conflict-brake-weight 1.2 \
  --conflict-progress-scale 0.6 \
  --conflict-resolution-reward-weight 1.5 \
  --conflict-escalation-penalty-weight 2.0 \
  --unsafe-close-speed-penalty-weight 0.5 \
  --conflict-distance 6.0 \
  --anticipation-distance 6.0 \
  --head-on-guidance-distance 6.0 \
  --heading-error-weight 1.5 \
  --action-smoothness-weight 1.5 \
  --angular-accel-penalty-weight 1.0 \
  --pure-cruise-reward-weight 2.0 \
  --pure-idle-penalty-weight 3.0 \
  --pure-turn-penalty-weight 0.5 \
  --pure-spin-penalty-weight 3.0 \
  --path-deviation-penalty-weight 1.0 \
  --stall-penalty -20.0 \
  --time-penalty 0.05 \
  --deadlock-penalty-weight 4.0 \
  --stop-go-penalty-weight 1.5 \
  --goal-proximity-reward-weight 1.5 \
  \
  --crossing-starboard-turn-reward-weight 2.0 \
  --crossing-forward-reward-weight 1.2 \
  --overtaking-starboard-turn-reward-weight 1.5 \
  --overtaking-forward-reward-weight 1.0 \
  --overtaking-corridor-reward-weight 2.0 \
  --overtaking-centerline-penalty-weight 2.0 \
  --overtaking-close-penalty-weight 2.5 \
  --colregs-port-turn-penalty-weight 1.5 \
  \
  --no-progress-timeout 20.0 \
  \
  --checkpoint-interval 5000 \
  --num-sampler-workers 2 \
  --base-ros-domain-id 110 \
  --log-interval-updates 1
