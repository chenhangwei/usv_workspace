#!/bin/bash
# headon36: Balanced tight avoidance — narrow near-miss band + moderate safety
#
# Lessons learned:
#   h33 (W=10,exp=3,d=6,col=-150,lr=2e-5,clip=0.08): 0% coll, worst_sep=1.2-2.2m (too far)
#   h34 (W=2,exp=2,d=1.5,col=-50,lr=2e-5,clip=0.08): 0% coll, worst_sep=2.0m (penalty reduction didn't help with slow lr)
#   h35 (W=1,exp=2,d=1.0,col=-30,lr=2e-5,clip=0.08): 0% coll, worst_sep=2.26m (same — lr too slow)
#   h35v2 (W=1,exp=2,d=1.0,col=-30,lr=1e-4,clip=0.20): head_on 1.05m✓, overtaking 0.82m✓, crossing 100% COLLISION!
#
# Root cause analysis:
#   - lr=1e-4 + clip=0.20 destabilizes crossing (complex 3-agent geometry needs more safety margin)
#   - collision_penalty=-30 is too weak to prevent crossing collisions with fast policy updates
#   - near_miss_distance=1.0 is good (narrow band), but collision penalty must be strong enough
#
# h36 strategy: "tight but safe"
#   - near_miss_distance=1.0 (narrow 0.25m band — only penalize 0.75-1.0m)
#   - near_miss_weight=1.0, near_miss_exponent=2.0 (gentle pressure in the band)
#   - collision_penalty=-80 (NOT -30: strong deterrent to prevent crossing crashes)
#   - team_reward_weight=0.10 (reduced from 0.30)
#   - conflict_risk_weight=0.8, unsafe_close_speed=0.5 (reduced)
#   - lr=5e-5 (moderate: 2.5x original, not 5x)
#   - clip-range=0.15 (moderate: between 0.08 and 0.20)
#   - entropy=0.035 (slight increase for exploration)
#   - max-grad-norm=0.3 (moderate)
#
# Penalty at 0.75m: near_miss=0.25 + team=0.025 = 0.275/step
# Collision: -80 → collision is 290x worse than near-miss boundary
# This should allow tight passes while maintaining strong collision avoidance
#
# Resume from h33-405K (clean restart, avoid inheriting h35/h35v2 instability)
# Train to 650K (245K new steps for thorough adaptation)

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon36.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon33_checkpoints/headon33_step_0405504.pt \
  --num-agents 3 \
  --total-timesteps 650000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 5e-5 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.15 \
  --entropy-coef 0.035 \
  --max-grad-norm 0.3 \
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
