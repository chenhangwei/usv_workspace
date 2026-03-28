#!/bin/bash
# headon40: Targeted head_on optimization preserving crossing/overtaking safety
#
# BUG FIX: _apply_resume_configuration now respects CLI overrides (sys.argv check)
#
# h39-620K (actual h33 params, 15ep):
#   head_on:    0% coll, worst=2.025m  ← TARGET: approach 0.75m
#   crossing:   0% coll, worst=0.916m  ✓
#   overtaking: 0% coll, worst=0.862m  ✓
#
# ROOT CAUSE of head_on 2.0m:
#   1. near_miss_distance=6.0 → penalty at 2m is -15.25/step
#   2. head_on_target_starboard_offset=1.0 → agents target 1m offset each → 2m total
#   3. All guidance/conflict distances=6.0 → early reaction completes full offset
#
# LESSON from h40 attempt 1 (conflict_distance=4.0):
#   Crossing regressed to 100% collision! Reducing conflict_distance removed
#   the crossing safety net. Must keep conflict_distance=6.0.
#
# h40 v3 strategy — SELECTIVE changes (head_on-specific only):
#   - near_miss_distance: 6.0 → 2.0 (global baseline for crossing/overtaking)
#   - head_on_near_miss_distance: 1.1 (only for two_usv_head_on)
#   - anticipation_distance: 6.0 → 4.0 (allows head_on_guidance to use 4.0m,
#     but conflict_risk still uses max(4,6)=6.0, crossing_overtaking uses max(4,6)=6.0)
#   - head_on_guidance_distance: 6.0 → 4.0 (head_on turns start at 4m, not 6m)
#   - head_on_target_starboard_offset: 1.0 → 0.40 (reduce symmetric split offset)
#   - KEEP conflict_distance=6.0 (crossing/overtaking conflict_risk and guidance at 6m)
#   - KEEP near_miss_weight=5.0, collision_penalty=-150 (strong near-crash avoidance)
#   - team_reward_weight: 0.3 → 0.10 (reduce team separation incentive)
#
# Distance cascade analysis:
#   conflict_risk:      max(anticipation=4, conflict=6, coll=0.75) = 6.0 ✓ safe
#   head_on_guidance:   max(head_on=4, anticipation=4, coll=0.75)  = 4.0 ← delayed!
#   cross/overt_guide:  max(anticipation=4, conflict=6, coll=0.75) = 6.0 ✓ safe
#
# Expected: head_on drops from 2.0m to ~1.0-1.5m, crossing/overtaking stay safe
# Resume from h39-620K, train to 800K (180K new steps)

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon41.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon39_checkpoints/headon39_step_0620352.pt \
  --num-agents 3 \
  --total-timesteps 800000 \
  --rollout-steps 192 \
  --update-epochs 3 \
  --minibatch-size 128 \
  --learning-rate 5e-5 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.15 \
  --entropy-coef 0.03 \
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
  --near-miss-distance 2.0 \
  --head-on-near-miss-distance 0.95 \
  --near-miss-weight 5.0 \
  --near-miss-exponent 2.0 \
  --collision-distance 0.75 \
  --collision-penalty -150.0 \
  --team-reward-weight 0.10 \
  --progress-weight 3.0 \
  --goal-bonus 30.0 \
  --conflict-risk-weight 3.0 \
  --conflict-brake-weight 1.2 \
  --conflict-progress-scale 0.6 \
  --conflict-resolution-reward-weight 1.5 \
  --conflict-escalation-penalty-weight 2.0 \
  --unsafe-close-speed-penalty-weight 3.0 \
  --conflict-distance 6.0 \
  --anticipation-distance 3.0 \
  --head-on-guidance-distance 3.0 \
  --head-on-target-starboard-offset 0.25 \
  --head-on-corridor-reward-weight 1.4 \
  --head-on-centerline-penalty-weight 1.6 \
  --head-on-close-penalty-weight 1.2 \
  --head-on-no-turn-penalty-weight 1.0 \
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
