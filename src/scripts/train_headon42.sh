#!/bin/bash
# headon42: Ultimate head_on separation compression
#
# BUG FIX: _apply_resume_configuration now respects CLI overrides (sys.argv check)
#
# WHY HEAD ON GAP IS 2X OTHER SCENARIOS:
# In head_on, *both* USVs yield to their starboard side simultaneously. 
# If they both steer to achieve a 0.5m lateral offset, total optical clearance is 1.0m. 
# This "doubling effect" means they clear each other twice as fast and by twice the distance 
# compared to crossing/overtaking (where one vessel holds course and one yields).
#
# h42 strategy — HYPER-COMPRESSED head_on parameters (isolated from crossing/overtaking):
#   - near_miss_distance: 2.0 (global baseline for crossing/overtaking)
#   - head_on_near_miss_distance: 0.95 -> 0.82 (almost down to absolute collision=0.75)
#   - head_on_target_starboard_offset: 0.25 -> 0.15 (both yielding 0.15m = 0.3m clearance)
#   - head_on_guidance_distance: 3.0 -> 2.5 (trigger turn even later)
#
# Resume from stable h41-665K checkpoint, train to 850K

set -eo pipefail
cd "$(dirname "$0")/../.."
source install/setup.bash
set -u

OUTPUT=/mnt/data/checkpoints/usv_rl/headon42.pt

python3 -m usv_rl.train_mappo_policy \
  --output "$OUTPUT" \
  --resume-from /mnt/data/checkpoints/usv_rl/headon41_checkpoints/headon41_step_0665280.pt \
  --num-agents 3 \
  --total-timesteps 850000 \
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
  --head-on-near-miss-distance 0.82 \
  --head-on-guidance-distance 2.5 \
  --head-on-target-starboard-offset 0.15 \
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
