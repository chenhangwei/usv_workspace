#!/bin/bash
# fresh467: narrow post-recapture resume from fresh464.
#
# fresh466 proved the constant resume target is wired, but it disrupted the
# route-recapture geometry.  This variant starts from the fresh464 checkpoint,
# uses a smaller PPO step and stronger anchor, and only applies clear-ahead on
# usv_02 when CTE is already inside a narrow post-recapture band.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh464_goal_heading_action_probe_from_fresh438.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh467_recap_resume_narrow_from_fresh464.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh467_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-215}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 8.0e-8 \
  --learning-rate-end 3.0e-8 \
  --clip-range 0.00006 \
  --update-epochs 3 \
  --random-cte-recovery-weight 0.80 \
  --random-cte-recovery-weight-end 1.20 \
  --random-recovery-pretrain-epochs 2 \
  --random-recovery-pretrain-learning-rate 1.2e-6 \
  --random-recovery-pretrain-max-grad-norm 0.25 \
  --random-recovery-pretrain-cte-scale 0.35 \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.14 \
  --random-clear-ahead-weight-end 0.24 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.04 \
  --random-clear-ahead-max-route-progress 0.18 \
  --random-clear-ahead-max-abs-cte 1.65 \
  --random-clear-ahead-min-neighbor-separation 5.5 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.13 \
  --random-clear-ahead-min-speed 0.08 \
  --random-clear-ahead-max-omega 0.05 \
  --random-clear-ahead-omega-reference 1.10 \
  --random-clear-ahead-omega-weight 0.18 \
  --random-recovery-pretrain-clear-scale 1.40 \
  --policy-anchor-weight 6500.0 \
  --policy-anchor-weight-end 8500.0 \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"