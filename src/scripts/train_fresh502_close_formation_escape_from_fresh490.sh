#!/bin/bash
# fresh502: "veteran-driver" Stage-B retrain on top of fresh490 (raw_cmd-fixed obs).
#
# Combines three teachers in parallel rather than serially:
#   * signed raw-CTE recovery (inherited from fresh490 base script)
#   * all-agent clear-ahead (from fresh501, moderate weight, open-water lane)
#   * NEW close-formation-escape (decisive forward + lateral escape when
#     nearest neighbor sits in [1.40, 2.40] m and own vx <= 0.22 m/s)
#
# The close-formation-escape teacher targets the exact equilibrium that
# fresh500/fresh501 could not break: stalled_far with a close neighbor where
# the clear-ahead gate is inactive (clear-ahead requires neighbor >= 4.80 m).
#
# Training shape is materially longer than fresh500/fresh501 (default 20000
# env-steps vs 2400) and adds a small action-smoothness bump to encourage
# smoother throttle/yaw behavior.  Runtime low-speed floors remain disabled.
#
# Acceptance gate (Stage B): core5 mean progress >= 0.35 with collision=0 and
# worst min_sep >= 1.10 m via scripts/eval_fresh490_pair0203_recommended.sh
# (pair0103 add-on optional).

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh490_all_agent_raw_cte_signed_return_from_fresh486.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh502_close_formation_escape_from_fresh490.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh502_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-20000}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-226}" \
exec bash scripts/train_fresh490_all_agent_raw_cte_signed_return_from_fresh486.sh \
  --learning-rate 5.0e-8 \
  --learning-rate-end 2.0e-8 \
  --clip-range 0.000030 \
  --update-epochs 4 \
  --action-smoothness-weight 0.02 \
  --random-clear-ahead-agent-index 0 \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-agent-index 2 \
  --random-clear-ahead-weight 0.60 \
  --random-clear-ahead-weight-end 1.00 \
  --random-clear-ahead-min-route-progress 0.000 \
  --random-clear-ahead-max-route-progress 0.72 \
  --random-clear-ahead-max-abs-cte 1.60 \
  --random-clear-ahead-min-neighbor-separation 4.80 \
  --random-clear-ahead-target-speed 0.180 \
  --random-clear-ahead-min-speed 0.080 \
  --random-clear-ahead-speed-cte-slowdown 0.75 \
  --random-clear-ahead-speed-cte-start 0.70 \
  --random-clear-ahead-speed-cte-full 1.60 \
  --random-close-formation-escape-weight 0.80 \
  --random-close-formation-escape-weight-end 1.40 \
  --random-close-formation-escape-min-sep 1.40 \
  --random-close-formation-escape-max-sep 2.40 \
  --random-close-formation-escape-max-self-speed 0.22 \
  --random-close-formation-escape-goal-tolerance 1.0 \
  --random-close-formation-escape-max-distance 20.0 \
  --random-close-formation-escape-target-speed 0.40 \
  --random-close-formation-escape-min-speed 0.20 \
  --random-close-formation-escape-max-omega 0.45 \
  --random-close-formation-escape-omega-weight 1.6 \
  --random-close-formation-escape-require-closing \
  --random-recovery-pretrain-clear-scale 1.80 \
  --policy-anchor-weight 6000.0 \
  --policy-anchor-weight-end 9000.0 \
  "$@"
