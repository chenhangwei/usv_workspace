#!/bin/bash
# fresh604: rerun fresh602 after fixing speed_scale action bounds metadata.
#
# Why this exists:
# - fresh600/601/602 used action_mode=speed_scale, but train_mappo_policy
#   exported/trained with full-mode bounds for the actor squash:
#     wrong:   action_low/high = [0.0, 0.34] for the first dimension
#     correct: action_low/high = [0.30, 1.0] for throttle scale
# - This meant even a "full throttle" speed_scale policy was trained/evaluated
#   as if scale max were 0.34, causing the familiar 0.116 m/s crawl.
# - The metadata/bounds bug is now fixed in train_mappo_policy.py. This wrapper
#   reruns the exact fresh602 3-USV configuration under the corrected code and
#   writes a separate artifact.

set -eo pipefail
cd "$(dirname "$0")/.."

OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh604_speedfix_bounds_3usv.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh604_checkpoints}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-286}" \
bash scripts/train_fresh602_speedfix_3usv.sh
