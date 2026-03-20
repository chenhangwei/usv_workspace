#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/mnt/workspace/usv_workspace"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "usage: start_mappo_a10_parallel_safe_train.sh [run_name] [extra train args...]"
  echo "env: NUM_SAMPLER_WORKERS(default=2) BASE_ROS_DOMAIN_ID(default=180)"
  exit 0
fi

RUN_NAME="${1:-mappo_dense_a10_parallel_safe_$(date +%Y%m%d_%H%M%S)}"

if [[ $# -gt 0 ]]; then
  shift
fi

cd "${WORKSPACE_DIR}"

NUM_SAMPLER_WORKERS="${NUM_SAMPLER_WORKERS:-2}"
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-180}"

exec ./scripts/start_mappo_a10_parallel_train.sh \
  "${RUN_NAME}" \
  --num-sampler-workers "${NUM_SAMPLER_WORKERS}" \
  --base-ros-domain-id "${BASE_ROS_DOMAIN_ID}" \
  --collision-penalty -160 \
  --near-miss-weight 18 \
  --conflict-distance 6.5 \
  --anticipation-distance 7.0 \
  --conflict-risk-weight 4.5 \
  --conflict-brake-weight 1.6 \
  --stop-go-penalty-weight 2.2 \
  --head-on-guidance-distance 6.5 \
  --head-on-centerline-penalty-weight 3.6 \
  --crossing-starboard-turn-reward-weight 0.9 \
  --crossing-forward-reward-weight 0.35 \
  --overtaking-starboard-turn-reward-weight 0.7 \
  --overtaking-forward-reward-weight 0.3 \
  --colregs-port-turn-penalty-weight 0.8 \
  --goal-proximity-reward-weight 0.45 \
  --team-reward-weight 0.65 \
  --team-progress-weight 0.75 \
  --coordination-reward-weight 0.12 \
  --deadlock-penalty-weight 5.5 \
  "$@"