#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/mnt/workspace/usv_workspace"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "usage: start_mappo_a10_parallel_headon_stabilize_train.sh [run_name] [extra train args...]"
  echo "env: NUM_SAMPLER_WORKERS(default=2) BASE_ROS_DOMAIN_ID(default=180)"
  exit 0
fi

RUN_NAME="${1:-mappo_dense_a10_parallel_headon_stabilize_$(date +%Y%m%d_%H%M%S)}"

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
  --conflict-risk-weight 4.2 \
  --conflict-brake-weight 1.3 \
  --desired-conflict-speed 0.24 \
  --stop-go-penalty-weight 2.4 \
  --head-on-guidance-distance 7.5 \
  --head-on-target-starboard-offset 1.45 \
  --head-on-corridor-reward-weight 4.6 \
  --head-on-centerline-penalty-weight 5.2 \
  --head-on-turn-reward-weight 2.2 \
  --head-on-forward-reward-weight 0.25 \
  --head-on-speed-drop-penalty-weight 0.35 \
  --crossing-starboard-turn-reward-weight 1.25 \
  --crossing-forward-reward-weight 0.80 \
  --overtaking-starboard-turn-reward-weight 0.85 \
  --overtaking-forward-reward-weight 0.45 \
  --colregs-port-turn-penalty-weight 1.15 \
  --goal-proximity-reward-weight 0.40 \
  --team-reward-weight 0.60 \
  --team-progress-weight 0.60 \
  --coordination-reward-weight 0.12 \
  --deadlock-penalty-weight 6.0 \
  "$@"