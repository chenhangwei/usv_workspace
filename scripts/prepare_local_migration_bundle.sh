#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/mnt/workspace/usv_workspace"
DATA_DIR="/mnt/data/checkpoints/usv_rl"
DEFAULT_BUNDLE_ROOT="/mnt/data/migration"

infer_run_name() {
  local latest_file
  latest_file="$(ls -1t /tmp/current_*_run_name 2>/dev/null | head -n 1 || true)"
  if [[ -n "${latest_file}" && -f "${latest_file}" ]]; then
    cat "${latest_file}"
  fi
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "usage: prepare_local_migration_bundle.sh [run_name] [bundle_root]"
  echo "default bundle_root: ${DEFAULT_BUNDLE_ROOT}"
  exit 0
fi

RUN_NAME="${1:-$(infer_run_name)}"
BUNDLE_ROOT="${2:-${DEFAULT_BUNDLE_ROOT}}"

if [[ -z "${RUN_NAME}" ]]; then
  echo "error: run_name is required or must be inferable from /tmp/current_*_run_name" >&2
  exit 2
fi

OUTPUT_PATH="${DATA_DIR}/${RUN_NAME}.pt"
LOG_PATH="${DATA_DIR}/${RUN_NAME}.log"
CHECKPOINT_DIR="${DATA_DIR}/${RUN_NAME}_checkpoints"
BASELINE_JSON="/tmp/recover_7680_online_benchmark.json"

if [[ ! -f "${OUTPUT_PATH}" ]]; then
  echo "error: final output checkpoint not found: ${OUTPUT_PATH}" >&2
  exit 3
fi

mkdir -p "${BUNDLE_ROOT}"
BUNDLE_DIR="${BUNDLE_ROOT}/${RUN_NAME}_local_migration_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${BUNDLE_DIR}/staging/checkpoints/usv_rl" "${BUNDLE_DIR}/staging/baselines"

SOURCE_ARCHIVE="${BUNDLE_DIR}/usv_workspace_source.tgz"
ARTIFACT_ARCHIVE="${BUNDLE_DIR}/${RUN_NAME}_artifacts.tgz"

tar \
  --exclude='./build' \
  --exclude='./install' \
  --exclude='./log' \
  --exclude='./.git' \
  --exclude='__pycache__' \
  -czf "${SOURCE_ARCHIVE}" \
  -C "${WORKSPACE_DIR}" .

cp -f "${OUTPUT_PATH}" "${BUNDLE_DIR}/staging/checkpoints/usv_rl/"
if [[ -f "${LOG_PATH}" ]]; then
  cp -f "${LOG_PATH}" "${BUNDLE_DIR}/staging/checkpoints/usv_rl/"
fi
if [[ -d "${CHECKPOINT_DIR}" ]]; then
  cp -a "${CHECKPOINT_DIR}" "${BUNDLE_DIR}/staging/checkpoints/usv_rl/"
fi
if [[ -f "${BASELINE_JSON}" ]]; then
  cp -f "${BASELINE_JSON}" "${BUNDLE_DIR}/staging/baselines/"
fi

tar -czf "${ARTIFACT_ARCHIVE}" -C "${BUNDLE_DIR}/staging" .
rm -rf "${BUNDLE_DIR}/staging"

LATEST_CHECKPOINT="$(find "${CHECKPOINT_DIR}" -maxdepth 1 -type f -name "${RUN_NAME}_step_*.pt" 2>/dev/null | sort | tail -n 1 || true)"
COMPLETED_TIMESTEPS="$(python3 - "${OUTPUT_PATH}" <<'PY'
import sys
try:
    import torch
    payload = torch.load(sys.argv[1], map_location='cpu')
    if isinstance(payload, dict):
        print(int(payload.get('completed_timesteps', 0)))
    else:
        print(0)
except Exception:
    print(0)
PY
)"

cat > "${BUNDLE_DIR}/LOCAL_MIGRATION_README.md" <<EOF
# Local Migration Bundle

- run_name: ${RUN_NAME}
- completed_timesteps: ${COMPLETED_TIMESTEPS}
- final_model: ${OUTPUT_PATH}
- latest_checkpoint: ${LATEST_CHECKPOINT:-none}

## 1. Copy these two archives to the local machine

- $(basename "${SOURCE_ARCHIVE}")
- $(basename "${ARTIFACT_ARCHIVE}")

## 2. Recreate the server directory layout locally

\
sudo mkdir -p /mnt/workspace/usv_workspace\

\
sudo mkdir -p /mnt/data/checkpoints/usv_rl /mnt/data/evals/usv_rl /mnt/data/logs/usv_rl /mnt/data/datasets/usv_rl\

\
sudo chown -R \$USER:\$USER /mnt/workspace /mnt/data\

## 3. Extract the source archive

\
mkdir -p /mnt/workspace/usv_workspace\

\
tar -xzf $(basename "${SOURCE_ARCHIVE}") -C /mnt/workspace/usv_workspace\

## 4. Extract the artifact archive to a temp folder, then copy the run artifacts

\
mkdir -p ~/usv_local_migration_artifacts\

\
tar -xzf $(basename "${ARTIFACT_ARCHIVE}") -C ~/usv_local_migration_artifacts\

\
cp -a ~/usv_local_migration_artifacts/checkpoints/usv_rl/. /mnt/data/checkpoints/usv_rl/\

## 5. Build locally

\
source /opt/ros/jazzy/setup.bash\

\
cd /mnt/workspace/usv_workspace\

\
rosdep install --from-paths src --ignore-src -r -y\

\
colcon build --packages-up-to usv_rl usv_sim --symlink-install\

\
source install/setup.bash\

## 6. Verify the local environment

\
ros2 run usv_rl preflight_check --output-json /mnt/data/evals/usv_rl/preflight_local.json\

## 7. Continue MAPPO training locally

The new total timesteps must be greater than ${COMPLETED_TIMESTEPS}.

\
cd /mnt/workspace/usv_workspace\

\
./scripts/resume_mappo_training.sh /mnt/data/checkpoints/usv_rl/${RUN_NAME}.pt <NEW_TOTAL_TIMESTEPS_GT_${COMPLETED_TIMESTEPS}>\

EOF

cat > "${BUNDLE_DIR}/resume_local_training.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail

if [[ "\${1:-}" == "-h" || "\${1:-}" == "--help" || \$# -lt 1 ]]; then
  echo "usage: resume_local_training.sh <new_total_timesteps> [output_path.pt] [extra train args...]"
  exit 0
fi

cd /mnt/workspace/usv_workspace
./scripts/resume_mappo_training.sh /mnt/data/checkpoints/usv_rl/${RUN_NAME}.pt "\$@"
EOF
chmod +x "${BUNDLE_DIR}/resume_local_training.sh"

sha256sum "${SOURCE_ARCHIVE}" "${ARTIFACT_ARCHIVE}" > "${BUNDLE_DIR}/SHA256SUMS"
printf '%s\n' "${BUNDLE_DIR}" > "/tmp/${RUN_NAME}_local_migration_bundle_path"

echo "bundle_dir=${BUNDLE_DIR}"
echo "source_archive=${SOURCE_ARCHIVE}"
echo "artifact_archive=${ARTIFACT_ARCHIVE}"