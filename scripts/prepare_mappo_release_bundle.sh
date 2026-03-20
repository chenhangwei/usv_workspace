#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/mnt/workspace/usv_workspace"
DEFAULT_RELEASE_ROOT="/mnt/data/releases/usv_rl"

usage() {
  cat <<'EOF'
usage: prepare_mappo_release_bundle.sh <checkpoint.pt> [--output-dir DIR] [--summary-json FILE] [--asset-name NAME] [--role ROLE] [--notes TEXT]

Creates a small release staging directory for a selected MAPPO milestone artifact.
The output is intended for manual upload to GitHub Release or object storage.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -lt 1 ]]; then
  usage
  exit 0
fi

CHECKPOINT_PATH="$1"
shift || true

OUTPUT_DIR=""
ROLE="candidate"
ASSET_NAME=""
NOTES=""
SUMMARY_FILES=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --summary-json)
      SUMMARY_FILES+=("$2")
      shift 2
      ;;
    --asset-name)
      ASSET_NAME="$2"
      shift 2
      ;;
    --role)
      ROLE="$2"
      shift 2
      ;;
    --notes)
      NOTES="$2"
      shift 2
      ;;
    *)
      echo "error: unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "${CHECKPOINT_PATH}" ]]; then
  echo "error: checkpoint not found: ${CHECKPOINT_PATH}" >&2
  exit 2
fi

for summary_file in "${SUMMARY_FILES[@]}"; do
  if [[ ! -f "${summary_file}" ]]; then
    echo "error: summary file not found: ${summary_file}" >&2
    exit 2
  fi
done

CHECKPOINT_ABS="$(readlink -f "${CHECKPOINT_PATH}")"
CHECKPOINT_BASENAME="$(basename "${CHECKPOINT_PATH}")"
CHECKPOINT_STEM="$(basename "${CHECKPOINT_PATH}" .pt)"
ASSET_NAME="${ASSET_NAME:-${CHECKPOINT_BASENAME}}"

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${DEFAULT_RELEASE_ROOT}/${CHECKPOINT_STEM}_release_$(date +%Y%m%d_%H%M%S)"
fi

mkdir -p "${OUTPUT_DIR}"

COPIED_CHECKPOINT_PATH="${OUTPUT_DIR}/${ASSET_NAME}"
cp -f "${CHECKPOINT_PATH}" "${COPIED_CHECKPOINT_PATH}"

COPIED_SUMMARIES=()
for summary_file in "${SUMMARY_FILES[@]}"; do
  copied_summary_path="${OUTPUT_DIR}/$(basename "${summary_file}")"
  cp -f "${summary_file}" "${copied_summary_path}"
  COPIED_SUMMARIES+=("${copied_summary_path}")
done

CHECKPOINT_SHA256="$(sha256sum "${COPIED_CHECKPOINT_PATH}" | awk '{print $1}')"
CHECKPOINT_SIZE_BYTES="$(stat -c '%s' "${COPIED_CHECKPOINT_PATH}")"
GIT_BRANCH="$(git -C "${WORKSPACE_DIR}" branch --show-current 2>/dev/null || true)"
GIT_COMMIT="$(git -C "${WORKSPACE_DIR}" rev-parse HEAD 2>/dev/null || true)"
CREATED_AT="$(date -Iseconds)"
MANIFEST_PATH="${OUTPUT_DIR}/manifest.json"
RELEASE_NOTES_PATH="${OUTPUT_DIR}/RELEASE_NOTES.md"
SHA256_PATH="${OUTPUT_DIR}/SHA256SUMS"

MANIFEST_SUMMARY_LIST_PATH="${OUTPUT_DIR}/.manifest_summary_files.txt"
printf '%s\n' "${COPIED_SUMMARIES[@]}" > "${MANIFEST_SUMMARY_LIST_PATH}"

CHECKPOINT_ABS="${CHECKPOINT_ABS}" \
ASSET_NAME="${ASSET_NAME}" \
ROLE="${ROLE}" \
GIT_BRANCH="${GIT_BRANCH}" \
GIT_COMMIT="${GIT_COMMIT}" \
CREATED_AT="${CREATED_AT}" \
CHECKPOINT_SHA256="${CHECKPOINT_SHA256}" \
CHECKPOINT_SIZE_BYTES="${CHECKPOINT_SIZE_BYTES}" \
NOTES="${NOTES}" \
MANIFEST_SUMMARY_LIST_PATH="${MANIFEST_SUMMARY_LIST_PATH}" \
python3 - <<'PY' > "${MANIFEST_PATH}"
import json
import os
from pathlib import Path

summary_files = []
summary_list_path = Path(os.environ['MANIFEST_SUMMARY_LIST_PATH'])
if summary_list_path.exists():
  summary_files = [line for line in summary_list_path.read_text(encoding='utf-8').splitlines() if line]

manifest = {
  'created_at': os.environ['CREATED_AT'],
  'checkpoint_source_path': os.environ['CHECKPOINT_ABS'],
  'asset_name': os.environ['ASSET_NAME'],
  'role': os.environ['ROLE'],
  'git_branch': os.environ['GIT_BRANCH'],
  'git_commit': os.environ['GIT_COMMIT'],
  'checkpoint_sha256': os.environ['CHECKPOINT_SHA256'],
  'checkpoint_size_bytes': int(os.environ['CHECKPOINT_SIZE_BYTES']),
  'summary_files': summary_files,
  'notes': os.environ['NOTES'],
}
print(json.dumps(manifest, ensure_ascii=False, indent=2))
PY

rm -f "${MANIFEST_SUMMARY_LIST_PATH}"

cat > "${RELEASE_NOTES_PATH}" <<EOF
# MAPPO Milestone Release Notes

- role: ${ROLE}
- asset_name: ${ASSET_NAME}
- source_checkpoint: ${CHECKPOINT_ABS}
- git_branch: ${GIT_BRANCH:-unknown}
- git_commit: ${GIT_COMMIT:-unknown}
- checkpoint_sha256: ${CHECKPOINT_SHA256}

## Suggested Summary

填写这次 checkpoint 为什么值得发布，例如：

- 离线评估结论
- 在线 repeat 结论
- 是否作为当前主候选 / 备份 / 基线

## Optional Notes

${NOTES:-无}

## Included Files

- $(basename "${COPIED_CHECKPOINT_PATH}")
$(for copied_summary_path in "${COPIED_SUMMARIES[@]}"; do printf -- '- %s\n' "$(basename "${copied_summary_path}")"; done)
- manifest.json
- SHA256SUMS
EOF

(
  cd "${OUTPUT_DIR}"
  sha256sum "$(basename "${COPIED_CHECKPOINT_PATH}")" $(for copied_summary_path in "${COPIED_SUMMARIES[@]}"; do printf '%s ' "$(basename "${copied_summary_path}")"; done) manifest.json RELEASE_NOTES.md > "${SHA256_PATH}"
)

echo "output_dir=${OUTPUT_DIR}"
echo "release_asset=${COPIED_CHECKPOINT_PATH}"
echo "manifest=${MANIFEST_PATH}"
echo "release_notes=${RELEASE_NOTES_PATH}"
echo "sha256sums=${SHA256_PATH}"
