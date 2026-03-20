#!/usr/bin/env bash
set -euo pipefail

retry_command() {
  local attempts="$1"
  shift

  local attempt=1
  while true; do
    if "$@"; then
      return 0
    fi

    if (( attempt >= attempts )); then
      return 1
    fi

    echo "Command failed, retrying (${attempt}/${attempts})..." >&2
    attempt=$((attempt + 1))
    sleep 5
  done
}

if [[ "$(id -u)" -ne 0 ]]; then
  echo "This script must be run as root." >&2
  exit 1
fi

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
else
  echo "/etc/os-release not found." >&2
  exit 1
fi

ubuntu_codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
rosdep_source_base="${ROSDEP_SOURCE_BASE:-https://mirrors.tuna.tsinghua.edu.cn/rosdistro}"
rosdistro_index_url="${ROSDISTRO_INDEX_URL:-https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml}"
local_rosdep_cache_dir="${LOCAL_ROSDEP_CACHE_DIR:-/tmp/pai_dsw_rosdep}"
if [[ -z "${ubuntu_codename}" ]]; then
  case "${VERSION_ID:-}" in
    24.04)
      ubuntu_codename="noble"
      ;;
    22.04)
      ubuntu_codename="jammy"
      ;;
    *)
      echo "Unable to infer Ubuntu codename from VERSION_ID=${VERSION_ID:-unknown}." >&2
      exit 1
      ;;
  esac
fi

if [[ "${ID:-}" != "ubuntu" ]]; then
  echo "This script currently supports Ubuntu images only. Detected ID=${ID:-unknown}." >&2
  exit 1
fi

echo "[1/7] Installing base system packages"
apt-get update
apt-get install -y --no-install-recommends \
  ca-certificates \
  curl \
  gnupg \
  lsb-release \
  software-properties-common \
  locales \
  python3-pip

echo "[2/7] Enabling UTF-8 locale"
locale-gen en_US en_US.UTF-8
update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

echo "[3/7] Enabling Ubuntu universe repository"
add-apt-repository -y universe

echo "[4/7] Configuring ROS 2 apt repository for ${ubuntu_codename}"
install -d -m 0755 /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${ubuntu_codename} main" \
  > /etc/apt/sources.list.d/ros2.list

echo "[5/7] Installing ROS 2 Jazzy runtime and build tools"
apt-get update
apt-get install -y --no-install-recommends \
  ros-jazzy-ros-base \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete \
  python3-serial

echo "[6/7] Initializing rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
install -d "${local_rosdep_cache_dir}/rosdep" "${local_rosdep_cache_dir}/jazzy"
curl -fsSL "${rosdep_source_base}/rosdep/base.yaml" -o "${local_rosdep_cache_dir}/rosdep/base.yaml"
curl -fsSL "${rosdep_source_base}/rosdep/python.yaml" -o "${local_rosdep_cache_dir}/rosdep/python.yaml"
curl -fsSL "${rosdep_source_base}/jazzy/distribution.yaml" -o "${local_rosdep_cache_dir}/jazzy/distribution.yaml"
cat > "${local_rosdep_cache_dir}/index-v4.yaml" <<'EOF'
%YAML 1.1
---
distributions:
  jazzy:
    distribution: [jazzy/distribution.yaml]
    distribution_status: active
    distribution_type: ros2
    python_version: 3
type: index
version: 4
EOF
cat > /etc/ros/rosdep/sources.list.d/20-default.list <<EOF
# generic
yaml file://${local_rosdep_cache_dir}/rosdep/base.yaml
yaml file://${local_rosdep_cache_dir}/rosdep/python.yaml
EOF
if ! retry_command 3 env ROSDISTRO_INDEX_URL="file://${local_rosdep_cache_dir}/index-v4.yaml" rosdep update; then
  cat >&2 <<'EOF'
rosdep update failed after 3 attempts.
This usually means the current DSW instance failed to refresh the local rosdep cache from the configured mirror.
ROS 2 core packages are already installed, so you can continue and rerun:
  bash scripts/pai_dsw_bootstrap_ros_jazzy.sh
  rosdep install --from-paths src --ignore-src -r -y
once network access is stable.
EOF
fi

echo "[7/7] Installing optional PPO Python dependencies"
python3 -m pip install --break-system-packages --upgrade pip
python3 -m pip install --break-system-packages casadi gymnasium stable-baselines3

cat <<'EOF'

Bootstrap completed.

Next steps:
  source /opt/ros/jazzy/setup.bash
  cd /mnt/workspace/usv_workspace
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --packages-up-to usv_rl usv_sim --symlink-install
  source install/setup.bash
  python3 src/usv_rl/usv_rl/preflight_check.py --output-json /mnt/data/evals/usv_rl/preflight_after_bootstrap.json || true

EOF