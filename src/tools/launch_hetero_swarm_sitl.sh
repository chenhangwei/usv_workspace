#!/usr/bin/env bash
# ==============================================================================
# 异构立体集群 SITL 一键联调脚本
# 同时在 Gazebo 中启动：
#   - usv_01: 28m 仿生飞艇 (bionic_airship, 4022) @ (0, 0, 15)
#   - usv_02:  5m 仿生海豚 (bionic_dolphin, 4023) @ (12, 0, 6)
#   - usv_03: 1.8m 飞行球  (flying_sphere,  4024) @ (-12, 0, 2)
# 并启动 MicroXRCEAgent + 3 套 ROS2 机载控制/状态节点 + 地面站 gs_launch.py
#
# 用法:
#   ./launch_hetero_swarm_sitl.sh            # 带 Gazebo GUI + 地面站 GUI
#   ./launch_hetero_swarm_sitl.sh --headless # 无头 Gazebo + 地面站 GUI
#   ./launch_hetero_swarm_sitl.sh --no-gs    # 仅启动仿真与机载 ROS2 节点
# ==============================================================================
set -e

PX4_DIR="${PX4_DIR:-/home/chenhangwei/PX4-Autopilot}"
WS_DIR="${WS_DIR:-/home/chenhangwei/usv_workspace}"
GZ_BIN="/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"
LOG_DIR="/tmp/hetero_swarm_sitl"
mkdir -p "$LOG_DIR"

HEADLESS_FLAG=0
START_GS=1
for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS_FLAG=1 ;;
    --no-gs) START_GS=0 ;;
  esac
done

export ROS_DOMAIN_ID=99
source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"

PIDS=()
cleanup() {
  echo ""
  echo ">>> 正在清理异构集群仿真进程..."
  for pid in "${PIDS[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -f "bin/px4" 2>/dev/null || true
  pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
  echo ">>> 清理完成。"
}
trap cleanup EXIT INT TERM

# 0. 清理历史残留
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "bin/px4" 2>/dev/null || true
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
sleep 1

# 1. 启动 MicroXRCEAgent (UDP 8888)
XRCE_AGENT_BIN=$(command -v MicroXRCEAgent 2>/dev/null || echo "/home/chenhangwei/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent")
if [[ -x "$XRCE_AGENT_BIN" ]]; then
  echo ">>> [1/4] 启动 MicroXRCEAgent ($XRCE_AGENT_BIN udp4 -p 8888)..."
  "$XRCE_AGENT_BIN" udp4 -p 8888 > "${LOG_DIR}/xrce_agent.log" 2>&1 &
  PIDS+=($!)
else
  echo "!!! 警告: 未找到 MicroXRCEAgent，请确认已安装后重试。"
fi

# 2. 依次启动 3 个异构 PX4 SITL 实例（由第 1 个实例自动拉起 Gazebo，独立工作目录防参数冲突）
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
ROMFS_DIR="${PX4_DIR}/build/px4_sitl_default/etc"
if [[ "$HEADLESS_FLAG" -eq 1 ]]; then
  export HEADLESS=1
fi
if [[ -f "${PX4_DIR}/build/px4_sitl_default/rootfs/gz_env.sh" ]]; then
  source "${PX4_DIR}/build/px4_sitl_default/rootfs/gz_env.sh"
else
  export PX4_GZ_WORLDS="${PX4_DIR}/Tools/simulation/gz/worlds"
  export PX4_GZ_MODELS="${PX4_DIR}/Tools/simulation/gz/models"
  export GZ_SIM_RESOURCE_PATH="${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}:${GZ_SIM_RESOURCE_PATH}"
fi

launch_px4_instance() {
  local idx="$1"
  local ns="$2"
  local autostart="$3"
  local model="$4"
  local pose="$5"
  local inst_dir="${LOG_DIR}/px4_${ns}"
  rm -rf "$inst_dir"
  mkdir -p "$inst_dir"
  (
    cd "$inst_dir"
    PX4_SYS_AUTOSTART="$autostart" \
    PX4_SIM_MODEL="$model" \
    PX4_GZ_MODEL_POSE="$pose" \
    PX4_UXRCE_DDS_NS="$ns" \
    UXRCE_DDS_DOM_ID=99 \
    "$PX4_BIN" -i "$idx" -d "$ROMFS_DIR" > "${LOG_DIR}/px4_${ns}.log" 2>&1
  ) &
  PIDS+=($!)
}

echo ">>> [2/4] 启动 3 架异构 PX4 SITL 实例（飞艇 + 海豚 + 飞行球）..."
launch_px4_instance 1 "usv_01" 4022 "bionic_airship" "0,0,15,0,0,0"
sleep 10
launch_px4_instance 2 "usv_02" 4023 "bionic_dolphin" "12,0,6,0,0,0"
sleep 5
launch_px4_instance 3 "usv_03" 4024 "flying_sphere"  "-12,0,2,0,0,0"
sleep 6

# 4. 启动 3 套机载 ROS 2 控制与状态桥接节点
launch_ros_onboard() {
  local ns="$1"
  local vtype="$2"
  local rxy="$3"
  local rz="$4"
  ros2 run usv_control usv_control_node --ros-args \
    -r __ns:="/${ns}" \
    -p platform_mode:=3d \
    -p vehicle_type:="${vtype}" \
    -p reach_xy_threshold:="${rxy}" \
    -p reach_z_threshold:="${rz}" \
    -r /fmu/out/vehicle_status_v1:="/${ns}/fmu/out/vehicle_status_v1" \
    -r /fmu/out/vehicle_local_position:="/${ns}/fmu/out/vehicle_local_position" \
    -r /fmu/in/trajectory_setpoint:="/${ns}/fmu/in/trajectory_setpoint" \
    -r /fmu/in/trajectory_setpoint6dof:="/${ns}/fmu/in/trajectory_setpoint6dof" \
    -r /fmu/in/vehicle_command:="/${ns}/fmu/in/vehicle_command" \
    -r /fmu/in/offboard_control_mode:="/${ns}/fmu/in/offboard_control_mode" \
    > "${LOG_DIR}/ctrl_${ns}.log" 2>&1 &
  PIDS+=($!)

  ros2 run usv_comm usv_status_node --ros-args \
    -r __ns:="/${ns}" \
    -p usv_id:="${ns}" \
    -r /fmu/out/vehicle_status_v1:="/${ns}/fmu/out/vehicle_status_v1" \
    -r /fmu/out/vehicle_local_position:="/${ns}/fmu/out/vehicle_local_position" \
    -r /fmu/out/battery_status:="/${ns}/fmu/out/battery_status" \
    -r /fmu/out/vehicle_attitude:="/${ns}/fmu/out/vehicle_attitude" \
    > "${LOG_DIR}/status_${ns}.log" 2>&1 &
  PIDS+=($!)
}

echo ">>> [4/5] 启动机载 ROS 2 控制与状态节点..."
launch_ros_onboard "usv_01" "bionic_airship" 4.0 1.5
launch_ros_onboard "usv_02" "bionic_dolphin" 1.5 0.8
launch_ros_onboard "usv_03" "flying_sphere"  0.8 0.5
sleep 2

# 5. 启动地面站
if [[ "$START_GS" -eq 1 ]]; then
  echo ">>> [5/5] 启动地面站 gs_launch.py (domain_id:=99, use_zenoh:=false)..."
  ros2 launch gs_bringup gs_launch.py use_zenoh:=false domain_id:=99
else
  echo ">>> [5/5] 已就绪（--no-gs 模式），日志目录: ${LOG_DIR}。按 Ctrl+C 退出。"
  wait
fi
