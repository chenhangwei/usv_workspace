#!/bin/bash
# 快速启动 MAVROS 连接飞控（优化版）
# 使用方法：./quick_mavros.sh [usv_id] [serial_port]
# 示例：./quick_mavros.sh 01 /dev/ttyACM0

USV_ID=${1:-01}
SERIAL_PORT=${2:-/dev/ttyACM0}
BAUDRATE=921600

echo "========================================"
echo "快速启动 MAVROS（优化版）"
echo "========================================"
echo "USV ID: usv_${USV_ID}"
echo "串口: ${SERIAL_PORT}"
echo "波特率: ${BAUDRATE}"
echo "========================================"

# Source ROS 2 环境
source ~/usv_workspace/install/setup.bash

# 启动 MAVROS（仅加载必需插件）
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/usv_${USV_ID} \
  -p fcu_url:=serial://${SERIAL_PORT}:${BAUDRATE} \
  -p system_id:=${USV_ID#0} \
  -p target_system_id:=${USV_ID#0} \
  -p plugin_allowlist:="['sys_status','sys_time','command','local_position','setpoint_raw','global_position','gps_status','battery']"

echo ""
echo "MAVROS 已停止"
