#!/bin/bash
# 飞控版本检查脚本
# 用法: ./check_fcu_version.sh [usv_id]

USV_ID=${1:-usv_01}

echo "======================================"
echo "飞控版本检查工具"
echo "======================================"
echo "检查 USV: $USV_ID"
echo ""

# 方法 1: 查看 MAVROS 连接状态
echo "=== 方法 1: MAVROS 连接状态 ==="
timeout 3 ros2 topic echo /${USV_ID}/mavros/state --once 2>/dev/null | grep -E "connected|mode" || echo "❌ 无法获取状态"
echo ""

# 方法 2: 查看心跳信息（包含飞控类型）
echo "=== 方法 2: 心跳信息 ==="
echo "正在监听心跳消息（5秒）..."
timeout 5 ros2 topic echo /${USV_ID}/mavros/heartbeat --once 2>/dev/null || echo "❌ 无法获取心跳"
echo ""

# 方法 3: 查看系统状态
echo "=== 方法 3: 系统状态 ==="
timeout 3 ros2 topic echo /${USV_ID}/mavros/sys_status --once 2>/dev/null | head -20 || echo "❌ 无法获取系统状态"
echo ""

# 方法 4: 从日志查找
echo "=== 方法 4: 从启动日志查找 ArduPilot 版本 ==="
ros2 topic echo /rosout --once 2>/dev/null | grep -i "ardupilot\|FCU:" | tail -5 || echo "❌ 日志中未找到版本信息"
echo ""

# 方法 5: 使用 MAVProxy（如果已安装）
echo "=== 方法 5: 使用 mavproxy（如果可用）==="
if command -v mavproxy.py &> /dev/null; then
    echo "可以使用 MAVProxy 连接飞控查询版本："
    echo "mavproxy.py --master=/dev/ttyACM0 --baudrate=921600"
    echo "然后输入命令: version"
else
    echo "❌ MAVProxy 未安装"
fi
echo ""

echo "======================================"
echo "建议："
echo "1. 如果上述方法都失败，使用 Mission Planner 或 QGC 连接飞控"
echo "2. Mission Planner: 连接后查看 HUD 右上角版本信息"
echo "3. QGC: 连接后查看 Settings → Vehicle Info"
echo "======================================"
