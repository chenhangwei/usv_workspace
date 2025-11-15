#!/bin/bash
# 导航目标点调试工具

echo "🔍 导航目标点调试工具"
echo "===================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 检查参数
USV_NS=${1:-usv_01}

echo -e "${YELLOW}目标USV: ${USV_NS}${NC}"
echo ""

# 1. 检查节点状态
echo -e "${GREEN}1️⃣ 检查关键节点...${NC}"
echo "-------------------"
ros2 node list 2>/dev/null | grep -E "navigate_to_point|coord_transform|mavros" | while read node; do
    echo "  ✓ $node"
done
echo ""

# 2. 检查话题
echo -e "${GREEN}2️⃣ 检查关键话题...${NC}"
echo "-------------------"
ros2 topic list 2>/dev/null | grep -E "${USV_NS}/(set_usv_target|setpoint_raw|navigate_to_point)" | while read topic; do
    echo "  ✓ $topic"
done
echo ""

# 3. 检查坐标转换节点配置
echo -e "${GREEN}3️⃣ 检查坐标转换配置...${NC}"
echo "-------------------"
echo "  GPS原点配置:"
ros2 param get /${USV_NS}/coord_transform_node gps_origin_lat 2>/dev/null | head -1
ros2 param get /${USV_NS}/coord_transform_node gps_origin_lon 2>/dev/null | head -1
ros2 param get /${USV_NS}/coord_transform_node enable_coord_transform 2>/dev/null | head -1
ros2 param get /${USV_NS}/coord_transform_node use_global_position_target 2>/dev/null | head -1
echo ""

# 4. 检查控制节点配置
echo -e "${GREEN}4️⃣ 检查控制节点配置...${NC}"
echo "-------------------"
ros2 param get /${USV_NS}/usv_control_node enable_local_control 2>/dev/null | head -1
echo ""

# 5. 发送测试目标点
echo -e "${GREEN}5️⃣ 发送测试目标点...${NC}"
echo "-------------------"
TEST_X=60.0
TEST_Y=35.0
TEST_Z=0.0
echo "  测试坐标: X=${TEST_X}, Y=${TEST_Y}, Z=${TEST_Z}"
ros2 topic pub --once /${USV_NS}/set_usv_target_position \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: ${TEST_X}, y: ${TEST_Y}, z: ${TEST_Z}}}}" 2>/dev/null
echo "  ✓ 目标点已发送"
echo ""

# 6. 监听日志
echo -e "${GREEN}6️⃣ 监听USV日志（10秒）...${NC}"
echo "-------------------"
echo "  等待日志输出..."
echo ""

timeout 10 ros2 topic echo /${USV_NS}/rosout 2>/dev/null | grep -E "Action Server|坐标转换节点|📥|📤|📨" --color=always -A 5 &
PID=$!

# 等待5秒后检查话题
sleep 5

echo ""
echo -e "${GREEN}7️⃣ 检查输出话题...${NC}"
echo "-------------------"
echo "  查看 setpoint_raw/global (最新1条):"
timeout 2 ros2 topic echo /${USV_NS}/setpoint_raw/global --once 2>/dev/null | grep -E "latitude|longitude|altitude" || echo "  ⚠️ 无数据"
echo ""

# 等待日志进程结束
wait $PID 2>/dev/null

echo ""
echo -e "${GREEN}✅ 调试完成！${NC}"
echo ""
echo "💡 提示："
echo "  - 如需查看更多日志: ros2 topic echo /${USV_NS}/rosout | grep '坐标转换' -A 5"
echo "  - 如需持续监听: ros2 topic echo /${USV_NS}/setpoint_raw/global"
echo "  - 如需监听MAVLink: nc -ul 192.168.10.1 -p 14550 | hexdump -C"
