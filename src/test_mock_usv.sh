#!/bin/bash
# 虚拟USV完整测试脚本
# 模拟从地面站发送目标点到USV接收的完整流程

echo "🎮 虚拟USV完整测试"
echo "===================="
echo ""

# 颜色
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# 参数
USV_NS=${1:-usv_01}
TEST_X=${2:-60.0}
TEST_Y=${3:-35.0}
TEST_Z=${4:-0.0}

echo -e "${YELLOW}配置信息:${NC}"
echo "  USV命名空间: $USV_NS"
echo "  测试目标点: X=$TEST_X, Y=$TEST_Y, Z=$TEST_Z"
echo ""

# 检查是否已经在运行
if ros2 node list 2>/dev/null | grep -q "mock_usv_data"; then
    echo -e "${YELLOW}⚠️  虚拟数据节点已在运行${NC}"
    echo ""
else
    echo -e "${GREEN}1️⃣ 启动虚拟USV数据节点...${NC}"
    echo "-------------------"
    
    # 启动虚拟数据节点（后台）
    ros2 run usv_comm mock_usv_data \
        --ros-args \
        -p namespace:=$USV_NS \
        -p initial_x:=0.0 \
        -p initial_y:=0.0 \
        -p move_speed:=2.0 \
        > /tmp/mock_usv_${USV_NS}.log 2>&1 &
    
    MOCK_PID=$!
    echo "  ✓ 虚拟数据节点已启动 (PID: $MOCK_PID)"
    echo "  ✓ 日志: /tmp/mock_usv_${USV_NS}.log"
    sleep 2
    echo ""
fi

# 检查必需节点
echo -e "${GREEN}2️⃣ 检查必需节点...${NC}"
echo "-------------------"

REQUIRED_NODES=(
    "navigate_to_point_server"
    "coord_transform_node"
)

ALL_NODES_OK=true
for node in "${REQUIRED_NODES[@]}"; do
    if ros2 node list 2>/dev/null | grep -q "$node"; then
        echo "  ✓ $node"
    else
        echo -e "  ${RED}✗ $node (未运行)${NC}"
        ALL_NODES_OK=false
    fi
done

if [ "$ALL_NODES_OK" = false ]; then
    echo ""
    echo -e "${RED}❌ 部分必需节点未运行！${NC}"
    echo ""
    echo "请先启动USV节点："
    echo "  ros2 launch usv_bringup usv_launch.py namespace:=$USV_NS"
    exit 1
fi
echo ""

# 检查虚拟数据发布
echo -e "${GREEN}3️⃣ 检查虚拟数据发布...${NC}"
echo "-------------------"

echo "  检查 GPS 数据..."
timeout 2 ros2 topic echo /${USV_NS}/global_position/global --once >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✓ GPS数据正常"
else
    echo -e "  ${YELLOW}⚠️  GPS数据未发布${NC}"
fi

echo "  检查本地位置数据..."
timeout 2 ros2 topic echo /${USV_NS}/local_position/pose --once >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✓ 本地位置数据正常"
else
    echo -e "  ${YELLOW}⚠️  本地位置数据未发布${NC}"
fi

echo "  检查 MAVROS 状态..."
timeout 2 ros2 topic echo /${USV_NS}/state --once >/dev/null 2>&1
if [ $? -eq 0 ]; then
    STATE=$(timeout 2 ros2 topic echo /${USV_NS}/state --once 2>/dev/null | grep "mode:" | awk '{print $2}' | tr -d "'")
    echo "  ✓ MAVROS状态正常 (模式: $STATE)"
else
    echo -e "  ${YELLOW}⚠️  MAVROS状态未发布${NC}"
fi
echo ""

# 查看当前位置
echo -e "${GREEN}4️⃣ 当前虚拟USV状态...${NC}"
echo "-------------------"

CURRENT_POSE=$(timeout 2 ros2 topic echo /${USV_NS}/local_position/pose --once 2>/dev/null)
if [ -n "$CURRENT_POSE" ]; then
    CURRENT_X=$(echo "$CURRENT_POSE" | grep "x:" | head -1 | awk '{print $2}')
    CURRENT_Y=$(echo "$CURRENT_POSE" | grep "y:" | head -1 | awk '{print $2}')
    CURRENT_Z=$(echo "$CURRENT_POSE" | grep "z:" | head -1 | awk '{print $2}')
    echo "  当前位置: X=$CURRENT_X, Y=$CURRENT_Y, Z=$CURRENT_Z"
else
    echo -e "  ${YELLOW}⚠️  无法获取当前位置${NC}"
fi
echo ""

# 发送导航目标点
echo -e "${GREEN}5️⃣ 发送导航目标点...${NC}"
echo "-------------------"
echo "  目标: X=$TEST_X, Y=$TEST_Y, Z=$TEST_Z"

# 使用 Action 发送
ros2 action send_goal /${USV_NS}/navigate_to_point \
    common_interfaces/action/NavigateToPoint \
    "{goal: {pose: {position: {x: $TEST_X, y: $TEST_Y, z: $TEST_Z}}}, timeout: 60.0}" \
    --feedback &

ACTION_PID=$!
echo "  ✓ 导航命令已发送 (Action PID: $ACTION_PID)"
sleep 1
echo ""

# 监听关键日志
echo -e "${GREEN}6️⃣ 监听调试日志（15秒）...${NC}"
echo "-------------------"
echo ""

# 启动日志监听（后台）
timeout 15 ros2 topic echo /${USV_NS}/rosout 2>/dev/null | \
    grep -E "Action Server|坐标转换节点|虚拟USV|📥|📤|📨|🎯" --color=always -A 5 &

LOG_PID=$!

# 等待一会儿
sleep 3

# 实时显示位置变化
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}实时位置监控 (更新中...)${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

for i in {1..10}; do
    CURRENT_POSE=$(timeout 1 ros2 topic echo /${USV_NS}/local_position/pose --once 2>/dev/null)
    if [ -n "$CURRENT_POSE" ]; then
        CURRENT_X=$(echo "$CURRENT_POSE" | grep "x:" | head -1 | awk '{print $2}')
        CURRENT_Y=$(echo "$CURRENT_POSE" | grep "y:" | head -1 | awk '{print $2}')
        
        # 计算距离
        DISTANCE=$(awk -v cx="$CURRENT_X" -v cy="$CURRENT_Y" -v tx="$TEST_X" -v ty="$TEST_Y" \
            'BEGIN{printf "%.2f", sqrt((tx-cx)^2 + (ty-cy)^2)}')
        
        echo -e "  [$i] 位置: ${GREEN}X=$CURRENT_X${NC}, ${GREEN}Y=$CURRENT_Y${NC} | 距目标: ${YELLOW}${DISTANCE}m${NC}"
    fi
    sleep 1
done

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# 等待日志监听结束
wait $LOG_PID 2>/dev/null

echo ""
echo -e "${GREEN}7️⃣ 检查输出话题...${NC}"
echo "-------------------"

echo "  查看全局GPS目标点:"
timeout 2 ros2 topic echo /${USV_NS}/setpoint_raw/global --once 2>/dev/null | \
    grep -E "latitude|longitude|altitude" | head -3

echo ""
echo -e "${GREEN}✅ 测试完成！${NC}"
echo ""
echo "💡 提示:"
echo "  - 虚拟USV会以2m/s速度向目标点移动"
echo "  - 查看完整日志: tail -f /tmp/mock_usv_${USV_NS}.log"
echo "  - 停止虚拟数据: pkill -f mock_usv_data"
echo "  - 查看位置: ros2 topic echo /${USV_NS}/local_position/pose"
echo "  - 查看GPS: ros2 topic echo /${USV_NS}/global_position/global"
echo ""
