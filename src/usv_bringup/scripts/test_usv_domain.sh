#!/bin/bash
# USV Domain 测试脚本
# 用于在 USV 上测试 Domain Bridge 通信

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}USV Domain Bridge 测试${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查是否在 USV 上
if [ "$(hostname)" != "gauss01" ] && [ "$(hostname)" != "gauss02" ] && [ "$(hostname)" != "gauss03" ]; then
    echo -e "${YELLOW}⚠ 警告: 此脚本应在 USV 设备 (gauss01/02/03) 上运行${NC}"
    echo -e "当前主机: $(hostname)"
    echo ""
fi

# 检查 ROS_DOMAIN_ID
echo -e "${YELLOW}[1/3]${NC} 检查 ROS_DOMAIN_ID..."
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo -e "${RED}   ✗ ROS_DOMAIN_ID 未设置${NC}"
    echo ""
    echo -e "${BLUE}请根据 USV 设置对应的 Domain ID:${NC}"
    echo -e "  usv_01 (gauss01) -> ${GREEN}export ROS_DOMAIN_ID=11${NC}"
    echo -e "  usv_02 (gauss02) -> ${GREEN}export ROS_DOMAIN_ID=12${NC}"
    echo -e "  usv_03 (gauss03) -> ${GREEN}export ROS_DOMAIN_ID=13${NC}"
    echo ""
    echo -e "${YELLOW}建议添加到 ~/.bashrc:${NC}"
    case "$(hostname)" in
        gauss01)
            echo -e "  echo 'export ROS_DOMAIN_ID=11' >> ~/.bashrc"
            ;;
        gauss02)
            echo -e "  echo 'export ROS_DOMAIN_ID=12' >> ~/.bashrc"
            ;;
        gauss03)
            echo -e "  echo 'export ROS_DOMAIN_ID=13' >> ~/.bashrc"
            ;;
        *)
            echo -e "  echo 'export ROS_DOMAIN_ID=<domain_id>' >> ~/.bashrc"
            ;;
    esac
    exit 1
else
    echo -e "${GREEN}   ✓ ROS_DOMAIN_ID = $ROS_DOMAIN_ID${NC}"
    
    # 验证 Domain ID 是否正确
    case "$(hostname)" in
        gauss01)
            if [ "$ROS_DOMAIN_ID" != "11" ]; then
                echo -e "${RED}   ✗ 错误: gauss01 应使用 Domain 11，当前是 $ROS_DOMAIN_ID${NC}"
                exit 1
            fi
            ;;
        gauss02)
            if [ "$ROS_DOMAIN_ID" != "12" ]; then
                echo -e "${RED}   ✗ 错误: gauss02 应使用 Domain 12，当前是 $ROS_DOMAIN_ID${NC}"
                exit 1
            fi
            ;;
        gauss03)
            if [ "$ROS_DOMAIN_ID" != "13" ]; then
                echo -e "${RED}   ✗ 错误: gauss03 应使用 Domain 13，当前是 $ROS_DOMAIN_ID${NC}"
                exit 1
            fi
            ;;
    esac
fi
echo ""

# 检查 USV 节点
echo -e "${YELLOW}[2/3]${NC} 检查 USV 节点..."
NODE_COUNT=$(ros2 node list 2>/dev/null | grep -c "usv_01" || true)
if [ "$NODE_COUNT" -gt 0 ]; then
    echo -e "${GREEN}   ✓ 发现 $NODE_COUNT 个 USV 节点${NC}"
    ros2 node list 2>/dev/null | grep "usv_01" | head -5 | sed 's/^/      /'
else
    echo -e "${RED}   ✗ 未发现 USV 节点${NC}"
    echo -e "${YELLOW}   启动 USV: ros2 launch usv_bringup usv_launch.py${NC}"
fi
echo ""

# 检查关键话题
echo -e "${YELLOW}[3/3]${NC} 检查 USV 话题..."
TOPICS=(
    "/usv_01/usv_state"
    "/usv_01/mavros/state"
    "/usv_01/mavros/local_position/pose"
    "/usv_01/set_usv_target_position"
)

TOPIC_OK=0
TOPIC_FAIL=0

for topic in "${TOPICS[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
        echo -e "   ${GREEN}✓${NC} $topic"
        ((TOPIC_OK++))
    else
        echo -e "   ${RED}✗${NC} $topic"
        ((TOPIC_FAIL++))
    fi
done
echo ""

# 总结
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}测试总结${NC}"
echo -e "${BLUE}========================================${NC}"

if [ "$TOPIC_OK" -gt 0 ]; then
    echo -e "${GREEN}✓ USV 节点运行正常${NC}"
    echo -e "  - ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo -e "  - 可见话题: $TOPIC_OK/$((TOPIC_OK + TOPIC_FAIL))"
    echo ""
    echo -e "${BLUE}Domain Bridge 将自动转发这些话题到地面站 (Domain 99)${NC}"
    echo ""
    echo -e "${BLUE}测试命令:${NC}"
    echo -e "  # 查看状态"
    echo -e "  ros2 topic echo /usv_01/usv_state"
    echo ""
    echo -e "  # 在地面站查看 (需要 domain_bridge 运行)"
    echo -e "  # 地面站: export ROS_DOMAIN_ID=99"
    echo -e "  # 地面站: ros2 topic echo /usv_01/usv_state"
else
    echo -e "${RED}✗ USV 未正常运行${NC}"
    echo -e "${YELLOW}请检查:${NC}"
    echo -e "  1. USV 节点是否启动"
    echo -e "  2. ROS_DOMAIN_ID 是否正确设置"
    echo -e "  3. MAVROS 是否连接到飞控"
fi

echo ""
