#!/bin/bash

# 基于话题的导航系统测试脚本
# 用途: 快速验证新导航系统功能

set -e

WORKSPACE_DIR="$HOME/usv_workspace"
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} 导航系统测试脚本${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查环境
cd "$WORKSPACE_DIR"
if [ ! -f "install/setup.bash" ]; then
    echo -e "${RED}错误: 未找到 install/setup.bash${NC}"
    echo -e "${YELLOW}请先编译工作空间: colcon build${NC}"
    exit 1
fi

source install/setup.bash

echo -e "\n${YELLOW}[1/5] 检查消息类型...${NC}"
echo "NavigationGoal:"
ros2 interface show common_interfaces/msg/NavigationGoal | head -5
echo ""
echo "NavigationFeedback:"
ros2 interface show common_interfaces/msg/NavigationFeedback | head -5
echo ""
echo "NavigationResult:"
ros2 interface show common_interfaces/msg/NavigationResult | head -5

echo -e "\n${YELLOW}[2/5] 检查 Domain Bridge 状态...${NC}"
if pgrep -f "domain_bridge" > /dev/null; then
    echo -e "${GREEN}✓ Domain Bridge 正在运行${NC}"
    ps aux | grep domain_bridge | grep -v grep | head -1
else
    echo -e "${RED}✗ Domain Bridge 未运行${NC}"
    echo -e "${YELLOW}尝试启动...${NC}"
    bash src/gs_bringup/scripts/domain_bridge.sh start
fi

echo -e "\n${YELLOW}[3/5] 验证 Domain Bridge 配置...${NC}"
if grep -q "navigation_goal" /home/chenhangwei/domain_bridge/domain_bridge.yaml; then
    echo -e "${GREEN}✓ 导航话题已配置${NC}"
    grep -A 2 "usv_01/navigation_goal" /home/chenhangwei/domain_bridge/domain_bridge.yaml
else
    echo -e "${RED}✗ 导航话题未配置${NC}"
    echo -e "${YELLOW}请检查 Domain Bridge 配置文件${NC}"
fi

echo -e "\n${YELLOW}[4/5] 检查可用节点入口点...${NC}"
if ros2 pkg executables usv_comm | grep -q "navigate_to_point_node"; then
    echo -e "${GREEN}✓ navigate_to_point_node 已注册${NC}"
else
    echo -e "${RED}✗ navigate_to_point_node 未注册${NC}"
    echo -e "${YELLOW}请检查 usv_comm/setup.py${NC}"
fi

echo -e "\n${YELLOW}[5/5] 测试选项${NC}"
echo "请选择测试方式:"
echo "  1) 监听 Domain 99 导航话题 (地面站视角)"
echo "  2) 监听 Domain 11 导航话题 (USV_01 视角)"
echo "  3) 手动发送测试导航目标"
echo "  4) 查看 Domain Bridge 日志"
echo "  5) 退出"
echo ""
read -p "请输入选项 [1-5]: " choice

case $choice in
    1)
        echo -e "${GREEN}监听地面站导航话题 (Domain 99)...${NC}"
        echo "按 Ctrl+C 退出"
        export ROS_DOMAIN_ID=99
        echo ""
        echo "可用话题:"
        ros2 topic list | grep navigation || echo "  (未检测到导航话题)"
        echo ""
        read -p "输入要监听的话题 (例: /usv_01/navigation_feedback): " topic
        ros2 topic echo "$topic"
        ;;
    2)
        echo -e "${GREEN}监听 USV_01 导航话题 (Domain 11)...${NC}"
        echo "按 Ctrl+C 退出"
        export ROS_DOMAIN_ID=11
        echo ""
        echo "可用话题:"
        ros2 topic list | grep navigation || echo "  (未检测到导航话题)"
        echo ""
        read -p "输入要监听的话题 (例: /usv_01/navigation_goal): " topic
        ros2 topic echo "$topic"
        ;;
    3)
        echo -e "${GREEN}发送测试导航目标...${NC}"
        export ROS_DOMAIN_ID=11
        read -p "输入目标 X 坐标 (米): " x
        read -p "输入目标 Y 坐标 (米): " y
        read -p "输入超时时间 (秒, 默认 60): " timeout
        timeout=${timeout:-60}
        
        echo -e "${YELLOW}发送目标: ($x, $y), 超时=${timeout}s${NC}"
        ros2 topic pub --once /usv_01/navigation_goal common_interfaces/msg/NavigationGoal "
goal_id: 999
target_pose:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: 'map'
  pose:
    position:
      x: $x
      y: $y
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
timeout: $timeout
timestamp:
  sec: 0
  nanosec: 0
"
        echo -e "${GREEN}✓ 目标已发送${NC}"
        echo ""
        echo "现在可以监听反馈:"
        echo "  export ROS_DOMAIN_ID=11"
        echo "  ros2 topic echo /usv_01/navigation_feedback"
        ;;
    4)
        echo -e "${GREEN}查看 Domain Bridge 日志...${NC}"
        echo "按 Ctrl+A 然后 D 退出"
        sleep 2
        bash src/gs_bringup/scripts/domain_bridge.sh attach
        ;;
    5)
        echo -e "${GREEN}退出${NC}"
        exit 0
        ;;
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}测试完成!${NC}"
echo -e "${YELLOW}完整测试指南: src/TOPIC_BASED_NAVIGATION_IMPLEMENTATION.md${NC}"
