#!/bin/bash

# 地面站导航更新验证脚本
# 用途: 快速检查所有导航相关代码是否已更新为话题版本

set -e

WORKSPACE_DIR="$HOME/usv_workspace"
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE} 地面站导航更新验证${NC}"
echo -e "${BLUE}========================================${NC}"

cd "$WORKSPACE_DIR/src/gs_gui"

echo -e "\n${YELLOW}[1/5] 检查导航调用是否已更新...${NC}"

# 检查是否还有使用 send_nav_goal_via_action 的地方
action_calls=$(grep -rn "\.send_nav_goal_via_action(" gs_gui/*.py 2>/dev/null | grep -v "def send_nav_goal_via_action" | grep -v "# 旧代码" || true)

if [ -z "$action_calls" ]; then
    echo -e "${GREEN}✓ 所有导航调用已更新为 send_nav_goal_via_topic${NC}"
else
    echo -e "${RED}✗ 发现仍在使用 send_nav_goal_via_action:${NC}"
    echo "$action_calls"
    exit 1
fi

echo -e "\n${YELLOW}[2/5] 检查 cluster_controller.py 更新...${NC}"

# 检查 cluster_controller 中的导航调用
if grep -q "send_nav_goal_via_topic" gs_gui/cluster_controller.py; then
    echo -e "${GREEN}✓ cluster_controller.py 已使用话题版本${NC}"
    
    # 统计调用次数
    count=$(grep -c "send_nav_goal_via_topic" gs_gui/cluster_controller.py)
    echo -e "  └─ 共 ${count} 处调用"
else
    echo -e "${RED}✗ cluster_controller.py 未更新${NC}"
    exit 1
fi

# 检查取消逻辑是否包含话题版本清理
if grep -q "_usv_nav_target_cache" gs_gui/cluster_controller.py; then
    echo -e "${GREEN}✓ 取消逻辑已添加话题版本清理${NC}"
else
    echo -e "${YELLOW}⚠ 取消逻辑未包含话题版本清理 (可选)${NC}"
fi

echo -e "\n${YELLOW}[3/5] 检查 ground_station_node.py 更新...${NC}"

# 检查 ground_station_node 中是否有 send_nav_goal_via_topic 调用
topic_calls_gs=$(grep -n "send_nav_goal_via_topic" gs_gui/ground_station_node.py | grep -v "def send_nav_goal_via_topic")
if [ -n "$topic_calls_gs" ]; then
    echo -e "${GREEN}✓ ground_station_node.py 已使用话题版本${NC}"
    echo "$topic_calls_gs" | while read line; do
        echo -e "  └─ 行 $(echo $line | cut -d: -f1): $(echo $line | cut -d: -f2- | cut -c1-50)..."
    done
else
    echo -e "${RED}✗ ground_station_node.py 未发现话题版本调用${NC}"
    exit 1
fi

# 检查是否有新的话题版本方法
if grep -q "def send_nav_goal_via_topic" gs_gui/ground_station_node.py; then
    echo -e "${GREEN}✓ send_nav_goal_via_topic 方法已定义${NC}"
else
    echo -e "${RED}✗ send_nav_goal_via_topic 方法未找到${NC}"
    exit 1
fi

if grep -q "def navigation_feedback_callback" gs_gui/ground_station_node.py; then
    echo -e "${GREEN}✓ navigation_feedback_callback 方法已定义${NC}"
else
    echo -e "${RED}✗ navigation_feedback_callback 方法未找到${NC}"
    exit 1
fi

if grep -q "def navigation_result_callback" gs_gui/ground_station_node.py; then
    echo -e "${GREEN}✓ navigation_result_callback 方法已定义${NC}"
else
    echo -e "${RED}✗ navigation_result_callback 方法未找到${NC}"
    exit 1
fi

echo -e "\n${YELLOW}[4/5] 检查导航消息导入...${NC}"

# 检查是否导入了新的消息类型
if grep -q "from common_interfaces.msg import NavigationGoal, NavigationFeedback, NavigationResult" gs_gui/usv_manager.py; then
    echo -e "${GREEN}✓ usv_manager.py 已导入话题消息类型${NC}"
else
    echo -e "${YELLOW}⚠ usv_manager.py 可能未正确导入话题消息类型${NC}"
fi

echo -e "\n${YELLOW}[5/5] 统计代码更新...${NC}"

# 统计各文件的更新情况
echo -e "${BLUE}文件更新统计:${NC}"
for file in gs_gui/cluster_controller.py gs_gui/ground_station_node.py gs_gui/usv_manager.py; do
    if [ -f "$file" ]; then
        topic_count=$(grep -c "navigation_goal\|navigation_feedback\|navigation_result" "$file" 2>/dev/null || echo "0")
        echo -e "  • $(basename $file): ${topic_count} 处导航话题相关代码"
    fi
done

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN} 验证完成! ✓${NC}"
echo -e "${GREEN}========================================${NC}"

echo -e "\n${BLUE}下一步:${NC}"
echo "  1. 重新编译: cd ~/usv_workspace && colcon build --packages-select gs_gui"
echo "  2. 启动测试: bash test_topic_navigation.sh"
echo "  3. 查看文档: cat GROUND_STATION_NAVIGATION_UPDATE.md"

echo -e "\n${YELLOW}提示: Action 版本代码已保留用于向后兼容,但不会被调用${NC}"
