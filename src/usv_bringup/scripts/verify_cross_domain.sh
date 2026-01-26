#!/bin/bash
# =============================================================================
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of verify cross domain.
#
# Author: chenhangwei
# Date: 2026-01-26
# =============================================================================

# 跨域通信验证脚本
# 用途: 验证 gauss01 (Domain 11) 与地面站 (Domain 99) 之间的话题转发
# 使用方法: ./verify_cross_domain.sh

set -e

# 颜色输出
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}======================================${NC}"
echo -e "${YELLOW}  USV 跨域通信验证脚本${NC}"
echo -e "${YELLOW}======================================${NC}"
echo ""

# 1. 验证当前域 ID
echo -e "${YELLOW}[1/5] 检查当前 ROS_DOMAIN_ID...${NC}"
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo -e "${RED}✗ ROS_DOMAIN_ID 未设置${NC}"
    exit 1
else
    echo -e "${GREEN}✓ ROS_DOMAIN_ID = $ROS_DOMAIN_ID${NC}"
fi

# 2. 检查 gauss01 的话题
echo -e "\n${YELLOW}[2/5] 检查本机 USV 话题...${NC}"
usv_topics=$(ros2 topic list | grep "usv_0" || true)
if [ -z "$usv_topics" ]; then
    echo -e "${RED}✗ 未发现任何 USV 话题${NC}"
    echo -e "${YELLOW}提示: 确保 USV 已启动: ros2 launch usv_bringup usv_launch.py${NC}"
    exit 1
else
    topic_count=$(echo "$usv_topics" | wc -l)
    echo -e "${GREEN}✓ 发现 $topic_count 个 USV 话题${NC}"
    echo "$usv_topics" | head -5
    if [ $topic_count -gt 5 ]; then
        echo "... (还有 $((topic_count - 5)) 个话题)"
    fi
fi

# 3. 检查关键话题的发布频率
echo -e "\n${YELLOW}[3/5] 检查关键话题发布频率...${NC}"
check_topic_rate() {
    local topic=$1
    local expected_rate=$2
    
    if ros2 topic list | grep -q "$topic"; then
        echo -n "  - $topic: "
        actual_rate=$(timeout 3 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
        if [ -z "$actual_rate" ] || [ "$actual_rate" = "0" ]; then
            echo -e "${YELLOW}无数据${NC}"
        else
            echo -e "${GREEN}${actual_rate} Hz${NC}"
        fi
    else
        echo -e "  - $topic: ${RED}不存在${NC}"
    fi
}

check_topic_rate "/usv_03/mavros/state" "1"
check_topic_rate "/usv_03/local_position/pose" "50"
check_topic_rate "/usv_03/usv_state" "10"

# 4. 发布测试消息
echo -e "\n${YELLOW}[4/5] 测试话题发布...${NC}"
echo "  正在发布测试消息到 /test_domain_${ROS_DOMAIN_ID}..."
(ros2 topic pub --once /test_domain_${ROS_DOMAIN_ID} std_msgs/msg/String "{data: 'Test from Domain ${ROS_DOMAIN_ID}'}" 2>&1 | grep -q "publisher") && \
    echo -e "${GREEN}✓ 测试消息发布成功${NC}" || \
    echo -e "${RED}✗ 测试消息发布失败${NC}"

# 5. 检查是否需要持久化配置
echo -e "\n${YELLOW}[5/5] 检查持久化配置...${NC}"
if grep -q "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" ~/.bashrc 2>/dev/null; then
    echo -e "${GREEN}✓ ROS_DOMAIN_ID 已写入 ~/.bashrc${NC}"
else
    echo -e "${YELLOW}! ROS_DOMAIN_ID 未持久化${NC}"
    echo -e "${YELLOW}建议执行以下命令:${NC}"
    echo -e "  echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}' >> ~/.bashrc"
fi

# 显示地面站验证命令
echo -e "\n${YELLOW}======================================${NC}"
echo -e "${YELLOW}下一步: 在地面站验证跨域通信${NC}"
echo -e "${YELLOW}======================================${NC}"
echo ""
echo -e "在地面站执行以下命令:"
echo -e "  ${GREEN}export ROS_DOMAIN_ID=99${NC}"
echo -e "  ${GREEN}ros2 topic list | grep usv_03${NC}"
echo ""
echo -e "预期结果: 应该看到约 28 个 /usv_03/* 话题"
echo -e "示例话题:"
echo -e "  - /usv_03/mavros/state"
echo -e "  - /usv_03/local_position/pose"
echo -e "  - /usv_03/usv_state"
echo -e "  - /usv_03/set_usv_target_position"
echo ""
echo -e "如果地面站能看到这些话题,说明 Domain Bridge 工作正常!"
echo ""

# 节点信息
echo -e "${YELLOW}当前运行的 ROS 2 节点:${NC}"
ros2 node list | grep "usv_03" | head -10
echo ""
