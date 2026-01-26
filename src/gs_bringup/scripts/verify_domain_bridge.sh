#!/bin/bash
# =============================================================================
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of verify domain bridge.
#
# Author: chenhangwei
# Date: 2026-01-26
# =============================================================================
# Domain Bridge 验证脚本
# 用于验证 domain_bridge 是否正常转发话题

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Domain Bridge 验证脚本${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查 ROS_DOMAIN_ID
echo -e "${YELLOW}[1/5]${NC} 检查当前 ROS_DOMAIN_ID..."
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo -e "${RED}   ✗ ROS_DOMAIN_ID 未设置${NC}"
    echo -e "${YELLOW}   建议: export ROS_DOMAIN_ID=99${NC}"
else
    echo -e "${GREEN}   ✓ ROS_DOMAIN_ID = $ROS_DOMAIN_ID${NC}"
fi
echo ""

# 检查 domain_bridge 进程
echo -e "${YELLOW}[2/5]${NC} 检查 domain_bridge 进程..."
if pgrep -f "domain_bridge" > /dev/null; then
    echo -e "${GREEN}   ✓ domain_bridge 正在运行${NC}"
    ps aux | grep domain_bridge | grep -v grep | head -1
else
    echo -e "${RED}   ✗ domain_bridge 未运行${NC}"
    echo -e "${YELLOW}   启动: ros2 launch gs_bringup domain_bridge.launch.py${NC}"
fi
echo ""

# 检查配置文件
echo -e "${YELLOW}[3/5]${NC} 检查 domain_bridge 配置..."
CONFIG_FILE="$HOME/domain_bridge/domain_bridge.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo -e "${GREEN}   ✓ 配置文件存在: $CONFIG_FILE${NC}"
    RULE_COUNT=$(grep -c "topic:" "$CONFIG_FILE" || true)
    echo -e "   配置的话题转发规则: ${GREEN}$RULE_COUNT${NC} 条"
else
    echo -e "${RED}   ✗ 配置文件不存在: $CONFIG_FILE${NC}"
fi
echo ""

# 检查 ROS 话题
echo -e "${YELLOW}[4/5]${NC} 检查可见的 USV 话题..."
TOPIC_COUNT=$(ros2 topic list 2>/dev/null | grep -c "usv_" || true)
if [ "$TOPIC_COUNT" -gt 0 ]; then
    echo -e "${GREEN}   ✓ 发现 $TOPIC_COUNT 个 USV 相关话题${NC}"
    echo -e "${BLUE}   示例话题:${NC}"
    ros2 topic list 2>/dev/null | grep "usv_" | head -5 | sed 's/^/      /'
    if [ "$TOPIC_COUNT" -gt 5 ]; then
        echo -e "      ... (还有 $((TOPIC_COUNT - 5)) 个话题)"
    fi
else
    echo -e "${YELLOW}   ⚠ 未发现 USV 话题${NC}"
    echo -e "   可能原因: 1) USV 未启动  2) Domain Bridge 未运行  3) Domain ID 不匹配"
fi
echo ""

# 检查网络连接
echo -e "${YELLOW}[5/5]${NC} 检查 DDS 通信..."
ROS_NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
echo -e "${GREEN}   ✓ 发现 $ROS_NODE_COUNT 个 ROS 节点${NC}"
if [ "$ROS_NODE_COUNT" -gt 0 ]; then
    echo -e "${BLUE}   活跃节点:${NC}"
    ros2 node list 2>/dev/null | head -5 | sed 's/^/      /'
    if [ "$ROS_NODE_COUNT" -gt 5 ]; then
        echo -e "      ... (还有 $((ROS_NODE_COUNT - 5)) 个节点)"
    fi
fi
echo ""

# 总结
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}验证总结${NC}"
echo -e "${BLUE}========================================${NC}"

ISSUES=0
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo -e "${RED}✗ ROS_DOMAIN_ID 未设置${NC}"
    ((ISSUES++))
fi

if ! pgrep -f "domain_bridge" > /dev/null; then
    echo -e "${RED}✗ domain_bridge 未运行${NC}"
    ((ISSUES++))
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}✗ 配置文件不存在${NC}"
    ((ISSUES++))
fi

if [ "$ISSUES" -eq 0 ]; then
    echo -e "${GREEN}✓ 所有检查通过！${NC}"
    echo ""
    echo -e "${BLUE}快速测试命令:${NC}"
    echo -e "  # 监听 USV 状态"
    echo -e "  ros2 topic echo /usv_01/usv_state"
    echo ""
    echo -e "  # 发送控制命令"
    echo -e "  ros2 topic pub /usv_01/set_usv_mode std_msgs/msg/String \"data: 'GUIDED'\" --once"
else
    echo -e "${YELLOW}⚠ 发现 $ISSUES 个问题，请检查上述输出${NC}"
fi

echo ""
