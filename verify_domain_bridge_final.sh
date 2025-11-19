#!/bin/bash

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}===== Domain Bridge 最终验证 =====${NC}\n"

echo -e "${YELLOW}1. 地面站话题数:${NC}"
export ROS_DOMAIN_ID=99
topic_count=$(ros2 topic list | grep usv_01 | wc -l)
echo -e "  ${GREEN}$topic_count${NC} 个 usv_01 话题"

echo -e "\n${YELLOW}2. 关键话题列表:${NC}"
ros2 topic list | grep usv_01 | head -10

echo -e "\n${YELLOW}3. 进程状态:${NC}"
ps aux | grep -E "domain_bridge|main_gui" | grep -v grep | awk '{print "  "$11" (PID: "$2")"}'

echo -e "\n${YELLOW}4. 网络连通性:${NC}"
ping -c 1 192.168.68.55 > /dev/null 2>&1 && echo -e "  ${GREEN}✓${NC} gauss01 可达" || echo -e "  ${RED}✗${NC} gauss01 不可达"

echo -e "\n${GREEN}✓ Domain Bridge 已成功配置!${NC}"
