#!/bin/bash

# 一键启动地面站脚本

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}  USV 地面站启动脚本${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

# 检查网络
echo -e "${YELLOW}[1/4] 检查网络连通性...${NC}"
if ping -c 1 192.168.68.55 > /dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} gauss01 (192.168.68.55) 可达"
else
    echo -e "  ${YELLOW}!${NC} gauss01 不可达 (USV 可能未开机)"
fi

# 进入工作空间
echo -e "\n${YELLOW}[2/4] 准备环境...${NC}"
cd ~/usv_workspace
source install/setup.bash
echo -e "  ${GREEN}✓${NC} ROS 环境已加载"

# 检查是否已有进程运行
echo -e "\n${YELLOW}[3/4] 检查现有进程...${NC}"
if pgrep -f "gs_launch.py" > /dev/null; then
    echo -e "  ${YELLOW}!${NC} 地面站已在运行"
    echo -e "  如需重启,请先执行: pkill -f 'main_gui|domain_bridge'"
    exit 0
fi

# 启动地面站
echo -e "\n${YELLOW}[4/4] 启动地面站...${NC}"
echo -e "  ${GREEN}→${NC} 启动 GUI 和 Domain Bridge"
echo -e "  ${GREEN}→${NC} 按 Ctrl+C 停止"
echo ""
echo -e "${BLUE}======================================${NC}"
echo ""

ros2 launch gs_bringup gs_launch.py
