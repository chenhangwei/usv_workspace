#!/bin/bash

# Domain Bridge 诊断脚本
# 用途: 诊断地面站与 USV 之间的跨域通信问题
# 使用方法: ./diagnose_domain_bridge.sh

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${YELLOW}======================================${NC}"
echo -e "${YELLOW}  Domain Bridge 诊断脚本${NC}"
echo -e "${YELLOW}======================================${NC}"
echo ""

# 1. 检查地面站 Domain ID
echo -e "${BLUE}[1/8] 检查地面站 ROS_DOMAIN_ID...${NC}"
export ROS_DOMAIN_ID=99
echo -e "  当前 Domain ID: ${GREEN}$ROS_DOMAIN_ID${NC}"

# 2. 检查 domain_bridge 进程
echo -e "\n${BLUE}[2/8] 检查 domain_bridge 进程...${NC}"
if pgrep -f domain_bridge > /dev/null; then
    pid=$(pgrep -f domain_bridge)
    echo -e "${GREEN}✓ domain_bridge 正在运行 (PID: $pid)${NC}"
else
    echo -e "${RED}✗ domain_bridge 未运行${NC}"
    echo -e "${YELLOW}解决方法: ros2 launch gs_bringup domain_bridge.launch.py${NC}"
    exit 1
fi

# 3. 检查配置文件
echo -e "\n${BLUE}[3/8] 检查配置文件...${NC}"
config_file="$HOME/domain_bridge/domain_bridge.yaml"
if [ -f "$config_file" ]; then
    echo -e "${GREEN}✓ 配置文件存在: $config_file${NC}"
    echo -e "  文件大小: $(du -h "$config_file" | cut -f1)"
    echo -e "  配置的域: $(grep -A 10 "^domains:" "$config_file" | grep -E "^\s+- [0-9]+" | tr -d ' -' | paste -sd ',' -)"
else
    echo -e "${RED}✗ 配置文件不存在${NC}"
    exit 1
fi

# 4. 检查地面站话题
echo -e "\n${BLUE}[4/8] 检查地面站话题 (Domain 99)...${NC}"
gs_topics=$(ros2 topic list 2>/dev/null | wc -l)
echo -e "  地面站话题数: ${GREEN}$gs_topics${NC}"
if [ $gs_topics -eq 0 ]; then
    echo -e "${RED}✗ 地面站没有任何话题${NC}"
fi

# 5. 尝试监听其他域
echo -e "\n${BLUE}[5/8] 扫描其他域的话题...${NC}"
for domain in 0 11 12 13; do
    echo -n "  Domain $domain: "
    topic_count=$(ROS_DOMAIN_ID=$domain timeout 2 ros2 topic list 2>/dev/null | wc -l || echo "0")
    if [ "$topic_count" -gt 0 ]; then
        echo -e "${GREEN}$topic_count 个话题${NC}"
        if [ $domain -eq 11 ]; then
            echo -e "    ${YELLOW}检查 usv_03 话题:${NC}"
            ROS_DOMAIN_ID=$domain ros2 topic list 2>/dev/null | grep "usv_03" | head -5
        fi
    else
        echo -e "${YELLOW}无话题${NC}"
    fi
done

# 6. 检查网络连通性
echo -e "\n${BLUE}[6/8] 检查网络连通性...${NC}"
usv_hosts=("192.168.68.55" "192.168.68.54" "192.168.68.52")
usv_names=("gauss01" "gauss02" "gauss03")

for i in {0..2}; do
    echo -n "  ${usv_names[$i]} (${usv_hosts[$i]}): "
    if ping -c 1 -W 1 ${usv_hosts[$i]} &>/dev/null; then
        echo -e "${GREEN}✓ 可达${NC}"
    else
        echo -e "${RED}✗ 不可达${NC}"
    fi
done

# 7. 检查 DDS 发现
echo -e "\n${BLUE}[7/8] 检查 DDS 参与者发现...${NC}"
echo -e "  ${YELLOW}监听 DDS 发现消息 (3秒)...${NC}"
timeout 3 ros2 daemon stop &>/dev/null || true
discovered=$(timeout 3 ros2 topic list 2>&1 | grep -c "usv_0" || echo "0")
if [ "$discovered" -gt 0 ]; then
    echo -e "${GREEN}✓ 发现 $discovered 个 USV 相关话题${NC}"
else
    echo -e "${YELLOW}! 未发现 USV 话题${NC}"
fi

# 8. 显示 domain_bridge 配置摘要
echo -e "\n${BLUE}[8/8] Domain Bridge 配置摘要...${NC}"
echo -e "  转发规则数: $(grep -c "topic:" "$config_file" || echo "0")"
echo -e "  关键话题配置:"
for topic in "usv_state" "mavros/state" "local_position/pose"; do
    if grep -q "$topic" "$config_file"; then
        echo -e "    - ${GREEN}✓${NC} $topic"
    else
        echo -e "    - ${RED}✗${NC} $topic"
    fi
done

# 诊断结果总结
echo -e "\n${YELLOW}======================================${NC}"
echo -e "${YELLOW}  诊断结果${NC}"
echo -e "${YELLOW}======================================${NC}"
echo ""

# 检查 Domain 11 是否有话题
domain_11_count=$(ROS_DOMAIN_ID=11 timeout 2 ros2 topic list 2>/dev/null | wc -l || echo "0")

if [ "$domain_11_count" -eq 0 ]; then
    echo -e "${RED}问题: Domain 11 没有任何话题${NC}"
    echo -e "\n可能原因:"
    echo -e "  1. ${YELLOW}USV 未设置 ROS_DOMAIN_ID=11${NC}"
    echo -e "     解决: 在 gauss01 上执行:"
    echo -e "       export ROS_DOMAIN_ID=11"
    echo -e "       ros2 launch usv_bringup usv_launch.py"
    echo ""
    echo -e "  2. ${YELLOW}网络不通${NC}"
    echo -e "     检查: ping 192.168.68.101"
    echo ""
    echo -e "  3. ${YELLOW}防火墙阻止 DDS 通信${NC}"
    echo -e "     解决: 在 USV 和地面站上执行:"
    echo -e "       sudo ufw allow from 192.168.68.0/24"
    echo ""
elif [ "$gs_topics" -lt 10 ]; then
    echo -e "${YELLOW}问题: Domain 11 有话题但地面站看不到${NC}"
    echo -e "\n可能原因:"
    echo -e "  1. ${YELLOW}domain_bridge 未正确转发${NC}"
    echo -e "     检查日志: journalctl -u domain_bridge -n 50"
    echo ""
    echo -e "  2. ${YELLOW}配置文件错误${NC}"
    echo -e "     检查: cat $config_file"
    echo ""
else
    echo -e "${GREEN}✓ Domain Bridge 配置正常${NC}"
    echo -e "\n如果仍无法看到 USV 话题,请:"
    echo -e "  1. 重启 domain_bridge"
    echo -e "  2. 确认 USV 正在发布话题"
    echo -e "  3. 检查网络延迟和丢包率"
fi

echo ""
