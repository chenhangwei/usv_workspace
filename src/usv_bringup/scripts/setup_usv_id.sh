#!/bin/bash
# ==============================================================================
# USV ID 一键配置脚本
# ==============================================================================
# 在每台机载计算机上首次部署时运行一次即可，之后复制代码无需修改任何文件
#
# 用法:
#   sudo bash setup_usv_id.sh          # 交互式选择
#   sudo bash setup_usv_id.sh 1        # 直接指定 USV ID
#   sudo bash setup_usv_id.sh 2
#   sudo bash setup_usv_id.sh 3
#
# 配置文件位置: /etc/usv_id
# ==============================================================================

set -e

CONFIG_FILE="/etc/usv_id"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════╗"
echo "║       USV 机载计算机身份配置工具          ║"
echo "╚══════════════════════════════════════════╝"
echo -e "${NC}"

# 显示当前配置
if [ -f "$CONFIG_FILE" ]; then
    current_id=$(cat "$CONFIG_FILE" 2>/dev/null || echo "无法读取")
    echo -e "${YELLOW}当前配置: USV ID = ${current_id}${NC}"
else
    echo -e "${YELLOW}当前状态: 尚未配置${NC}"
fi
echo ""

# 获取 USV ID
if [ -n "$1" ]; then
    USV_ID="$1"
else
    echo "请选择本机对应的 USV 编号:"
    echo ""
    echo "  ┌──────┬────────────┬──────────────────┬───────────┐"
    echo "  │  ID  │ namespace  │    本机 IP        │ system_id │"
    echo "  ├──────┼────────────┼──────────────────┼───────────┤"
    echo "  │  1   │ usv_01     │ 192.168.68.55    │    101    │"
    echo "  │  2   │ usv_02     │ 192.168.68.54    │    102    │"
    echo "  │  3   │ usv_03     │ 192.168.68.52    │    103    │"
    echo "  └──────┴────────────┴──────────────────┴───────────┘"
    echo ""
    read -p "输入 USV ID (1/2/3): " USV_ID
fi

# 验证输入
if [[ ! "$USV_ID" =~ ^[1-9][0-9]*$ ]]; then
    echo -e "${RED}错误: USV ID 必须是正整数${NC}"
    exit 1
fi

if [ "$USV_ID" -lt 1 ] || [ "$USV_ID" -gt 3 ]; then
    echo -e "${YELLOW}警告: USV ID = ${USV_ID} 不在默认配置表 (1-3) 中${NC}"
    read -p "是否继续? (y/N): " confirm
    if [ "$confirm" != "y" ] && [ "$confirm" != "Y" ]; then
        echo "已取消"
        exit 0
    fi
fi

# 写入配置
echo "$USV_ID" > "$CONFIG_FILE"
chmod 644 "$CONFIG_FILE"

echo ""
echo -e "${GREEN}✅ 配置完成!${NC}"
echo -e "   配置文件: ${CONFIG_FILE}"
echo -e "   USV ID:   ${USV_ID}"
echo ""
echo -e "${CYAN}验证方法:${NC}"
echo "   cat /etc/usv_id"
echo ""
echo -e "${CYAN}启动命令 (无需任何额外参数):${NC}"
echo "   ros2 launch usv_bringup usv_launch.py"
echo ""

# 可选: 设置主机名 (需要重启生效)
read -p "是否同时将主机名设置为 usv-$(printf '%02d' $USV_ID)? (y/N): " set_hostname
if [ "$set_hostname" = "y" ] || [ "$set_hostname" = "Y" ]; then
    new_hostname="usv-$(printf '%02d' $USV_ID)"
    hostnamectl set-hostname "$new_hostname" 2>/dev/null || echo "$new_hostname" > /etc/hostname
    echo -e "${GREEN}✅ 主机名已设置为: ${new_hostname} (重启后生效)${NC}"
fi
