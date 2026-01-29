#!/bin/bash
# =============================================================================
# 同步代码到远程 USV 脚本
# =============================================================================
# 
# 用法: ./deploy_to_usv.sh
# 
# 功能: 将本地通过 Copilot 修复的代码 (src/usv_control) 同步到远程 USV
#

# 配置 (与 download_usv_logs.sh 保持一致)
USV_HOST="192.168.68.54"
USV_USER="chenhangwei"
REMOTE_WS="~/usv_workspace/src"

# 颜色
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${YELLOW}正在准备将代码同步到 ${USV_USER}@${USV_HOST} ...${NC}"

# 1. 检查本地是否有改动过的文件
# 重点同步 usv_control 包
LOCAL_PATH="$HOME/usv_workspace/src/usv_control"

if [ ! -d "$LOCAL_PATH" ]; then
    echo -e "${RED}错误: 找不到本地路径 $LOCAL_PATH${NC}"
    exit 1
fi

# 2. 检查网络连接
if ! ssh -o ConnectTimeout=3 -o BatchMode=yes "${USV_USER}@${USV_HOST}" "echo ok" &>/dev/null; then
    echo -e "${RED}无法连接到 USV。请检查网络或 SSH 配置。${NC}"
    exit 1
fi

echo -e "${GREEN}连接成功。开始同步 usv_control...${NC}"

# 3. 使用 rsync 同步 (如果 rsync 不存在则回退到 scp，但 scp 对文件夹较慢)
# 排除 __pycache__ 和 .git
rsync -avz --progress --exclude '__pycache__' --exclude '*.pyc' \
    "$LOCAL_PATH" "${USV_USER}@${USV_HOST}:${REMOTE_WS}/"

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✅ 代码同步成功！${NC}"
    echo ""
    echo -e "${YELLOW}下一步操作指南:${NC}"
    echo "1. SSH 登录到 USV:"
    echo "   ssh ${USV_USER}@${USV_HOST}"
    echo ""
    echo "2. 在 USV 上重新编译 python 包 (使改动生效):"
    echo "   cd ~/usv_workspace"
    echo "   colcon build --packages-select usv_control --symlink-install"
    echo ""
    echo "3. 重启控制节点:"
    echo "   (在运行节点的终端按 Ctrl+C 停止，然后重新运行 launch 脚本)"
else
    echo -e "${RED}❌ 同步失败${NC}"
fi
