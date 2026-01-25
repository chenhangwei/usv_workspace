#!/bin/bash
#
# USV 导航日志下载脚本
# 从远程 USV 下载日志文件到本地
#
# 用法:
#   ./download_usv_logs.sh              # 下载最新的日志
#   ./download_usv_logs.sh --list       # 列出远程日志文件
#   ./download_usv_logs.sh --all        # 下载所有日志
#   ./download_usv_logs.sh <filename>   # 下载指定文件
#   ./download_usv_logs.sh --clean      # 清除远程 USV 上的日志
#   ./download_usv_logs.sh --clean-local  # 清除本地日志
#
# 分析日志:
#   python3 ~/usv_workspace/src/usv_control/scripts/analyze_nav_log.py
#   python3 ~/usv_workspace/src/usv_control/scripts/analyze_nav_log.py ~/usv_logs/nav_log_xxx.csv
#
# 配置:
#   修改下方的 USV_HOST 和 USV_USER 变量
#

# ==================== 配置 ====================
USV_HOST="192.168.68.54"      # USV 的 IP 地址
#USV_HOST="192.168.68.52"      # USV 的 IP 地址
USV_USER="chenhangwei"         # USV 的用户名
USV_LOG_DIR="~/usv_logs"       # USV 上的日志目录
LOCAL_LOG_DIR="$HOME/usv_logs" # 本地保存目录
# ==============================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 确保本地目录存在
mkdir -p "$LOCAL_LOG_DIR"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    USV 导航日志下载工具${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "USV 地址: ${GREEN}${USV_USER}@${USV_HOST}${NC}"
echo -e "本地目录: ${GREEN}${LOCAL_LOG_DIR}${NC}"
echo ""

# 检查连接
check_connection() {
    echo -n "检查连接... "
    if ssh -o ConnectTimeout=5 -o BatchMode=yes "${USV_USER}@${USV_HOST}" "echo ok" &>/dev/null; then
        echo -e "${GREEN}✓ 连接成功${NC}"
        return 0
    else
        echo -e "${RED}✗ 连接失败${NC}"
        echo -e "${YELLOW}提示: 确保 USV 在线且已配置 SSH 免密登录${NC}"
        echo -e "${YELLOW}      运行: ssh-copy-id ${USV_USER}@${USV_HOST}${NC}"
        return 1
    fi
}

# 列出远程日志文件
list_logs() {
    echo -e "${YELLOW}远程日志文件列表:${NC}"
    echo ""
    ssh "${USV_USER}@${USV_HOST}" "ls -lh ${USV_LOG_DIR}/nav_log_*.csv 2>/dev/null" | while read line; do
        echo "  $line"
    done
    
    # 统计文件数量
    count=$(ssh "${USV_USER}@${USV_HOST}" "ls ${USV_LOG_DIR}/nav_log_*.csv 2>/dev/null | wc -l")
    echo ""
    echo -e "共 ${GREEN}${count}${NC} 个日志文件"
}

# 下载最新的日志文件
download_latest() {
    echo -e "${YELLOW}查找最新日志...${NC}"
    
    latest=$(ssh "${USV_USER}@${USV_HOST}" "ls -t ${USV_LOG_DIR}/nav_log_*.csv 2>/dev/null | head -1")
    
    if [ -z "$latest" ]; then
        echo -e "${RED}未找到日志文件${NC}"
        return 1
    fi
    
    filename=$(basename "$latest")
    echo -e "最新日志: ${GREEN}${filename}${NC}"
    echo ""
    
    echo -n "下载中... "
    if scp "${USV_USER}@${USV_HOST}:${latest}" "${LOCAL_LOG_DIR}/" &>/dev/null; then
        echo -e "${GREEN}✓ 完成${NC}"
        echo ""
        echo -e "已保存到: ${GREEN}${LOCAL_LOG_DIR}/${filename}${NC}"
        
        # 显示文件信息
        echo ""
        echo -e "${YELLOW}文件信息:${NC}"
        ls -lh "${LOCAL_LOG_DIR}/${filename}"
        lines=$(wc -l < "${LOCAL_LOG_DIR}/${filename}")
        echo -e "记录数: ${GREEN}$((lines - 1))${NC} 条 (不含表头)"
        
        # 提示分析
        echo ""
        echo -e "${BLUE}运行分析:${NC}"
        echo -e "  python3 ~/usv_workspace/src/usv_control/scripts/analyze_nav_log.py ${LOCAL_LOG_DIR}/${filename}"
    else
        echo -e "${RED}✗ 下载失败${NC}"
        return 1
    fi
}

# 下载所有日志文件
download_all() {
    echo -e "${YELLOW}下载所有日志文件...${NC}"
    echo ""
    
    # 获取文件列表
    files=$(ssh "${USV_USER}@${USV_HOST}" "ls ${USV_LOG_DIR}/nav_log_*.csv 2>/dev/null")
    
    if [ -z "$files" ]; then
        echo -e "${RED}未找到日志文件${NC}"
        return 1
    fi
    
    count=0
    for file in $files; do
        filename=$(basename "$file")
        echo -n "  下载 ${filename}... "
        if scp "${USV_USER}@${USV_HOST}:${file}" "${LOCAL_LOG_DIR}/" &>/dev/null; then
            echo -e "${GREEN}✓${NC}"
            ((count++))
        else
            echo -e "${RED}✗${NC}"
        fi
    done
    
    echo ""
    echo -e "共下载 ${GREEN}${count}${NC} 个文件到 ${LOCAL_LOG_DIR}/"
}

# 下载指定文件
download_file() {
    filename="$1"
    
    # 如果只给了文件名，添加路径
    if [[ "$filename" != */* ]]; then
        remote_path="${USV_LOG_DIR}/${filename}"
    else
        remote_path="$filename"
    fi
    
    echo -e "${YELLOW}下载: ${filename}${NC}"
    echo ""
    
    echo -n "下载中... "
    if scp "${USV_USER}@${USV_HOST}:${remote_path}" "${LOCAL_LOG_DIR}/" &>/dev/null; then
        echo -e "${GREEN}✓ 完成${NC}"
        echo -e "已保存到: ${GREEN}${LOCAL_LOG_DIR}/$(basename ${filename})${NC}"
    else
        echo -e "${RED}✗ 下载失败${NC}"
        echo -e "${YELLOW}提示: 检查文件名是否正确${NC}"
        return 1
    fi
}

# 清除远程 USV 上的日志文件
clean_remote_logs() {
    echo -e "${YELLOW}清除远程 USV 日志文件...${NC}"
    echo ""
    
    # 先列出要删除的文件
    count=$(ssh "${USV_USER}@${USV_HOST}" "ls ${USV_LOG_DIR}/nav_log_*.csv 2>/dev/null | wc -l")
    
    if [ "$count" -eq 0 ]; then
        echo -e "${YELLOW}远程没有日志文件${NC}"
        return 0
    fi
    
    echo -e "将删除 ${RED}${count}${NC} 个日志文件:"
    ssh "${USV_USER}@${USV_HOST}" "ls -lh ${USV_LOG_DIR}/nav_log_*.csv 2>/dev/null" | while read line; do
        echo "  $line"
    done
    echo ""
    
    # 确认
    read -p "确认删除? (y/N): " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo -e "${YELLOW}已取消${NC}"
        return 0
    fi
    
    echo -n "删除中... "
    if ssh "${USV_USER}@${USV_HOST}" "rm -f ${USV_LOG_DIR}/nav_log_*.csv" 2>/dev/null; then
        echo -e "${GREEN}✓ 完成${NC}"
        echo -e "已清除远程 USV 上的 ${GREEN}${count}${NC} 个日志文件"
    else
        echo -e "${RED}✗ 删除失败${NC}"
        return 1
    fi
}

# 清除本地日志文件
clean_local_logs() {
    echo -e "${YELLOW}清除本地日志文件...${NC}"
    echo ""
    
    # 检查本地日志
    if [ ! -d "$LOCAL_LOG_DIR" ]; then
        echo -e "${YELLOW}本地日志目录不存在${NC}"
        return 0
    fi
    
    count=$(ls "${LOCAL_LOG_DIR}"/nav_log_*.csv 2>/dev/null | wc -l)
    
    if [ "$count" -eq 0 ]; then
        echo -e "${YELLOW}本地没有日志文件${NC}"
        return 0
    fi
    
    echo -e "将删除 ${RED}${count}${NC} 个本地日志文件:"
    ls -lh "${LOCAL_LOG_DIR}"/nav_log_*.csv 2>/dev/null | while read line; do
        echo "  $line"
    done
    echo ""
    
    # 确认
    read -p "确认删除? (y/N): " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo -e "${YELLOW}已取消${NC}"
        return 0
    fi
    
    echo -n "删除中... "
    if rm -f "${LOCAL_LOG_DIR}"/nav_log_*.csv 2>/dev/null; then
        echo -e "${GREEN}✓ 完成${NC}"
        echo -e "已清除 ${GREEN}${count}${NC} 个本地日志文件"
    else
        echo -e "${RED}✗ 删除失败${NC}"
        return 1
    fi
}

# 主程序
main() {
    if ! check_connection; then
        exit 1
    fi
    echo ""
    
    case "${1:-}" in
        --list|-l)
            list_logs
            ;;
        --all|-a)
            download_all
            ;;
        --clean|-c)
            clean_remote_logs
            ;;
        --clean-local|-cl)
            clean_local_logs
            ;;
        --help|-h)
            echo "用法:"
            echo "  $0              下载最新的日志文件"
            echo "  $0 --list       列出远程日志文件"
            echo "  $0 --all        下载所有日志文件"
            echo "  $0 <filename>   下载指定文件"
            echo "  $0 --clean      清除远程 USV 上的日志"
            echo "  $0 --clean-local  清除本地日志"
            ;;
        "")
            download_latest
            ;;
        *)
            download_file "$1"
            ;;
    esac
}

main "$@"
