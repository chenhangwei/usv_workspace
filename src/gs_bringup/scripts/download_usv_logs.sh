#!/bin/bash
# =============================================================================
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of download usv logs.
#
# Author: chenhangwei
# Date: 2026-01-26
# =============================================================================
#
# USV 导航日志下载脚本
# 从远程 USV 下载日志文件到本地
#
# 用法:
#   bash ~/usv_workspace/src/gs_bringup/scripts/download_usv_logs.sh
#   bash ~/usv_workspace/src/gs_bringup/scripts/download_usv_logs.sh --usv 1
#   bash ~/usv_workspace/src/gs_bringup/scripts/download_usv_logs.sh --usv 3 --all
#   bash ~/usv_workspace/src/gs_bringup/scripts/download_usv_logs.sh --list
#   bash ~/usv_workspace/src/gs_bringup/scripts/download_usv_logs.sh --all
#   bash ~/usv_workspace/src/gs_bringup/scripts/download_usv_logs.sh <filename>
#   bash ~/usv_workspace/src/gs_bringup/scripts/download_usv_logs.sh --clean
#   bash ~/usv_workspace/src/gs_bringup/scripts/download_usv_logs.sh --clean-local
#
# 分析日志:
#   python3 ~/usv_workspace/src/usv_control/scripts/analyze_nav_log.py
#   python3 ~/usv_workspace/src/usv_control/scripts/analyze_nav_log.py ~/usv_logs/nav_log_xxx.csv
#
# 配置:
#   修改下方的 USV_HOST 和 USV_USER 变量
#

# ==================== USV 多机配置表 ====================
# 格式: USV_ID → IP 地址
# 新增 USV 时只需在此添加一行
declare -A USV_IP_MAP=(
    [1]="192.168.68.55"    # usv_01
    [2]="192.168.68.54"    # usv_02
    [3]="192.168.68.52"    # usv_03
)

USV_USER="chenhangwei"         # USV 的用户名 (所有 USV 相同)
USV_LOG_DIR="~/usv_logs"       # USV 上的日志目录
LOCAL_LOG_DIR="$HOME/usv_logs" # 本地保存目录
# =========================================================

# 解析 --usv 参数 (从命令行参数中提取，不影响其他参数解析)
SELECTED_USV=""
REMAINING_ARGS=()
for arg in "$@"; do
    if [[ "$arg" =~ ^--usv=([0-9]+)$ ]]; then
        SELECTED_USV="${BASH_REMATCH[1]}"
    elif [[ "$arg" == "--usv" ]]; then
        SELECTED_USV="__next__"  # 标记下一个参数是 USV ID
    elif [[ "$SELECTED_USV" == "__next__" ]]; then
        SELECTED_USV="$arg"
    else
        REMAINING_ARGS+=("$arg")
    fi
done
set -- "${REMAINING_ARGS[@]}"

# 选择 USV
select_usv() {
    if [[ -n "$SELECTED_USV" ]]; then
        if [[ -z "${USV_IP_MAP[$SELECTED_USV]:-}" ]]; then
            echo -e "${RED}错误: USV ID=$SELECTED_USV 不在配置表中${NC}"
            echo "可用: ${!USV_IP_MAP[*]}"
            exit 1
        fi
        USV_HOST="${USV_IP_MAP[$SELECTED_USV]}"
        return
    fi

    # 只有一台时直接使用
    if [[ ${#USV_IP_MAP[@]} -eq 1 ]]; then
        SELECTED_USV="${!USV_IP_MAP[*]}"
        USV_HOST="${USV_IP_MAP[$SELECTED_USV]}"
        return
    fi

    # 交互式选择
    echo -e "${YELLOW}请选择要下载日志的 USV:${NC}"
    echo ""
    for id in $(echo "${!USV_IP_MAP[@]}" | tr ' ' '\n' | sort -n); do
        printf "  [%s]  usv_%02d  (%s)\n" "$id" "$id" "${USV_IP_MAP[$id]}"
    done
    echo "  [a]  全部下载"
    echo ""
    read -p "输入编号 (默认: 3): " choice
    choice=${choice:-3}

    if [[ "$choice" == "a" || "$choice" == "A" ]]; then
        SELECTED_USV="all"
        return
    fi

    if [[ -z "${USV_IP_MAP[$choice]:-}" ]]; then
        echo -e "${RED}无效选择: $choice${NC}"
        exit 1
    fi
    SELECTED_USV="$choice"
    USV_HOST="${USV_IP_MAP[$SELECTED_USV]}"
}

select_usv

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 确保本地目录存在
mkdir -p "$LOCAL_LOG_DIR"

if [[ "$SELECTED_USV" == "all" ]]; then
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    USV 导航日志下载工具 (全部)${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo -e "本地目录: ${GREEN}${LOCAL_LOG_DIR}${NC} (按版本/USV ID 分目录)"
    echo ""
else
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    USV 导航日志下载工具${NC}"
    echo -e "${BLUE}========================================${NC}"
    printf "USV: ${GREEN}usv_%02d${NC} (%s)\n" "$SELECTED_USV" "$USV_HOST"
    echo -e "USV 地址: ${GREEN}${USV_USER}@${USV_HOST}${NC}"
    echo -e "本地目录: ${GREEN}${LOCAL_LOG_DIR}${NC} (按版本/USV ID 分目录)"
    echo ""
fi

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

# 解析远程日志头信息 (USV ID / 版本)
get_remote_meta() {
    local remote_path="$1"
    local header
    local version="V_unknown"
    local usv_id="usv_unknown"

    header=$(ssh "${USV_USER}@${USV_HOST}" "head -n 30 ${remote_path} 2>/dev/null")
    
    # ===== 版本检测 =====
    # 方法1: 从注释头 "MPC Parameters Configuration (vXX)" 中提取
    detected_version=$(echo "$header" | grep -oiE '\(v([0-9]+)\)' | grep -oiE '[0-9]+' | sort -rn | head -1)
    if [[ -n "$detected_version" ]]; then
        version="V${detected_version}"
    else
        # 方法2: 尝试匹配独立的 vXX 关键字
        detected_version=$(echo "$header" | grep -oiE '\bv([0-9]+)\b' | grep -oiE '[0-9]+' | sort -rn | head -1)
        if [[ -n "$detected_version" ]]; then
            version="V${detected_version}"
        fi
    fi
    
    # 方法3 (回退): 从 CSV 列名推断版本
    if [[ "$version" == "V_unknown" ]]; then
        # 获取 CSV 列标题行 (第一个非注释行)
        local csv_header
        csv_header=$(echo "$header" | grep -v '^#' | head -1)
        if [[ -n "$csv_header" ]]; then
            if echo "$csv_header" | grep -q 'wifi_rssi_dbm'; then
                version="V14"   # wifi 字段是最新版本特征
            elif echo "$csv_header" | grep -q 'orca_active'; then
                version="V14"   # ORCA 是 v14 特征
            elif echo "$csv_header" | grep -q 'ampc_enabled'; then
                version="V8"    # AMPC 是 v8 特征
            elif echo "$csv_header" | grep -q 'current_tau_omega'; then
                version="V6"    # adaptive tau 是 v6 特征
            elif echo "$csv_header" | grep -q 'cross_track_error'; then
                version="V5"    # CTE 是 v5 特征
            fi
        fi
    fi

    # ===== USV ID 检测 =====
    # 方法1: 从注释头 "# USV ID: usv_XX" 中提取
    usv_id=$(echo "$header" | grep -i "USV ID" | head -1 | sed -E 's/.*USV[[:space:]]*ID[^:]*:[[:space:]]*//I' | tr -d '\r' | awk '{print $1}')
    if [[ -z "$usv_id" ]]; then
        # 方法2: 匹配注释头中的 usv_XX 模式
        usv_id=$(echo "$header" | grep -io "usv_[0-9][0-9]" | head -1)
    fi
    
    # 方法3 (回退): 从远程主机的 hostname 推断
    if [[ -z "$usv_id" ]]; then
        local remote_hostname
        remote_hostname=$(ssh -o ConnectTimeout=3 "${USV_USER}@${USV_HOST}" "hostname" 2>/dev/null | tr -d '\r')
        if [[ "$remote_hostname" =~ usv_?([0-9]+) ]]; then
            usv_id=$(printf "usv_%02d" "${BASH_REMATCH[1]}")
        fi
    fi
    
    # 方法4 (回退): 从 IP 地址到 USV ID 的映射
    if [[ -z "$usv_id" ]]; then
        case "$USV_HOST" in
            192.168.68.52) usv_id="usv_03" ;;
            192.168.68.54) usv_id="usv_02" ;;
            # 可扩展更多 IP 映射
        esac
    fi

    # 格式化 USV ID
    if [[ -z "$usv_id" ]]; then
        usv_id="usv_unknown"
    elif [[ "$usv_id" =~ ^[0-9]+$ ]]; then
        usv_id=$(printf "usv_%02d" "$usv_id")
    elif [[ "$usv_id" =~ ^usv_[0-9]+$ ]]; then
        num=${usv_id#usv_}
        usv_id=$(printf "usv_%02d" "$num" 2>/dev/null || echo "$usv_id")
    fi

    echo "${version}|${usv_id}"
}

# 根据版本和 USV ID 确定本地保存路径
get_local_dir() {
    local version="$1"
    local usv_id="$2"
    echo "${LOCAL_LOG_DIR}/${version}/${usv_id}"
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
    
    meta=$(get_remote_meta "$latest")
    version=${meta%%|*}
    usv_id=${meta##*|}
    local_dir=$(get_local_dir "$version" "$usv_id")
    mkdir -p "$local_dir"

    echo -n "下载中... "
    if scp "${USV_USER}@${USV_HOST}:${latest}" "${local_dir}/" &>/dev/null; then
        echo -e "${GREEN}✓ 完成${NC}"
        echo ""
        echo -e "已保存到: ${GREEN}${local_dir}/${filename}${NC}"
        
        # 显示文件信息
        echo ""
        echo -e "${YELLOW}文件信息:${NC}"
        ls -lh "${local_dir}/${filename}"
        lines=$(wc -l < "${local_dir}/${filename}")
        echo -e "记录数: ${GREEN}$((lines - 1))${NC} 条 (不含表头)"
        
        # 提示分析
        echo ""
        echo -e "${BLUE}运行分析:${NC}"
        echo -e "  python3 ~/usv_workspace/src/usv_control/scripts/analyze_nav_log.py ${local_dir}/${filename}"
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
        meta=$(get_remote_meta "$file")
        version=${meta%%|*}
        usv_id=${meta##*|}
        local_dir=$(get_local_dir "$version" "$usv_id")
        mkdir -p "$local_dir"
        echo -n "  下载 ${filename} -> ${local_dir}... "
        if scp "${USV_USER}@${USV_HOST}:${file}" "${local_dir}/" &>/dev/null; then
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
    
    meta=$(get_remote_meta "$remote_path")
    version=${meta%%|*}
    usv_id=${meta##*|}
    local_dir=$(get_local_dir "$version" "$usv_id")
    mkdir -p "$local_dir"

    echo -n "下载中... "
    if scp "${USV_USER}@${USV_HOST}:${remote_path}" "${local_dir}/" &>/dev/null; then
        echo -e "${GREEN}✓ 完成${NC}"
        echo -e "已保存到: ${GREEN}${local_dir}/$(basename ${filename})${NC}"
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
    
    count=$(find "${LOCAL_LOG_DIR}" -type f -name "nav_log_*.csv" 2>/dev/null | wc -l)
    
    if [ "$count" -eq 0 ]; then
        echo -e "${YELLOW}本地没有日志文件${NC}"
        return 0
    fi
    
    echo -e "将删除 ${RED}${count}${NC} 个本地日志文件:"
    find "${LOCAL_LOG_DIR}" -type f -name "nav_log_*.csv" -exec ls -lh {} \; 2>/dev/null | while read line; do
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
    if find "${LOCAL_LOG_DIR}" -type f -name "nav_log_*.csv" -exec rm -f {} \; 2>/dev/null; then
        echo -e "${GREEN}✓ 完成${NC}"
        echo -e "已清除 ${GREEN}${count}${NC} 个本地日志文件"
    else
        echo -e "${RED}✗ 删除失败${NC}"
        return 1
    fi
}

# 对单台 USV 执行操作
run_for_one_usv() {
    local action="$1"
    if ! check_connection; then
        return 1
    fi
    echo ""
    
    case "$action" in
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
        "")
            download_latest
            ;;
        *)
            download_file "$action"
            ;;
    esac
}

# 主程序
main() {
    local action="${1:-}"

    # 帮助信息
    if [[ "$action" == "--help" || "$action" == "-h" ]]; then
        echo "用法:"
        echo "  $0 [--usv N] [操作]"
        echo ""
        echo "USV 选择:"
        echo "  --usv 1           指定 usv_01"
        echo "  --usv 3           指定 usv_03"
        echo "  --usv=2           指定 usv_02"
        echo "  (不指定)           交互式选择"
        echo ""
        echo "操作:"
        echo "  (无参数)           下载最新的日志文件"
        echo "  --list             列出远程日志文件"
        echo "  --all              下载所有日志文件"
        echo "  <filename>         下载指定文件"
        echo "  --clean            清除远程 USV 上的日志"
        echo "  --clean-local      清除本地日志"
        echo ""
        echo "示例:"
        echo "  $0 --usv 1                从 usv_01 下载最新日志"
        echo "  $0 --usv 3 --all          从 usv_03 下载所有日志"
        echo "  $0 --usv=2 --list         列出 usv_02 的远程日志"
        return 0
    fi

    if [[ "$SELECTED_USV" == "all" ]]; then
        # 遍历所有 USV
        for id in $(echo "${!USV_IP_MAP[@]}" | tr ' ' '\n' | sort -n); do
            USV_HOST="${USV_IP_MAP[$id]}"
            printf "\n${BLUE}━━━ usv_%02d (%s) ━━━${NC}\n" "$id" "$USV_HOST"
            run_for_one_usv "$action"
        done
    else
        run_for_one_usv "$action"
    fi
}

main "$@"
