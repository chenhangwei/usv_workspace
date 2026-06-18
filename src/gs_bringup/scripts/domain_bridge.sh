#!/bin/bash
# =============================================================================
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of domain bridge.
#
# Author: chenhangwei
# Date: 2026-01-26
# =============================================================================
# Domain Bridge 快速启动脚本
# 用于在地面站后台运行 domain_bridge

set -e

SCRIPT_NAME="domain_bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${DOMAIN_BRIDGE_CONFIG_FILE:-${SCRIPT_DIR}/../config/domain_bridge.yaml}"
LAUNCH_CMD="ros2 launch gs_bringup domain_bridge.launch.py config_file:=${CONFIG_FILE}"
LOCK_FILE="/tmp/domain_bridge.lock"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印函数
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查配置文件
check_config() {
    if [ ! -f "$CONFIG_FILE" ]; then
        print_error "配置文件不存在: $CONFIG_FILE"
        print_info "请确认 gs_bringup 配置目录下存在 domain_bridge.yaml，或通过环境变量 DOMAIN_BRIDGE_CONFIG_FILE 指定路径"
        exit 1
    fi
    print_info "配置文件: $CONFIG_FILE"
}

# 检查 screen 是否安装
check_screen() {
    if ! command -v screen &> /dev/null; then
        print_error "screen 未安装，请先安装："
        echo "  sudo apt-get install screen"
        exit 1
    fi
}

# 启动 domain_bridge
start_bridge() {
    check_config
    check_screen
    
    # 🔒 单例锁检查 - 防止重复启动
    if [ -f "$LOCK_FILE" ]; then
        LOCK_PID=$(cat "$LOCK_FILE")
        if kill -0 "$LOCK_PID" 2>/dev/null; then
            print_error "Domain Bridge 已经在运行中 (PID: $LOCK_PID)"
            print_info "如需重启，请先执行: $0 stop"
            exit 1
        else
            print_warning "发现过期锁文件，清理中..."
            rm -f "$LOCK_FILE"
        fi
    fi
    
    # 检查是否已经在 screen 运行
    if screen -list | grep -q "$SCRIPT_NAME"; then
        print_warning "Domain Bridge 已经在 screen 运行中"
        print_info "如需重启，请先执行: $0 stop"
        exit 1
    fi
    
    # 检查是否有 domain_bridge 进程在运行
    if pgrep -f "domain_bridge.*domain_bridge.yaml" > /dev/null; then
        print_error "检测到 domain_bridge 进程已在运行"
        print_info "进程列表:"
        pgrep -fa "domain_bridge.*domain_bridge.yaml"
        print_info "如需强制重启: $0 stop && $0 start"
        exit 1
    fi
    
    print_info "启动 Domain Bridge (screen session: $SCRIPT_NAME)..."
    
    # 启动前创建锁文件
    echo $$ > "$LOCK_FILE"
    
    screen -dmS "$SCRIPT_NAME" bash -c "$LAUNCH_CMD; exec bash"
    sleep 2
    
    # 验证启动成功
    if screen -list | grep -q "$SCRIPT_NAME"; then
        # 获取实际的 domain_bridge 进程 PID 并更新锁文件
        BRIDGE_PID=$(pgrep -f "domain_bridge.*domain_bridge.yaml" | head -1)
        if [ -n "$BRIDGE_PID" ]; then
            echo "$BRIDGE_PID" > "$LOCK_FILE"
            print_info "Domain Bridge 已成功启动！(PID: $BRIDGE_PID)"
        else
            print_info "Domain Bridge 已成功启动！"
        fi
        print_info "查看日志: $0 attach"
        print_info "停止运行: $0 stop"
    else
        rm -f "$LOCK_FILE"
        print_error "Domain Bridge 启动失败"
        exit 1
    fi
}

# 停止 domain_bridge
stop_bridge() {
    local stopped=false
    
    # 停止 screen 会话
    if screen -list | grep -q "$SCRIPT_NAME"; then
        print_info "正在停止 Domain Bridge (screen)..."
        screen -S "$SCRIPT_NAME" -X quit
        sleep 1
        stopped=true
    fi
    
    # 强制杀掉所有 domain_bridge 进程
    if pgrep -f "domain_bridge.*domain_bridge.yaml" > /dev/null; then
        print_info "清理残留的 domain_bridge 进程..."
        pkill -f "domain_bridge.*domain_bridge.yaml"
        sleep 1
        stopped=true
    fi
    
    # 清理锁文件
    if [ -f "$LOCK_FILE" ]; then
        rm -f "$LOCK_FILE"
        print_info "已清理锁文件"
    fi
    
    if [ "$stopped" = true ]; then
        print_info "Domain Bridge 已停止"
    else
        print_warning "Domain Bridge 未在运行"
    fi
}

# 重启 domain_bridge
restart_bridge() {
    print_info "重启 Domain Bridge..."
    stop_bridge
    sleep 2
    start_bridge
}

# 连接到 screen 会话
attach_bridge() {
    if screen -list | grep -q "$SCRIPT_NAME"; then
        print_info "连接到 Domain Bridge (按 Ctrl+A 然后 D 分离)"
        screen -r "$SCRIPT_NAME"
    else
        print_error "Domain Bridge 未在运行"
        print_info "启动: $0 start"
        exit 1
    fi
}

# 查看状态
status_bridge() {
    echo "======================================"
    echo "Domain Bridge 状态"
    echo "======================================"
    
    # 检查锁文件
    if [ -f "$LOCK_FILE" ]; then
        LOCK_PID=$(cat "$LOCK_FILE")
        if kill -0 "$LOCK_PID" 2>/dev/null; then
            print_info "锁文件: 有效 (PID: $LOCK_PID)"
        else
            print_warning "锁文件: 过期 (进程已不存在)"
        fi
    else
        print_info "锁文件: 不存在"
    fi
    echo ""
    
    # 检查进程
    if pgrep -f "domain_bridge.*domain_bridge.yaml" > /dev/null; then
        print_info "运行状态: ${GREEN}运行中${NC}"
        echo ""
        echo "进程列表:"
        pgrep -fa "domain_bridge.*domain_bridge.yaml"
        echo ""
    else
        print_warning "运行状态: ${RED}未运行${NC}"
        echo ""
    fi
    
    # 检查 screen
    if screen -list | grep -q "$SCRIPT_NAME"; then
        echo "Screen 会话:"
        screen -list | grep "$SCRIPT_NAME"
        echo ""
        print_info "查看实时日志: $0 attach"
    else
        print_info "启动命令: $0 start"
    fi
    
    echo ""
    echo "配置文件: $CONFIG_FILE"
    
    # 检查 ROS 话题
    if command -v ros2 &> /dev/null; then
        echo ""
        echo "ROS Domain ID: $ROS_DOMAIN_ID"
        if [ -n "$ROS_DOMAIN_ID" ]; then
            print_warning "当前终端 ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
            print_info "地面站应使用 Domain 99，运行: export ROS_DOMAIN_ID=99"
        fi
    fi
}

# 显示帮助
show_help() {
    cat << EOF
Domain Bridge 管理脚本

用法:
    $0 {start|stop|restart|attach|status|help}

命令:
    start       启动 Domain Bridge (后台运行)
    stop        停止 Domain Bridge
    restart     重启 Domain Bridge
    attach      连接到运行中的 Domain Bridge (查看日志)
    status      查看运行状态
    help        显示此帮助信息

示例:
    # 启动 domain_bridge
    $0 start

    # 查看实时日志（按 Ctrl+A 然后 D 分离）
    $0 attach

    # 查看状态
    $0 status

    # 停止运行
    $0 stop

配置文件: $CONFIG_FILE

注意事项:
    - 确保地面站设置了正确的 ROS_DOMAIN_ID (export ROS_DOMAIN_ID=99)
    - 确保 USV 设置了各自的 Domain ID (11, 12, 13)
    - 所有设备需要在同一网络
    - 检查防火墙设置，允许 ROS DDS 通信 (端口 7400-7500)
EOF
}

# 主函数
main() {
    case "${1:-}" in
        start)
            start_bridge
            ;;
        stop)
            stop_bridge
            ;;
        restart)
            restart_bridge
            ;;
        attach|logs)
            attach_bridge
            ;;
        status)
            status_bridge
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "未知命令: ${1:-}"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

main "$@"
