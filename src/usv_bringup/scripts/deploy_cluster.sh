#!/bin/bash
# =============================================================================
# 集群无人球部署脚本
# 用于批量启动 PX4 uXRCE-DDS 架构的多台 USV
# =============================================================================

# 网络配置
GROUND_STATION_IP="192.168.68.50"
ROUTER_PORT="7447"

# USV 配置表
# 格式: USV_ID:IP:SERIAL_PORT:GROUP
declare -A USV_CONFIG
USV_CONFIG["usv_01"]="192.168.68.55:/dev/ttyUSB0:A"
USV_CONFIG["usv_02"]="192.168.68.54:/dev/ttyUSB0:A"
USV_CONFIG["usv_03"]="192.168.68.52:/dev/ttyUSB0:A"
# 添加更多 USV...
# USV_CONFIG["usv_04"]="192.168.68.xx:/dev/ttyUSB0:A"
# USV_CONFIG["usv_09"]="192.168.68.xx:/dev/ttyUSB0:B"  # 分组 B

# SSH 用户名
SSH_USER="root"

# 远程工作空间路径
REMOTE_WORKSPACE="/home/usv/usv_workspace"

# ROS 2 发行版（jazzy, kilted, humble）
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# =============================================================================
# 函数定义
# =============================================================================

print_header() {
    echo ""
    echo "============================================================"
    echo "$1"
    echo "============================================================"
}

print_info() {
    echo "[INFO] $1"
}

print_success() {
    echo "[✓] $1"
}

print_error() {
    echo "[✗] $1"
}

# 部署单个 USV
deploy_usv() {
    local usv_id=$1
    local config=${USV_CONFIG[$usv_id]}
    
    if [ -z "$config" ]; then
        print_error "未找到 $usv_id 的配置"
        return 1
    fi
    
    # 解析配置
    IFS=':' read -r usv_ip serial_port group_id <<< "$config"
    
    print_info "部署 $usv_id ($usv_ip) -> 分组 $group_id"
    
    # 远程执行启动命令
    ssh -o ConnectTimeout=5 ${SSH_USER}@${usv_ip} << EOF
        # 加载 ROS 环境（支持 jazzy, kilted, humble）
        if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
            source /opt/ros/${ROS_DISTRO}/setup.bash
        elif [ -f /opt/ros/jazzy/setup.bash ]; then
            source /opt/ros/jazzy/setup.bash
        elif [ -f /opt/ros/kilted/setup.bash ]; then
            source /opt/ros/kilted/setup.bash
        else
            source /opt/ros/humble/setup.bash
        fi
        source ${REMOTE_WORKSPACE}/install/setup.bash
        
        # 停止已有进程
        pkill -f "usv_px4_dds_launch" 2>/dev/null || true
        pkill -f "MicroXRCEAgent" 2>/dev/null || true
        sleep 1
        
        # 设置环境变量
        export ROS_DOMAIN_ID=\$(python3 -c "print({'A': 10, 'B': 20, 'C': 30, 'D': 40, 'E': 50, 'F': 60}.get('$group_id', 10))")
        
        # 启动 USV 节点
        nohup ros2 launch usv_bringup usv_px4_dds_launch.py \
            namespace:=${usv_id} \
            group_id:=${group_id} \
            serial_port:=${serial_port} \
            router_ip:=${GROUND_STATION_IP} \
            use_zenoh:=true \
            > /tmp/${usv_id}_launch.log 2>&1 &
        
        echo "启动完成，日志: /tmp/${usv_id}_launch.log"
EOF
    
    if [ $? -eq 0 ]; then
        print_success "$usv_id 部署成功"
    else
        print_error "$usv_id 部署失败"
    fi
}

# 停止单个 USV
stop_usv() {
    local usv_id=$1
    local config=${USV_CONFIG[$usv_id]}
    
    if [ -z "$config" ]; then
        print_error "未找到 $usv_id 的配置"
        return 1
    fi
    
    IFS=':' read -r usv_ip _ _ <<< "$config"
    
    print_info "停止 $usv_id ($usv_ip)"
    
    ssh -o ConnectTimeout=5 ${SSH_USER}@${usv_ip} << EOF
        pkill -f "usv_px4_dds_launch" 2>/dev/null || true
        pkill -f "MicroXRCEAgent" 2>/dev/null || true
        pkill -f "usv_control_px4" 2>/dev/null || true
        pkill -f "usv_command_px4" 2>/dev/null || true
        pkill -f "usv_status_px4" 2>/dev/null || true
        echo "已停止所有 USV 进程"
EOF
    
    print_success "$usv_id 已停止"
}

# 检查 USV 状态
check_usv() {
    local usv_id=$1
    local config=${USV_CONFIG[$usv_id]}
    
    if [ -z "$config" ]; then
        print_error "未找到 $usv_id 的配置"
        return 1
    fi
    
    IFS=':' read -r usv_ip _ _ <<< "$config"
    
    # 检查网络连通性
    if ping -c 1 -W 1 ${usv_ip} > /dev/null 2>&1; then
        echo "[✓] $usv_id ($usv_ip) - 网络正常"
        
        # 检查进程
        ssh -o ConnectTimeout=5 ${SSH_USER}@${usv_ip} "pgrep -f MicroXRCEAgent > /dev/null && echo '  [✓] Agent 运行中' || echo '  [✗] Agent 未运行'"
        ssh -o ConnectTimeout=5 ${SSH_USER}@${usv_ip} "pgrep -f usv_control_px4 > /dev/null && echo '  [✓] 控制节点运行中' || echo '  [✗] 控制节点未运行'"
    else
        echo "[✗] $usv_id ($usv_ip) - 网络不通"
    fi
}

# 启动地面站
start_ground_station() {
    print_header "启动地面站"
    
    # 加载环境
    source /opt/ros/humble/setup.bash
    source ~/usv_workspace/install/setup.bash
    
    # 首先启动 Zenoh Router
    print_info "启动 Zenoh Router..."
    zenohd -c ~/usv_workspace/src/gs_bringup/config/zenoh_router_config.json5 &
    sleep 2
    
    # 启动地面站节点
    print_info "启动地面站节点..."
    ros2 launch gs_bringup gs_px4_dds_launch.py \
        router_ip:=${GROUND_STATION_IP} \
        use_zenoh:=true
}

# 同步代码到所有 USV
sync_code() {
    print_header "同步代码到所有 USV"
    
    for usv_id in "${!USV_CONFIG[@]}"; do
        local config=${USV_CONFIG[$usv_id]}
        IFS=':' read -r usv_ip _ _ <<< "$config"
        
        print_info "同步到 $usv_id ($usv_ip)..."
        
        rsync -avz --exclude='build' --exclude='install' --exclude='log' \
            ~/usv_workspace/src/ \
            ${SSH_USER}@${usv_ip}:${REMOTE_WORKSPACE}/src/
        
        if [ $? -eq 0 ]; then
            print_success "$usv_id 同步完成"
        else
            print_error "$usv_id 同步失败"
        fi
    done
}

# 远程编译
build_remote() {
    print_header "远程编译所有 USV"
    
    for usv_id in "${!USV_CONFIG[@]}"; do
        local config=${USV_CONFIG[$usv_id]}
        IFS=':' read -r usv_ip _ _ <<< "$config"
        
        print_info "编译 $usv_id ($usv_ip)..."
        
        ssh -o ConnectTimeout=5 ${SSH_USER}@${usv_ip} << EOF
            source /opt/ros/humble/setup.bash
            cd ${REMOTE_WORKSPACE}
            colcon build --packages-select px4_msgs common_interfaces usv_control usv_comm usv_bringup
            echo "编译完成"
EOF
    done
}

# =============================================================================
# 主程序
# =============================================================================

show_usage() {
    echo "用法: $0 <命令> [参数]"
    echo ""
    echo "命令:"
    echo "  deploy-all      部署所有 USV"
    echo "  deploy <usv_id> 部署指定 USV"
    echo "  stop-all        停止所有 USV"
    echo "  stop <usv_id>   停止指定 USV"
    echo "  status          检查所有 USV 状态"
    echo "  gs              启动地面站"
    echo "  sync            同步代码到所有 USV"
    echo "  build           远程编译所有 USV"
    echo ""
    echo "示例:"
    echo "  $0 deploy-all"
    echo "  $0 deploy usv_01"
    echo "  $0 gs"
}

case "$1" in
    deploy-all)
        print_header "部署所有 USV"
        for usv_id in "${!USV_CONFIG[@]}"; do
            deploy_usv "$usv_id"
        done
        ;;
    deploy)
        if [ -z "$2" ]; then
            print_error "请指定 USV ID"
            exit 1
        fi
        deploy_usv "$2"
        ;;
    stop-all)
        print_header "停止所有 USV"
        for usv_id in "${!USV_CONFIG[@]}"; do
            stop_usv "$usv_id"
        done
        ;;
    stop)
        if [ -z "$2" ]; then
            print_error "请指定 USV ID"
            exit 1
        fi
        stop_usv "$2"
        ;;
    status)
        print_header "检查 USV 状态"
        echo ""
        echo "地面站: ${GROUND_STATION_IP}"
        echo ""
        for usv_id in "${!USV_CONFIG[@]}"; do
            check_usv "$usv_id"
        done
        ;;
    gs)
        start_ground_station
        ;;
    sync)
        sync_code
        ;;
    build)
        build_remote
        ;;
    *)
        show_usage
        exit 1
        ;;
esac
