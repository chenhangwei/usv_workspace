#!/bin/bash
# =============================================================================
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of quick start.
#
# Author: chenhangwei
# Date: 2026-01-26
# =============================================================================

# ROS 2 分布式启动 - 快速测试脚本
# 该脚本演示如何使用 ROS 2 原生分布式 launch 系统启动 USV 集群

set -e  # 遇到错误立即退出

echo "========================================="
echo "ROS 2 分布式启动 - USV 集群系统"
echo "========================================="
echo ""

# 检查是否已 source 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 错误：ROS 2 环境未加载"
    echo "请先执行：source /opt/ros/humble/setup.bash"
    exit 1
fi

if [ ! -f "install/setup.bash" ]; then
    echo "❌ 错误：工作空间未编译"
    echo "请先执行：colcon build"
    exit 1
fi

# Source 工作空间
source install/setup.bash
echo "✅ 工作空间已加载"
echo ""

# 检查配置文件是否存在
FLEET_CONFIG="src/gs_bringup/config/usv_fleet.yaml"
if [ ! -f "$FLEET_CONFIG" ]; then
    echo "⚠️  警告：未找到集群配置文件 $FLEET_CONFIG"
    echo "将使用默认配置"
else
    echo "✅ 集群配置文件：$FLEET_CONFIG"
    echo ""
    echo "启用的 USV："
    grep "enabled: true" "$FLEET_CONFIG" -B 1 | grep -E "usv_[0-9]+" || echo "  (无)"
    echo ""
fi

# 询问启动模式
echo "请选择启动模式："
echo "  1. 分布式启动（通过 SSH 启动远程 USV）"
echo "  2. 仅启动地面站"
echo ""
read -p "请输入选项 [1-2]: " choice

case $choice in
    1)
        echo ""
        echo "========================================="
        echo "启动模式：分布式启动"
        echo "========================================="
        echo ""
        echo "⚠️  请确保："
        echo "  1. 已配置 SSH 免密登录到所有 USV"
        echo "  2. 所有 USV 的工作空间已编译"
        echo "  3. 网络连接正常"
        echo ""
        read -p "确认继续？(y/n): " confirm
        if [ "$confirm" != "y" ]; then
            echo "已取消"
            exit 0
        fi
        
        echo ""
        echo "🚀 正在启动分布式系统..."
        echo ""
        ros2 launch gs_bringup gs_distributed_launch.py
        ;;
        
    2)
        echo ""
        echo "========================================="
        echo "启动模式：仅地面站"
        echo "========================================="
        echo ""
        echo "🚀 正在启动地面站..."
        echo ""
        ros2 launch gs_bringup gs_launch.py
        ;;
        
    *)
        echo "❌ 无效选项"
        exit 1
        ;;
esac
