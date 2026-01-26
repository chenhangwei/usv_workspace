#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Launch script for domain_bridge.launch.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
Domain Bridge 启动文件 - 地面站端
用于实现跨 ROS Domain 通信，连接不同 Domain ID 的 USV 和地面站

特性：
1. 自动设置地面站 Domain ID
2. 支持自定义配置文件路径
3. 可独立启动或集成到其他启动文件

使用方法：
    # 使用默认配置
    ros2 launch gs_bringup domain_bridge.launch.py
    
    # 指定配置文件
    ros2 launch gs_bringup domain_bridge.launch.py config_file:=/path/to/config.yaml
    
    # 后台运行（推荐）
    screen -S domain_bridge
    ros2 launch gs_bringup domain_bridge.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    生成 Domain Bridge 启动描述
    
    配置说明：
    - ROS_DOMAIN_ID: 地面站的 Domain ID（默认 99）
    - config_file: domain_bridge 配置文件路径
    
    ⚠️ 防止重复启动机制：
    - 检查锁文件 /tmp/domain_bridge.lock
    - 检查是否已有 domain_bridge 进程运行
    
    注意: 单例检查已移至 domain_bridge.sh 脚本中处理
    launch文件不再进行进程检查,避免检测到自己启动的进程
    """
    import subprocess
    import sys
    
    # 单例检查由 domain_bridge.sh 脚本负责,此处不再检查
    # 原因: launch文件启动后会检测到自己启动的进程,导致误报
    
    # =============================================================================
    # 参数声明
    # =============================================================================
    
    # =============================================================================
    # 智能路径解析：自动选择源码目录或安装目录
    # =============================================================================
    def get_config_path(filename):
        """
        智能查找配置文件路径，优先级：
        1. 安装目录（install/share/gs_bringup/config/）
        2. 源码目录（src/gs_bringup/config/）
        """
        # 尝试从安装的包中获取
        try:
            pkg_share = get_package_share_directory('gs_bringup')
            installed_path = os.path.join(pkg_share, 'config', filename)
            if os.path.exists(installed_path):
                return installed_path
        except Exception:
            pass
        
        # 回退到源码目录（相对于当前 launch 文件）
        source_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config', filename))
        return source_path
    
    # 地面站 Domain ID
    gs_domain_id_arg = DeclareLaunchArgument(
        'gs_domain_id',
        default_value='99',
        description='地面站的 ROS Domain ID'
    )
    
    # Domain Bridge 配置文件路径（智能路径）
    default_config = get_config_path('domain_bridge.yaml')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Domain Bridge YAML 配置文件路径'
    )
    
    gs_domain_id = LaunchConfiguration('gs_domain_id')
    config_file = LaunchConfiguration('config_file')
    
    # =============================================================================
    # 环境变量设置
    # =============================================================================
    
    # 设置地面站 Domain ID
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', gs_domain_id)
    
    # =============================================================================
    # Domain Bridge 节点
    # =============================================================================
    
    # 获取 common_interfaces 包的路径，确保 domain_bridge 能找到自定义消息类型
    try:
        common_interfaces_path = get_package_share_directory('common_interfaces')
        # 设置 LD_LIBRARY_PATH 以包含 common_interfaces 的库路径
        additional_env = {
            'LD_LIBRARY_PATH': os.path.join(os.path.dirname(common_interfaces_path), 'lib') + ':' + os.environ.get('LD_LIBRARY_PATH', '')
        }
    except Exception:
        additional_env = {}
    
    domain_bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_bridge',
        output='screen',
        arguments=[config_file],
        additional_env=additional_env,
        # ⚠️ 禁用 respawn - 防止意外重启导致多实例
        respawn=False,
        # on_exit=lambda: os.remove(lock_file) if os.path.exists(lock_file) else None,
    )
    
    # 锁文件管理已移至 domain_bridge.sh 脚本中
    # launch文件不再管理锁文件,避免冲突
    import atexit
    
    # =============================================================================
    # 启动描述
    # =============================================================================
    
    return LaunchDescription([
        # 参数
        gs_domain_id_arg,
        config_file_arg,
        
        # 环境变量
        set_domain_id,
        
        # 节点
        domain_bridge_node,
    ])
