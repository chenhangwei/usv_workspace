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
3. 支持设置 FastDDS Profile 文件
4. 默认按 USV 命名空间拆分启动多个小 bridge 进程，避免单进程 507 条规则导致转发不稳定
5. 可独立启动或集成到其他启动文件

使用方法：
    # 重启 bridge 前清理旧进程和 ROS daemon（推荐在桥异常或重新配置后执行）
    pkill -f domain_bridge
    ros2 daemon stop
    ros2 daemon start

    # 推荐：启动配置文件中列出的全部 USV 桥
    # 默认 bridge_namespaces:=all，会自动识别 domain_bridge.yaml 中的 usv_01 ... usv_13
    # 每艘 USV 会生成一个临时小配置并启动一个独立 domain_bridge 进程
    ros2 launch gs_bringup domain_bridge.launch.py

    # 只启动部分 USV 桥（现场只开了部分设备时推荐）
    ros2 launch gs_bringup domain_bridge.launch.py bridge_namespaces:=usv_04,usv_05

    # 显式启动全部 USV 桥（等价于默认启动）
    ros2 launch gs_bringup domain_bridge.launch.py bridge_namespaces:=all
    
    # 指定配置文件
    ros2 launch gs_bringup domain_bridge.launch.py config_file:=/path/to/config.yaml

    # 使用其他地面站 Domain ID 或 FastDDS 配置
    ros2 launch gs_bringup domain_bridge.launch.py gs_domain_id:=99 fastdds_profile:=/home/chenhangwei/fastdds_gs.xml
    
    # 后台保活运行（推荐 screen/tmux，关闭普通 terminal 会停止 bridge）
    screen -S domain_bridge
    ros2 launch gs_bringup domain_bridge.launch.py

    # 按 Ctrl+A 再按 D 可分离 screen 会话；恢复会话：
    screen -r domain_bridge

注意：
    - bridge_namespaces 不要传空。传空会使用完整大配置启动单个 bridge 进程，仅保留兼容，不推荐。
    - 需要停止桥时，在对应 terminal/screen 中按 Ctrl+C，launch 会一起关闭它启动的子进程。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import re
import tempfile
import yaml


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

    bridge_namespaces_arg = DeclareLaunchArgument(
        'bridge_namespaces',
        default_value='all',
        description='桥接指定命名空间，逗号分隔；all 表示自动桥接配置文件内所有 USV。留空才使用完整大配置，不推荐'
    )

    wait_for_publisher_arg = DeclareLaunchArgument(
        'wait_for_publisher',
        default_value='false',
        description='是否等待源 Domain publisher 出现后再创建桥接。大配置建议 false，避免部分规则长期不实例化'
    )

    # FastDDS 配置文件路径（地面站端）
    fastdds_profile_arg = DeclareLaunchArgument(
        'fastdds_profile',
        default_value='/home/chenhangwei/fastdds_gs.xml',
        description='FastDDS XML 配置文件路径'
    )
    
    gs_domain_id = LaunchConfiguration('gs_domain_id')
    config_file = LaunchConfiguration('config_file')
    bridge_namespaces = LaunchConfiguration('bridge_namespaces')
    wait_for_publisher = LaunchConfiguration('wait_for_publisher')
    fastdds_profile = LaunchConfiguration('fastdds_profile')

    def _make_filtered_config(source_config, config_data, topics, namespace):
        selected_topics = {
            topic_name: topic_config
            for topic_name, topic_config in topics.items()
            if topic_name == namespace or topic_name.startswith(f'{namespace}/')
        }

        filtered_config = dict(config_data)
        filtered_config['name'] = f"{config_data.get('name', 'domain_bridge')}_{namespace}"
        filtered_config['topics'] = selected_topics

        temp_dir = os.path.join(tempfile.gettempdir(), 'usv_domain_bridge')
        os.makedirs(temp_dir, exist_ok=True)
        filtered_path = os.path.join(temp_dir, f'domain_bridge_{namespace}.yaml')
        with open(filtered_path, 'w', encoding='utf-8') as filtered_stream:
            yaml.safe_dump(filtered_config, filtered_stream, sort_keys=False, allow_unicode=True)

        print(
            f"[domain_bridge.launch] 使用过滤后的配置: {filtered_path} "
            f"(namespace={namespace}, topics={len(selected_topics)}/{len(topics)})"
        )
        return filtered_path

    def _discover_usv_namespaces(topics):
        namespaces = {
            topic_name.split('/', 1)[0]
            for topic_name in topics
            if re.match(r'^usv_\d+$', topic_name.split('/', 1)[0])
        }
        return sorted(namespaces)

    def _launch_domain_bridge_nodes(context, *args, **kwargs):
        source_config = config_file.perform(context)
        namespace_text = bridge_namespaces.perform(context).strip()
        wait_for_publisher_value = wait_for_publisher.perform(context)

        with open(source_config, 'r', encoding='utf-8') as config_stream:
            config_data = yaml.safe_load(config_stream) or {}

        topics = config_data.get('topics', {}) or {}
        if namespace_text.lower() in ('all', '*'):
            namespaces = _discover_usv_namespaces(topics)
        else:
            namespaces = [item.strip().lstrip('/') for item in namespace_text.split(',') if item.strip()]

        if not namespaces:
            print(f"[domain_bridge.launch] bridge_namespaces 为空，使用完整配置: {source_config}")
            return [Node(
                package='domain_bridge',
                executable='domain_bridge',
                name='domain_bridge',
                output='screen',
                arguments=['--wait-for-publisher', wait_for_publisher_value, source_config],
                additional_env=additional_env,
                respawn=False,
            )]

        print(f"[domain_bridge.launch] 将为 {len(namespaces)} 个 USV 分别启动 bridge: {namespaces}")
        bridge_nodes = []
        for namespace in namespaces:
            filtered_path = _make_filtered_config(source_config, config_data, topics, namespace)
            bridge_nodes.append(Node(
                package='domain_bridge',
                executable='domain_bridge',
                name=f'domain_bridge_{namespace}',
                output='screen',
                arguments=['--wait-for-publisher', wait_for_publisher_value, filtered_path],
                additional_env=additional_env,
                respawn=False,
            ))
        return bridge_nodes
    
    # =============================================================================
    # 环境变量设置
    # =============================================================================
    
    # 设置地面站 Domain ID
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', gs_domain_id)

    # 显式指定 FastDDS Profile，避免仅依赖外部 shell 环境
    set_fastdds_profile = SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', fastdds_profile)
    
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
        bridge_namespaces_arg,
        wait_for_publisher_arg,
        fastdds_profile_arg,
        
        # 环境变量
        set_domain_id,
        set_fastdds_profile,
        
        # 节点
        OpaqueFunction(function=_launch_domain_bridge_nodes),
    ])
