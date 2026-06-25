#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Launch script for gs_launch.
#
# Author: chenhangwei
# Date: 2026-01-26
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import re
import tempfile
import yaml


def generate_launch_description():
    """
    地面站统一启动文件 - 整合 GUI + Domain Bridge
    
    特性：
    1. 启动地面站 GUI 节点
    2. 可选启动 Domain Bridge（避免重复）
    3. 支持自定义配置文件
    
    使用方法：
        # 启动地面站（不启动 domain_bridge）
        ros2 launch gs_bringup gs_launch.py
        
        # 启动地面站并同时启动 domain_bridge
        ros2 launch gs_bringup gs_launch.py enable_domain_bridge:=true
        
        # 指定自定义配置
        ros2 launch gs_bringup gs_launch.py \
            enable_domain_bridge:=true \
            gs_domain_id:=99 \
            domain_bridge_config:=~/domain_bridge/domain_bridge.yaml
    """

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
        if os.path.exists(source_path):
            return source_path
        
        # 最后回退到旧的相对路径（兼容性）
        legacy_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'gs_bringup', 'config', filename))
        return legacy_path
    
    # gs 参数文件（包含 area_center）
    default_gs_params = get_config_path('gs_params.yaml')
    gs_param_arg = DeclareLaunchArgument(
        'gs_param_file',
        default_value=default_gs_params,
        description='Ground station parameters file'
    )

    # USV fleet 配置文件（用于Domain隔离架构）
    default_fleet_config = get_config_path('usv_fleet.yaml')
    fleet_config_arg = DeclareLaunchArgument(
        'fleet_config_file',
        default_value=default_fleet_config,
        description='USV fleet configuration file (for Domain isolation architecture)'
    )
    
    # 是否启用 Domain Bridge（默认 false，避免与独立启动冲突）
    enable_domain_bridge_arg = DeclareLaunchArgument(
        'enable_domain_bridge',
        default_value='false',
        description='是否启动 Domain Bridge（如已独立启动请设为 false）'
    )
    
    # 地面站 Domain ID（仅在启用 Domain Bridge 时生效）
    gs_domain_id_arg = DeclareLaunchArgument(
        'gs_domain_id',
        default_value='99',
        description='地面站的 ROS Domain ID（启用 Domain Bridge 时使用）'
    )
    
    # Domain Bridge 配置文件（智能路径）
    default_domain_bridge_config = get_config_path('domain_bridge.yaml')
    domain_bridge_config_arg = DeclareLaunchArgument(
        'domain_bridge_config',
        default_value=default_domain_bridge_config,
        description='Domain Bridge YAML 配置文件路径（启用 Domain Bridge 时使用）'
    )

    domain_bridge_namespaces_arg = DeclareLaunchArgument(
        'domain_bridge_namespaces',
        default_value='all',
        description='桥接指定命名空间，逗号分隔；all 表示自动桥接配置文件内所有 USV。留空才使用完整大配置，不推荐'
    )

    domain_bridge_wait_for_publisher_arg = DeclareLaunchArgument(
        'domain_bridge_wait_for_publisher',
        default_value='false',
        description='Domain Bridge 是否等待源 publisher 后再建桥。大配置建议 false，避免部分规则长期不实例化'
    )

    enable_apf_neighbor_relay_arg = DeclareLaunchArgument(
        'enable_apf_neighbor_relay',
        default_value='true',
        description='是否启用 APF 邻船位姿统一下发节点'
    )

    gs_param_file = LaunchConfiguration('gs_param_file')
    fleet_config_file = LaunchConfiguration('fleet_config_file')
    enable_domain_bridge = LaunchConfiguration('enable_domain_bridge')
    gs_domain_id = LaunchConfiguration('gs_domain_id')
    domain_bridge_config = LaunchConfiguration('domain_bridge_config')
    domain_bridge_namespaces = LaunchConfiguration('domain_bridge_namespaces')
    domain_bridge_wait_for_publisher = LaunchConfiguration('domain_bridge_wait_for_publisher')
    enable_apf_neighbor_relay = LaunchConfiguration('enable_apf_neighbor_relay')

    def _resolve_gs_param_file(context, *args, **kwargs):
        """Resolve gs_params.yaml path at runtime: prefer package share, fallback to workspace src."""
        try:
            # try installed package share path
            pkg_share = FindPackageShare('usv_bringup').perform(context)
            candidate = os.path.join(pkg_share, 'config', 'gs_params.yaml')
            if os.path.isfile(candidate):
                return [SetLaunchConfiguration('gs_param_file', candidate)]
        except Exception:
            pass
        # fallback: look into workspace src relative to current working dir
        try:
            wd = os.getcwd()
            candidate2 = os.path.abspath(os.path.join(wd, 'src', 'usv_bringup', 'config', 'gs_params.yaml'))
            if os.path.isfile(candidate2):
                return [SetLaunchConfiguration('gs_param_file', candidate2)]
        except Exception:
            pass
        # final fallback: leave as-is (the LaunchConfiguration default may point to non-existent file)
        return []

    def _launch_value_is_true(value):
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def _make_filtered_config(config_data, topics, namespace):
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
            f"[gs_launch] Domain Bridge 使用过滤后的配置: {filtered_path} "
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

    def _resolve_requested_namespaces(context):
        namespace_text = domain_bridge_namespaces.perform(context).strip()
        if namespace_text.lower() in ('all', '*'):
            try:
                source_config = domain_bridge_config.perform(context)
                with open(source_config, 'r', encoding='utf-8') as config_stream:
                    config_data = yaml.safe_load(config_stream) or {}
                return _discover_usv_namespaces(config_data.get('topics', {}) or {})
            except Exception:
                return []
        return [item.strip().lstrip('/') for item in namespace_text.split(',') if item.strip()]

    def _launch_domain_bridge_nodes(context, *args, **kwargs):
        if not _launch_value_is_true(enable_domain_bridge.perform(context)):
            return []

        source_config = domain_bridge_config.perform(context)
        wait_for_publisher_value = domain_bridge_wait_for_publisher.perform(context)

        with open(source_config, 'r', encoding='utf-8') as config_stream:
            config_data = yaml.safe_load(config_stream) or {}

        topics = config_data.get('topics', {}) or {}
        namespaces = _resolve_requested_namespaces(context)

        if not namespaces:
            print(f"[gs_launch] Domain Bridge bridge_namespaces 为空，使用完整配置: {source_config}")
            return [Node(
                package='domain_bridge',
                executable='domain_bridge',
                name='domain_bridge',
                output='screen',
                arguments=['--wait-for-publisher', wait_for_publisher_value, source_config],
                additional_env=additional_env,
                respawn=False,
            )]

        print(f"[gs_launch] Domain Bridge 将为 {len(namespaces)} 个 USV 分别启动 bridge: {namespaces}")
        bridge_nodes = []
        for namespace in namespaces:
            filtered_path = _make_filtered_config(config_data, topics, namespace)
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

    def _launch_apf_neighbor_relay_node(context, *args, **kwargs):
        if not _launch_value_is_true(enable_apf_neighbor_relay.perform(context)):
            return []

        usv_ids = _resolve_requested_namespaces(context)
        usv_ids_param = ','.join(usv_ids)
        print(f"[gs_launch] APF 邻船聚合将使用 {len(usv_ids)} 个 USV: {usv_ids or 'fleet_config enabled list'}")
        return [Node(
            package='gs_gui',
            executable='apf_neighbor_relay_node',
            name='apf_neighbor_relay_node',
            output='screen',
            parameters=[
                {
                    'fleet_config_file': fleet_config_file,
                    'usv_ids': usv_ids_param,
                    'source_pose_topic_suffix': 'local_position/pose_from_gps',
                    'source_velocity_topic_suffix': 'local_position/velocity_local',
                    'apf_neighbors_topic_suffix': 'apf/neighbors',
                    'publish_rate': 2.0,
                    'neighbor_timeout': 0.8,
                    'max_neighbor_distance': 12.0,
                    'prefer_velocity_topic': True,
                }
            ],
        )]

    # =============================================================================
    # 地面站 GUI 节点
    # =============================================================================
    main_gui_app = Node(
        package='gs_gui',
        executable='main_gui_app',
        name='main_gui_app',
        output='screen',
        parameters=[
            gs_param_file,  # 从文件加载参数
            {
                'use_sim_time': False,
                'fleet_config_file': default_fleet_config,  # 直接使用字符串路径
                'active_usv_ids': domain_bridge_namespaces,
            }
        ]
    )

    apf_neighbor_relay_node = OpaqueFunction(function=_launch_apf_neighbor_relay_node)

    # =============================================================================
    # Domain Bridge 节点（条件启动）
    # =============================================================================
    # 设置地面站 Domain ID（仅在启用 Domain Bridge 时生效）
    set_domain_id = SetEnvironmentVariable(
        'ROS_DOMAIN_ID', 
        gs_domain_id,
        condition=IfCondition(enable_domain_bridge)
    )
    
    # 获取 common_interfaces 包路径，确保 domain_bridge 能找到自定义消息
    try:
        common_interfaces_path = get_package_share_directory('common_interfaces')
        additional_env = {
            'LD_LIBRARY_PATH': os.path.join(os.path.dirname(common_interfaces_path), 'lib') + ':' + os.environ.get('LD_LIBRARY_PATH', '')
        }
    except Exception:
        additional_env = {}
    
    # =============================================================================
    # 启动描述
    # =============================================================================
    return LaunchDescription([
        # 参数声明
        gs_param_arg,
        fleet_config_arg,
        enable_domain_bridge_arg,
        gs_domain_id_arg,
        domain_bridge_config_arg,
        domain_bridge_namespaces_arg,
        domain_bridge_wait_for_publisher_arg,
        enable_apf_neighbor_relay_arg,
        
        # 条件环境变量（仅在启用 Domain Bridge 时设置）
        set_domain_id,
        
        # 节点
        main_gui_app,
        apf_neighbor_relay_node,
        OpaqueFunction(function=_launch_domain_bridge_nodes),
    ])