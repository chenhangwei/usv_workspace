"""
地面站启动文件 - PX4 uXRCE-DDS 版本（40+ 集群支持）

该启动文件用于地面站，通过 Zenoh 接收所有分组 USV 的状态。

特性：
1. 集群管理节点处理所有 USV 状态聚合
2. 动态发现 USV
3. Zenoh Bridge 接收跨组数据
4. 支持编队命令分发
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成启动描述"""
    
    # =========================================================================
    # 参数声明
    # =========================================================================
    use_zenoh_arg = DeclareLaunchArgument(
        'use_zenoh',
        default_value='true',
        description='是否使用 Zenoh Bridge 接收跨组数据'
    )
    
    router_ip_arg = DeclareLaunchArgument(
        'router_ip',
        default_value='192.168.68.50',
        description='Zenoh Router IP 地址'
    )
    
    domain_id_arg = DeclareLaunchArgument(
        'domain_id',
        default_value='99',
        description='地面站 ROS Domain ID'
    )
    
    discovery_enabled_arg = DeclareLaunchArgument(
        'discovery_enabled',
        default_value='true',
        description='是否启用自动发现 USV'
    )
    
    start_gui_arg = DeclareLaunchArgument(
        'start_gui',
        default_value='true',
        description='是否启动 GUI'
    )
    
    # 获取参数
    use_zenoh = LaunchConfiguration('use_zenoh')
    router_ip = LaunchConfiguration('router_ip')
    domain_id = LaunchConfiguration('domain_id')
    discovery_enabled = LaunchConfiguration('discovery_enabled')
    start_gui = LaunchConfiguration('start_gui')
    
    # 参数文件
    gs_param_file = PathJoinSubstitution([
        FindPackageShare('gs_bringup'),
        'config',
        'gs_params.yaml'
    ])
    
    zenoh_config = PathJoinSubstitution([
        FindPackageShare('gs_bringup'),
        'config',
        'zenoh_gs_config.json5'
    ])
    
    # =========================================================================
    # 设置 Domain ID 环境变量
    # =========================================================================
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=domain_id
    )
    
    # =========================================================================
    # 启动信息
    # =========================================================================
    startup_info = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            '🖥️ 启动 PX4 uXRCE-DDS 地面站\n',
            '=' * 60, '\n',
            'Domain ID: ', domain_id, '\n',
            'Zenoh: ', use_zenoh, '\n',
            'Router IP: ', router_ip, '\n',
            '自动发现: ', discovery_enabled, '\n',
            '=' * 60,
        ]
    )
    
    # =========================================================================
    # Zenoh Bridge（Router 模式 - 地面站作为中心节点）
    # =========================================================================
    zenoh_bridge = ExecuteProcess(
        condition=IfCondition(use_zenoh),
        cmd=[
            'zenoh-bridge-ros2dds',
            '-c', zenoh_config,
            '-d', domain_id,
            # Router 模式：监听 0.0.0.0:7447，USV 连接到此
        ],
        output='screen',
        name='zenoh_bridge'
    )
    
    # =========================================================================
    # 集群管理节点
    # =========================================================================
    cluster_manager_node = Node(
        package='gs_gui',
        executable='cluster_manager_node',
        name='cluster_manager_node',
        output='screen',
        parameters=[
            gs_param_file,
            {
                'usv_discovery_enabled': True,
                'usv_discovery_timeout': 10.0,
                'heartbeat_timeout': 5.0,
                'status_publish_rate': 2.0,
                'max_usv_count': 50,
            },
        ],
    )
    
    # =========================================================================
    # GUI 应用（内部创建 GroundStationNode）
    # =========================================================================
    # 注：main_gui_app 内部已创建 GroundStationNode，无需单独启动
    main_gui_node = Node(
        condition=IfCondition(start_gui),
        package='gs_gui',
        executable='main_gui_app',
        name='main_gui_app',
        output='screen',
        parameters=[gs_param_file],
    )
    
    # =========================================================================
    # 延迟启动（等待 Zenoh 连接）
    # =========================================================================
    delayed_nodes = TimerAction(
        period=3.0,
        actions=[
            cluster_manager_node,
            main_gui_node,
        ]
    )
    
    return LaunchDescription([
        # 参数
        use_zenoh_arg,
        router_ip_arg,
        domain_id_arg,
        discovery_enabled_arg,
        start_gui_arg,
        
        # 环境设置
        set_domain_id,
        
        # 启动信息
        startup_info,
        
        # Zenoh Bridge
        zenoh_bridge,
        
        # 节点
        delayed_nodes,
    ])
