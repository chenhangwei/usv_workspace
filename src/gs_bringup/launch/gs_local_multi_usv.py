"""
本地模拟测试 Launch 文件
用于在单台电脑上测试分布式启动功能（不需要实际的 USV 硬件）

使用方法：
    ros2 launch gs_bringup gs_local_multi_usv.py
    
    # 指定启动的 USV 数量
    ros2 launch gs_bringup gs_local_multi_usv.py num_usvs:=3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    生成本地多 USV 启动描述（用于测试）
    """
    
    # =============================================================================
    # 参数声明
    # =============================================================================
    
    num_usvs_arg = DeclareLaunchArgument(
        'num_usvs',
        default_value='3',
        description='要启动的 USV 数量'
    )
    
    gs_param_arg = DeclareLaunchArgument(
        'gs_param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('gs_bringup'),
            'config',
            'gs_params.yaml'
        ]),
        description='地面站参数文件'
    )
    
    num_usvs = LaunchConfiguration('num_usvs')
    gs_param_file = LaunchConfiguration('gs_param_file')
    
    # =============================================================================
    # 地面站节点
    # =============================================================================
    
    ground_station = Node(
        package='gs_gui',
        executable='main_gui_app',
        name='main_gui_app',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            gs_param_file
        ]
    )
    
    # =============================================================================
    # USV 节点（本地模拟，无硬件）
    # =============================================================================
    
    # 注意：这里手动创建 3 个 USV（可以根据需要修改）
    # 在实际分布式部署中，这些节点会在各自的机载计算机上启动
    
    usv_launch_file = PathJoinSubstitution([
        FindPackageShare('usv_bringup'),
        'launch',
        'usv_launch.py'
    ])
    
    usv_launches = []
    
    # USV 01
    usv_01 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(usv_launch_file),
        launch_arguments={
            'namespace': 'usv_01',
            'fcu_url': 'udp://:14540@localhost:14557',  # SITL 仿真地址
            'tgt_system': '1',
        }.items()
    )
    
    # USV 02
    usv_02 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(usv_launch_file),
        launch_arguments={
            'namespace': 'usv_02',
            'fcu_url': 'udp://:14541@localhost:14567',
            'tgt_system': '2',
        }.items()
    )
    
    # USV 03
    usv_03 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(usv_launch_file),
        launch_arguments={
            'namespace': 'usv_03',
            'fcu_url': 'udp://:14542@localhost:14577',
            'tgt_system': '3',
        }.items()
    )
    
    # =============================================================================
    # 启动描述
    # =============================================================================
    
    return LaunchDescription([
        # 参数
        num_usvs_arg,
        gs_param_arg,
        
        # 地面站
        ground_station,
        
        # USV 节点（本地测试）
        # 注意：在实际部署中，这些会通过 SSH 在远程机器启动
        usv_01,
        usv_02,
        usv_03,
    ])
