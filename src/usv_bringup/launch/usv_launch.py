"""
集群无人球启动文件 - PX4 uXRCE-DDS 版本

支持 40+ 节点的大规模集群，使用 Zenoh 进行跨组通信。

特性：
1. 使用 Micro XRCE-DDS Agent 替代 MAVROS
2. 命名空间隔离各 USV 话题
3. 分组管理（每组 8 台）
4. 可选 Zenoh Bridge 跨组通信
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    TimerAction,
    GroupAction,
    SetEnvironmentVariable,
    LogInfo,
)
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution,
    PythonExpression,
    EnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成启动描述"""
    
    # =========================================================================
    # 参数声明
    # =========================================================================
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='usv_01',
        description='无人球命名空间 (usv_01, usv_02, ...)'
    )
    
    group_id_arg = DeclareLaunchArgument(
        'group_id',
        default_value='A',
        description='编队分组 (A, B, C, D, E, F)'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='飞控串口设备'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='921600',
        description='串口波特率'
    )
    
    use_zenoh_arg = DeclareLaunchArgument(
        'use_zenoh',
        default_value='true',
        description='是否使用 Zenoh Bridge 进行跨组通信'
    )
    
    router_ip_arg = DeclareLaunchArgument(
        'router_ip',
        default_value='192.168.1.100',
        description='Zenoh Router IP 地址'
    )
    
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='是否使用仿真模式（UDP 连接而非串口）'
    )
    
    simulation_port_arg = DeclareLaunchArgument(
        'simulation_port',
        default_value='8888',
        description='仿真模式 UDP 端口'
    )
    
    # 获取参数
    namespace = LaunchConfiguration('namespace')
    group_id = LaunchConfiguration('group_id')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    use_zenoh = LaunchConfiguration('use_zenoh')
    router_ip = LaunchConfiguration('router_ip')
    use_simulation = LaunchConfiguration('use_simulation')
    simulation_port = LaunchConfiguration('simulation_port')
    
    # =========================================================================
    # 分组 Domain ID 映射
    # A -> 10, B -> 20, C -> 30, D -> 40, E -> 50, F -> 60
    # =========================================================================
    domain_id = PythonExpression([
        "{'A': 10, 'B': 20, 'C': 30, 'D': 40, 'E': 50, 'F': 60}.get('", 
        group_id, 
        "', 10)"
    ])
    
    # 参数文件
    param_file = PathJoinSubstitution([
        FindPackageShare('usv_bringup'),
        'config',
        'usv_params.yaml'
    ])
    
    zenoh_config = PathJoinSubstitution([
        FindPackageShare('usv_bringup'),
        'config',
        'zenoh_usv_config.json5'
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
            '🚀 启动 PX4 uXRCE-DDS 无人球节点\n',
            '=' * 60, '\n',
            '命名空间: ', namespace, '\n',
            '分组: ', group_id, '\n',
            '串口: ', serial_port, '\n',
            '波特率: ', baudrate, '\n',
            'Domain ID: ', domain_id, '\n',
            '=' * 60,
        ]
    )
    
    # =========================================================================
    # Micro XRCE-DDS Agent（串口模式）
    # =========================================================================
    micro_xrce_agent_serial = ExecuteProcess(
        condition=UnlessCondition(use_simulation),
        cmd=[
            'MicroXRCEAgent', 'serial',
            '--dev', serial_port,
            '-b', baudrate,
            '-n', namespace,
        ],
        output='screen',
        name='micro_xrce_agent'
    )
    
    # =========================================================================
    # Micro XRCE-DDS Agent（UDP 仿真模式）
    # =========================================================================
    micro_xrce_agent_udp = ExecuteProcess(
        condition=IfCondition(use_simulation),
        cmd=[
            'MicroXRCEAgent', 'udp4',
            '-p', simulation_port,
            '-n', namespace,
        ],
        output='screen',
        name='micro_xrce_agent'
    )
    
    # =========================================================================
    # Zenoh Bridge（可选，用于跨组通信）
    # =========================================================================
    zenoh_bridge = ExecuteProcess(
        condition=IfCondition(use_zenoh),
        cmd=[
            'zenoh-bridge-ros2dds',
            '-c', zenoh_config,
            '-e', ['tcp/', router_ip, ':7447'],
            '-d', domain_id,
        ],
        output='screen',
        name='zenoh_bridge'
    )
    
    # =========================================================================
    # PX4 控制节点
    # =========================================================================
    usv_control_node = Node(
        package='usv_control',
        executable='usv_control_node',
        name='usv_control_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file],
        remappings=[
            # PX4 话题映射到命名空间
            ('/fmu/out/vehicle_status', 'fmu/out/vehicle_status'),
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/in/trajectory_setpoint', 'fmu/in/trajectory_setpoint'),
            ('/fmu/in/vehicle_command', 'fmu/in/vehicle_command'),
            ('/fmu/in/offboard_control_mode', 'fmu/in/offboard_control_mode'),
        ],
    )
    
    usv_command_node = Node(
        package='usv_control',
        executable='usv_command_node',
        name='usv_command_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file],
        remappings=[
            ('/fmu/out/vehicle_status', 'fmu/out/vehicle_status'),
            ('/fmu/in/vehicle_command', 'fmu/in/vehicle_command'),
            ('/fmu/in/offboard_control_mode', 'fmu/in/offboard_control_mode'),
        ],
    )
    
    usv_status_node = Node(
        package='usv_comm',
        executable='usv_status_node',
        name='usv_status_node',
        namespace=namespace,
        output='screen',
        parameters=[
            param_file,
            {'publish_rate': 5.0},  # 降低发布频率以减少带宽
        ],
        remappings=[
            ('/fmu/out/vehicle_status', 'fmu/out/vehicle_status'),
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/out/battery_status', 'fmu/out/battery_status'),
            ('/fmu/out/vehicle_attitude', 'fmu/out/vehicle_attitude'),
        ],
    )
    
    # =========================================================================
    # 避障节点 - PX4 版本
    # =========================================================================
    usv_avoidance_node = Node(
        package='usv_control',
        executable='usv_avoidance_node',
        name='usv_avoidance_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file],
        remappings=[
            ('/fmu/out/vehicle_status', 'fmu/out/vehicle_status'),
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/in/trajectory_setpoint', 'fmu/in/trajectory_setpoint'),
            ('/fmu/in/offboard_control_mode', 'fmu/in/offboard_control_mode'),
        ],
    )
    
    # =========================================================================
    # 坐标转换节点 - PX4 版本
    # =========================================================================
    coord_transform_node = Node(
        package='usv_control',
        executable='coord_transform_node',
        name='coord_transform_node',
        namespace=namespace,
        output='screen',
        parameters=[
            param_file,
            {'mode': 'local'},  # 使用本地坐标直传模式
            {'coordinate_system': 'ENU'},
        ],
        remappings=[
            ('/fmu/out/vehicle_global_position', 'fmu/out/vehicle_global_position'),
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/in/trajectory_setpoint', 'fmu/in/trajectory_setpoint'),
            ('/fmu/in/offboard_control_mode', 'fmu/in/offboard_control_mode'),
            ('/fmu/in/vehicle_command', 'fmu/in/vehicle_command'),
        ],
    )
    
    # =========================================================================
    # 自动设置 Home 点节点 - PX4 版本
    # =========================================================================
    auto_set_home_node = Node(
        package='usv_comm',
        executable='auto_set_home_node',
        name='auto_set_home_node',
        namespace=namespace,
        output='screen',
        parameters=[
            param_file,
            {'set_delay_sec': 5.0},
            {'use_current_gps': False},  # 使用固定坐标作为原点
            {'wait_for_gps': False},     # 不等待 GPS（室内/UWB 场景）
        ],
        remappings=[
            ('/fmu/out/vehicle_status', 'fmu/out/vehicle_status'),
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/out/vehicle_global_position', 'fmu/out/vehicle_global_position'),
            ('/fmu/out/vehicle_gps_position', 'fmu/out/vehicle_gps_position'),
            ('/fmu/in/vehicle_command', 'fmu/in/vehicle_command'),
        ],
    )
    
    # =========================================================================
    # 延迟启动控制节点（等待 Agent 连接）
    # =========================================================================
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            usv_control_node,
            usv_command_node,
            usv_status_node,
            usv_avoidance_node,
            coord_transform_node,
            auto_set_home_node,
        ]
    )
    
    return LaunchDescription([
        # 参数
        namespace_arg,
        group_id_arg,
        serial_port_arg,
        baudrate_arg,
        use_zenoh_arg,
        router_ip_arg,
        use_simulation_arg,
        simulation_port_arg,
        
        # 环境设置
        set_domain_id,
        
        # 启动信息
        startup_info,
        
        # Agent
        micro_xrce_agent_serial,
        micro_xrce_agent_udp,
        
        # Zenoh Bridge
        zenoh_bridge,
        
        # 控制节点
        delayed_nodes,
    ])
