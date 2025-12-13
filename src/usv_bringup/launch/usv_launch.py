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
        default_value='/dev/ttyACM0',
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
        description='是否使用 Zenoh Bridge 进行跨组通信（与地面站通信不需要）'
    )
    
    router_ip_arg = DeclareLaunchArgument(
        'router_ip',
        default_value='192.168.68.50',
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
    
    # 以太网连接参数
    use_ethernet_arg = DeclareLaunchArgument(
        'use_ethernet',
        default_value='true',
        description='是否使用以太网连接飞控（推荐）'
    )
    
    agent_port_arg = DeclareLaunchArgument(
        'agent_port',
        default_value='8888',
        description='以太网 UDP 端口（飞控 uXRCE-DDS 端口）'
    )
    
    # UWB 定位参数
    use_uwb_arg = DeclareLaunchArgument(
        'use_uwb',
        default_value='true',
        description='是否启用 UWB 室内定位'
    )
    
    uwb_port_arg = DeclareLaunchArgument(
        'uwb_port',
        default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5787006321-if00',
        description='UWB 串口路径'
    )

    # =========================================================================
    # 平台模式/姿态控制参数（2D/3D）
    # =========================================================================
    platform_mode_arg = DeclareLaunchArgument(
        'platform_mode',
        default_value='3d',
        description="平台模式：'3d'（无人球/仿生鱼）或 '2d'（差速车/无人鸭）。2d 将忽略 roll/pitch，仅保留位置+yaw"
    )

    use_setpoint_6dof_arg = DeclareLaunchArgument(
        'use_setpoint_6dof',
        default_value='false',
        description='是否发布 fmu/in/trajectory_setpoint6dof（位置+姿态四元数）。默认 false 以保持现有行为'
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
    use_ethernet = LaunchConfiguration('use_ethernet')
    agent_port = LaunchConfiguration('agent_port')
    use_uwb = LaunchConfiguration('use_uwb')
    uwb_port = LaunchConfiguration('uwb_port')
    platform_mode = LaunchConfiguration('platform_mode')
    use_setpoint_6dof = LaunchConfiguration('use_setpoint_6dof')
    
    # =========================================================================
    # 分组 Domain ID 映射
    # A -> 11, B -> 12, C -> 13, D -> 14, E -> 15, F -> 16
    # =========================================================================
    domain_id = PythonExpression([
        "{'A': 11, 'B': 12, 'C': 13, 'D': 14, 'E': 15, 'F': 16}.get('", 
        group_id, 
        "', 11)"
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
            'Domain ID: ', domain_id, '\n',
            '平台模式: ', platform_mode, '\n',
            '6DoF setpoint: ', use_setpoint_6dof, '\n',
            '=' * 60,
        ]
    )
    
    # 以太网模式启动信息
    ethernet_info = LogInfo(
        condition=IfCondition(use_ethernet),
        msg=[
            '📡 以太网配置:\n',
            '   Agent端口: ', agent_port, '\n',
            '   协议: UDP4\n',
            '   (飞控IP在飞控端配置)\n',
        ]
    )
    
    # 串口模式启动信息
    serial_info = LogInfo(
        condition=IfCondition(PythonExpression([
            "'", use_ethernet, "'.lower() == 'false' and '", use_simulation, "'.lower() == 'false'"
        ])),
        msg=[
            '🔌 串口配置:\n',
            '   设备: ', serial_port, '\n',
            '   波特率: ', baudrate, '\n',
        ]
    )
    
    # =========================================================================
    # Micro XRCE-DDS Agent（串口模式）
    # =========================================================================
    # 串口模式条件：非以太网 且 非仿真
    use_serial_condition = PythonExpression([
        "'", use_ethernet, "'.lower() == 'false' and '", use_simulation, "'.lower() == 'false'"
    ])
    
    micro_xrce_agent_serial = ExecuteProcess(
        condition=IfCondition(use_serial_condition),
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
    # Micro XRCE-DDS Agent（以太网模式 - 推荐）
    # 飞控通过网线连接树莓派，使用 UDP 通信
    # =========================================================================
    micro_xrce_agent_ethernet = ExecuteProcess(
        condition=IfCondition(use_ethernet),
        cmd=[
            'MicroXRCEAgent', 'udp4',
            '-p', agent_port,
            '-n', namespace,
        ],
        output='screen',
        name='micro_xrce_agent'
    )
    
    # =========================================================================
    # Zenoh Bridge（Peer 模式 - 支持任意启动顺序）
    # =========================================================================
    zenoh_bridge = ExecuteProcess(
        condition=IfCondition(use_zenoh),
        cmd=[
            'zenoh-bridge-ros2dds',
            '-l', 'tcp/0.0.0.0:7448',  # 监听端口（用于其他 USV 可能的连接）
            '-e', ['tcp/', router_ip, ':7447'],  # 连接到地面站
            '-c', zenoh_config,
            '-d', domain_id,
            '--no-multicast-scouting',  # 禁用组播探测（跨网络不需要）
            'peer',  # Peer 模式（位置参数，放在最后）
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
        parameters=[
            param_file,
            {
                'platform_mode': platform_mode,
                'use_setpoint_6dof': use_setpoint_6dof,
            },
        ],
        remappings=[
            # PX4 话题映射到命名空间 (注意：PX4 v1.15+ 使用 vehicle_status_v1)
            ('/fmu/out/vehicle_status_v1', 'fmu/out/vehicle_status_v1'),
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/in/trajectory_setpoint', 'fmu/in/trajectory_setpoint'),
            ('/fmu/in/trajectory_setpoint6dof', 'fmu/in/trajectory_setpoint6dof'),
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
            # 注意：PX4 v1.15+ 使用 vehicle_status_v1
            ('/fmu/out/vehicle_status_v1', 'fmu/out/vehicle_status_v1'),
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/in/vehicle_command', 'fmu/in/vehicle_command'),
            ('/fmu/in/offboard_control_mode', 'fmu/in/offboard_control_mode'),
            ('/fmu/in/trajectory_setpoint', 'fmu/in/trajectory_setpoint'),
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
            # 注意：PX4 v1.15+ 使用 vehicle_status_v1
            ('/fmu/out/vehicle_status_v1', 'fmu/out/vehicle_status_v1'),
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/out/battery_status', 'fmu/out/battery_status'),
            ('/fmu/out/vehicle_attitude', 'fmu/out/vehicle_attitude'),
            # 失控保护标志（PX4 默认发布，包含预检信息）
            ('/fmu/out/failsafe_flags', 'fmu/out/failsafe_flags'),
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
            {'coordinate_system': 'ENU'},  # 室内 UWB 使用 ENU 坐标系
        ],
        remappings=[
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/in/trajectory_setpoint', 'fmu/in/trajectory_setpoint'),
            ('/fmu/in/offboard_control_mode', 'fmu/in/offboard_control_mode'),
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
            # 室内 UWB 场景使用虚拟坐标
            {'fixed_lat': 0.0},
            {'fixed_lon': 0.0},
            {'fixed_alt': 0.0},
        ],
        remappings=[
            ('/fmu/out/vehicle_status', 'fmu/out/vehicle_status'),
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/in/vehicle_command', 'fmu/in/vehicle_command'),
        ],
    )
    
    # =========================================================================
    # UWB 定位节点
    # =========================================================================
    usv_uwb_node = Node(
        package='usv_drivers',
        executable='usv_uwb_node',
        name='usv_uwb_node',
        namespace=namespace,
        output='screen',
        parameters=[
            {'uwb_port': uwb_port},
            {'uwb_baudrate': 115200},
            {'uwb_timeout': 1.0},
        ],
        remappings=[
            ('/fmu/in/vehicle_visual_odometry', 'fmu/in/vehicle_visual_odometry'),
        ],
        condition=IfCondition(use_uwb),
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
            usv_uwb_node,
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
        use_ethernet_arg,
        agent_port_arg,
        use_uwb_arg,
        uwb_port_arg,
        platform_mode_arg,
        use_setpoint_6dof_arg,
        
        # 环境设置
        set_domain_id,
        
        # 启动信息
        startup_info,
        ethernet_info,
        serial_info,
        
        # Agent（三选一：以太网/串口/仿真）
        micro_xrce_agent_ethernet,  # 以太网模式
        micro_xrce_agent_serial,    # 串口模式
        micro_xrce_agent_udp,       # 仿真模式
        
        # Zenoh Bridge
        zenoh_bridge,
        
        # 控制节点
        delayed_nodes,
    ])
