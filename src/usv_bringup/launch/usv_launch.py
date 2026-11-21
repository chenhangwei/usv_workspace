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
    OpaqueFunction,
    TimerAction,
    GroupAction,
    SetEnvironmentVariable,
    LogInfo,
    Shutdown,
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
        description='编队分组 (A..F, 或 GS=99 用于本地仿真直连地面站)'
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
        default_value='auto',
        description="平台模式：'auto'（按 vehicle_type 自动推断：usv/usv_diff 为 2d，其余为 3d）、'3d' 或 '2d'"
    )

    use_setpoint_6dof_arg = DeclareLaunchArgument(
        'use_setpoint_6dof',
        default_value='false',
        description='是否发布 fmu/in/trajectory_setpoint6dof（位置+姿态四元数）。默认 false 以保持现有行为'
    )

    vehicle_type_arg = DeclareLaunchArgument(
        'vehicle_type',
        default_value='flying_sphere',
        description='载具类型：bionic_airship / bionic_dolphin / flying_sphere / usv'
    )

    reach_xy_threshold_arg = DeclareLaunchArgument(
        'reach_xy_threshold',
        default_value='-1.0',
        description='水平到达阈值(m)，<=0 则按 vehicle_type 自动选择默认阈值'
    )

    reach_z_threshold_arg = DeclareLaunchArgument(
        'reach_z_threshold',
        default_value='-1.0',
        description='高度到达阈值(m)，<=0 则按 vehicle_type 自动选择默认阈值'
    )

    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='-1.0',
        description='3D起飞预高度(m)，<=0 则按 vehicle_type 自动选择默认高度(飞行球2m/海豚6m/飞艇15m)'
    )

    landing_altitude_arg = DeclareLaunchArgument(
        'landing_altitude',
        default_value='-1.0',
        description='3D降落保护锁桨高度(m)，<0 则按 vehicle_type 自动选择默认高度'
    )

    use_hardware_drivers_arg = DeclareLaunchArgument(
        'use_hardware_drivers',
        default_value='true',
        description='是否启动硬件外设节点 (UWB/LED/声音/舵机/风扇)，SITL 仿真可设为 false'
    )

    start_agent_arg = DeclareLaunchArgument(
        'start_agent',
        default_value='true',
        description='是否在本 launch 中启动 MicroXRCEAgent（多机共享同一 Agent 时可设为 false）'
    )

    coordinate_system_arg = DeclareLaunchArgument(
        'coordinate_system',
        default_value='NED',
        description='usv_control_node 目标输入坐标系 (NED 或 ENU)'
    )

    spawn_offset_x_arg = DeclareLaunchArgument('spawn_offset_x', default_value='0.0', description='世界出生点 X 偏移 (ENU, m)')
    spawn_offset_y_arg = DeclareLaunchArgument('spawn_offset_y', default_value='0.0', description='世界出生点 Y 偏移 (ENU, m)')
    spawn_offset_z_arg = DeclareLaunchArgument('spawn_offset_z', default_value='0.0', description='世界出生点 Z 偏移 (ENU, m)')

    # RL 导航与策略推理参数（按 vehicle_type 自动选择对应配置，可显式覆盖）
    nav_mode_arg = DeclareLaunchArgument(
        'nav_mode',
        default_value='position',
        description="导航控制模式：'position'（默认轨迹位置控制）或 'rl'（端到端 RL (v, omega) 控制）"
    )
    rl_policy_enabled_arg = DeclareLaunchArgument(
        'rl_policy_enabled',
        default_value='false',
        description='是否在本机空间启动 RL policy_inference_node'
    )
    rl_policy_model_arg = DeclareLaunchArgument(
        'rl_policy_model',
        default_value='__auto__',
        description='RL 策略模型路径或文件名 (__auto__ 按 vehicle_type 自动匹配，如 usv_rl_policy.pt)'
    )
    rl_model_arg = DeclareLaunchArgument(
        'rl_model',
        default_value='__auto__',
        description='rl_policy_model 的简写别名，指定要加载的 RL 模型文件名或路径'
    )
    rl_policy_device_arg = DeclareLaunchArgument(
        'rl_policy_device',
        default_value='cpu',
        description='RL 推理设备 (cpu / cuda)'
    )
    rl_publish_rate_arg = DeclareLaunchArgument(
        'rl_publish_rate',
        default_value='10.0',
        description='RL 推理输出频率 (Hz)'
    )
    rl_cruise_speed_arg = DeclareLaunchArgument(
        'rl_cruise_speed',
        default_value='0.60',
        description='RL 策略参考巡航速度 (m/s)'
    )
    rl_encounter_type_arg = DeclareLaunchArgument(
        'rl_encounter_type',
        default_value='auto',
        description='RL 会遇类型条件 (auto / none / head_on / crossing / overtaking)'
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
    platform_mode_raw = LaunchConfiguration('platform_mode')
    use_setpoint_6dof = LaunchConfiguration('use_setpoint_6dof')
    vehicle_type = LaunchConfiguration('vehicle_type')
    platform_mode = PythonExpression([
        "('2d' if '", vehicle_type, "'.strip().lower() in ('usv', 'usv_diff') else '3d') "
        "if '", platform_mode_raw, "'.strip().lower() in ('', 'auto') else '", platform_mode_raw, "'.strip().lower()"
    ])
    reach_xy_threshold = LaunchConfiguration('reach_xy_threshold')
    reach_z_threshold = LaunchConfiguration('reach_z_threshold')
    takeoff_altitude = LaunchConfiguration('takeoff_altitude')
    landing_altitude = LaunchConfiguration('landing_altitude')
    use_hardware_drivers = LaunchConfiguration('use_hardware_drivers')
    start_agent = LaunchConfiguration('start_agent')
    coordinate_system = LaunchConfiguration('coordinate_system')
    spawn_offset_x = LaunchConfiguration('spawn_offset_x')
    spawn_offset_y = LaunchConfiguration('spawn_offset_y')
    spawn_offset_z = LaunchConfiguration('spawn_offset_z')
    nav_mode = LaunchConfiguration('nav_mode')
    rl_policy_enabled = LaunchConfiguration('rl_policy_enabled')
    rl_policy_model = LaunchConfiguration('rl_policy_model')
    rl_model = LaunchConfiguration('rl_model')
    rl_policy_device = LaunchConfiguration('rl_policy_device')
    rl_publish_rate = LaunchConfiguration('rl_publish_rate')
    rl_cruise_speed = LaunchConfiguration('rl_cruise_speed')
    rl_encounter_type = LaunchConfiguration('rl_encounter_type')
    
    # =========================================================================
    # 分组 Domain ID 映射
    # A -> 11, B -> 12, C -> 13, D -> 14, E -> 15, F -> 16, GS -> 99
    # =========================================================================
    domain_id = PythonExpression([
        "{'A': 11, 'B': 12, 'C': 13, 'D': 14, 'E': 15, 'F': 16, 'GS': 99}.get('", 
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
    # 串口模式条件：start_agent=true 且 非以太网 且 非仿真
    use_serial_condition = PythonExpression([
        "'", start_agent, "'.lower() == 'true' and '", use_ethernet, "'.lower() == 'false' and '", use_simulation, "'.lower() == 'false'"
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
    use_sim_agent_condition = PythonExpression([
        "'", start_agent, "'.lower() == 'true' and '", use_simulation, "'.lower() == 'true'"
    ])
    micro_xrce_agent_udp = ExecuteProcess(
        condition=IfCondition(use_sim_agent_condition),
        cmd=[
            '/home/chenhangwei/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent', 'udp4',
            '-p', simulation_port,
        ],
        output='screen',
        name='micro_xrce_agent'
    )
    
    # =========================================================================
    # Micro XRCE-DDS Agent（以太网模式 - 推荐）
    # 飞控通过网线连接树莓派，使用 UDP 通信
    # =========================================================================
    use_eth_agent_condition = PythonExpression([
        "'", start_agent, "'.lower() == 'true' and '", use_ethernet, "'.lower() == 'true'"
    ])
    micro_xrce_agent_ethernet = ExecuteProcess(
        condition=IfCondition(use_eth_agent_condition),
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
                'vehicle_type': vehicle_type,
                'nav_mode': nav_mode,
                'reach_xy_threshold': reach_xy_threshold,
                'reach_z_threshold': reach_z_threshold,
                'takeoff_altitude': takeoff_altitude,
                'landing_altitude': landing_altitude,
                'coordinate_system': coordinate_system,
                'spawn_offset_x': spawn_offset_x,
                'spawn_offset_y': spawn_offset_y,
                'spawn_offset_z': spawn_offset_z,
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
        on_exit=Shutdown(),
        parameters=[param_file],
        remappings=[
            # 注意：PX4 v1.15+ 使用 vehicle_status_v1
            ('/fmu/out/vehicle_status_v1', 'fmu/out/vehicle_status_v1'),
            ('/fmu/out/failsafe_flags', 'fmu/out/failsafe_flags'),
            ('/fmu/out/vehicle_command_ack', 'fmu/out/vehicle_command_ack'),
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
            {
                'publish_rate': 5.0,  # 降低发布频率以减少带宽
                'vehicle_type': vehicle_type,
                'platform_mode': platform_mode,
                'spawn_offset_x': spawn_offset_x,
                'spawn_offset_y': spawn_offset_y,
                'spawn_offset_z': spawn_offset_z,
            },
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
    
    use_uwb_condition = PythonExpression([
        "'", use_hardware_drivers, "'.lower() == 'true' and '", use_uwb, "'.lower() == 'true'"
    ])

    # =========================================================================
    # 编队跟随节点 - 本地高频计算编队偏移并发布 set_usv_nav_goal
    # =========================================================================
    formation_follower_node = Node(
        package='usv_control',
        executable='formation_follower_node',
        name='formation_follower_node',
        namespace=namespace,
        output='screen',
        parameters=[
            param_file,
            {
                'usv_id': namespace,
                'platform_mode': platform_mode,
                'vehicle_type': vehicle_type,
            },
        ],
    )

    # =========================================================================
    # 避障节点 - PX4 版本（仅发布避障目标与标志给 usv_control_node，不直接竞争 fmu/in/*）
    # =========================================================================
    usv_avoidance_node = Node(
        package='usv_control',
        executable='usv_avoidance_node',
        name='usv_avoidance_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file],
        remappings=[
            ('/fmu/out/vehicle_status_v1', 'fmu/out/vehicle_status_v1'),
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
        ],
    )
    
    # =========================================================================
    # 坐标转换节点 - PX4 版本（默认禁用直发设定值，统一由 usv_control_node 发布 fmu/in/trajectory_setpoint）
    # =========================================================================
    coord_transform_node = Node(
        package='usv_control',
        executable='coord_transform_node',
        name='coord_transform_node',
        namespace=namespace,
        output='screen',
        parameters=[
            param_file,
            {
                'coordinate_system': 'ENU',
                'enable_coord_transform': False,
            },
        ],
        remappings=[
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/in/trajectory_setpoint', 'fmu/in/trajectory_setpoint'),
            ('/fmu/in/offboard_control_mode', 'fmu/in/offboard_control_mode'),
        ],
    )
    
    # =========================================================================
    # 自动设置 Home 点节点 - 仅在室内 UWB 硬件定位模式下启用（避免覆盖 Gazebo GPS 原点）
    # =========================================================================
    auto_set_home_node = Node(
        package='usv_comm',
        executable='auto_set_home_node',
        name='auto_set_home_node',
        namespace=namespace,
        output='screen',
        condition=IfCondition(use_uwb_condition),
        parameters=[
            param_file,
            {'set_delay_sec': 5.0},
            # 室内 UWB 场景使用虚拟坐标
            {'fixed_lat': 0.0},
            {'fixed_lon': 0.0},
            {'fixed_alt': 0.0},
        ],
        remappings=[
            ('/fmu/out/vehicle_status_v1', 'fmu/out/vehicle_status_v1'),
            ('/fmu/out/vehicle_local_position', 'fmu/out/vehicle_local_position'),
            ('/fmu/in/vehicle_command', 'fmu/in/vehicle_command'),
        ],
    )

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
        condition=IfCondition(use_uwb_condition),
    )
    
    # =========================================================================
    # 辅助功能节点 (LED, 声音, 扭脖子, 风扇)
    # =========================================================================
    usv_led_node = Node(
        package='usv_led',
        executable='usv_led_node',
        name='usv_led_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file],
        condition=IfCondition(use_hardware_drivers),
    )
    
    usv_sound_node = Node(
        package='usv_sound',
        executable='usv_sound_node',
        name='usv_sound_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file],
        condition=IfCondition(use_hardware_drivers),
    )
    
    usv_head_action_node = Node(
        package='usv_action',
        executable='usv_head_action_node',
        name='usv_head_action_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file],
        condition=IfCondition(use_hardware_drivers),
    )
    
    usv_fan_node = Node(
        package='usv_fan',
        executable='usv_fan_node',
        name='usv_fan_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file],
        condition=IfCondition(use_hardware_drivers),
    )
    
    def _build_rl_policy_node(context, *args, **kwargs):
        enabled_str = rl_policy_enabled.perform(context).strip().lower()
        nav_mode_str = nav_mode.perform(context).strip().lower()
        if enabled_str not in ('1', 'true', 'yes', 'on') and nav_mode_str != 'rl':
            return []

        ns_str = namespace.perform(context).strip()
        vtype_str = vehicle_type.perform(context).strip().lower()
        share_dir = FindPackageShare('usv_control').perform(context)
        models_dir = os.path.join(share_dir, 'models')

        alias_str = rl_model.perform(context).strip()
        model_arg_str = rl_policy_model.perform(context).strip()
        chosen_raw = alias_str if (alias_str and alias_str != '__auto__') else model_arg_str

        default_by_vtype = {
            'usv_diff': 'usv_rl_policy.pt',
            'usv': 'usv_rl_policy.pt',
            'flying_sphere': 'flying_sphere_rl_policy.pt',
            'bionic_airship': 'bionic_airship_rl_policy.pt',
            'bionic_dolphin': 'bionic_dolphin_rl_policy.pt',
        }
        if chosen_raw and chosen_raw != '__auto__':
            expanded = os.path.expanduser(chosen_raw)
            if os.path.isfile(expanded):
                model_str = os.path.abspath(expanded)
            elif os.path.isfile(os.path.join(models_dir, chosen_raw)):
                model_str = os.path.join(models_dir, chosen_raw)
            elif os.path.isfile(os.path.join(models_dir, f'{chosen_raw}.pt')):
                model_str = os.path.join(models_dir, f'{chosen_raw}.pt')
            else:
                model_str = expanded
        else:
            cand = os.path.join(models_dir, default_by_vtype.get(vtype_str, 'usv_rl_policy.pt'))
            model_str = cand if os.path.isfile(cand) else os.path.join(models_dir, 'usv_rl_policy.pt')

        device_str = rl_policy_device.perform(context).strip()
        rate_str = rl_publish_rate.perform(context).strip()
        cruise_str = rl_cruise_speed.perform(context).strip()
        enc_str = rl_encounter_type.perform(context).strip()

        return [
            Node(
                package='usv_control',
                executable='policy_inference_node',
                name='policy_inference_node',
                namespace=ns_str,
                output='screen',
                arguments=[
                    '--namespace', ns_str,
                    '--model', model_str,
                    '--policy', 'auto',
                    '--rl-control-mode', 'pure',
                    '--device', device_str,
                    '--publish-rate', rate_str,
                    '--encounter-type', enc_str,
                    '--turn-governor-cruise', cruise_str,
                    '--disable-controller-param',
                ],
            )
        ]

    # =========================================================================
    # 延迟启动控制节点（等待 Agent 连接）
    # =========================================================================
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            usv_control_node,
            usv_command_node,
            usv_status_node,
            formation_follower_node,
            usv_avoidance_node,
            coord_transform_node,
            auto_set_home_node,
            usv_uwb_node,
            usv_led_node,
            usv_sound_node,
            usv_head_action_node,
            usv_fan_node,
            OpaqueFunction(function=_build_rl_policy_node),
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
        vehicle_type_arg,
        reach_xy_threshold_arg,
        reach_z_threshold_arg,
        takeoff_altitude_arg,
        landing_altitude_arg,
        use_hardware_drivers_arg,
        start_agent_arg,
        coordinate_system_arg,
        spawn_offset_x_arg,
        spawn_offset_y_arg,
        spawn_offset_z_arg,
        nav_mode_arg,
        rl_policy_enabled_arg,
        rl_policy_model_arg,
        rl_model_arg,
        rl_policy_device_arg,
        rl_publish_rate_arg,
        rl_cruise_speed_arg,
        rl_encounter_type_arg,
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
