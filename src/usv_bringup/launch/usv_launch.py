"""
无人船(Ultra Short Wave Vehicle, USV)启动文件
该文件用于启动完整的USV系统，包括飞控通信、传感器驱动、控制逻辑等模块
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    生成USV系统启动描述
    
    该函数定义了完整的USV系统启动配置，包括：
    1. 基础参数配置（命名空间、参数文件）
    2. 飞控通信模块（MAVROS）
    3. 状态管理模块
    4. 控制模块
    5. 传感器驱动模块
    6. 辅助功能模块（LED、声音、风扇等）
    
    可配置参数：
    - namespace: 节点命名空间
    - param_file: 参数文件路径
    - fcu_url: 飞控通信串口
    - gcs_url: 地面站通信地址
    - lidar_port: 激光雷达串口路径
    
    Returns:
        LaunchDescription: 包含所有节点和参数的启动描述对象
    """
    # =============================================================================
    # 参数声明
    # =============================================================================

    # 命名空间参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='usv_02',
        description='无人船节点的命名空间'
    )
    
    # 参数文件路径参数
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('usv_bringup'),
            'config',
            'usv_params.yaml'
        ]),
        description='设备站的参数文件路径'
    )

 
    
    # 飞控串口参数
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyACM0:921600',
        description='飞控通信串口和波特率'
    )
    
    # 地面站通信参数
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://:14570@192.168.68.53:14550',#
        description='地面站通信地址'
    )
    
    # MAVROS目标系统ID参数
    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='2',
        description='MAVROS目标系统ID'
    )
    
    # MAVROS目标组件ID参数
    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='MAVROS目标组件ID'
    )
    
    # 激光雷达串口参数
   # lidar_port_arg = DeclareLaunchArgument(
      #  'lidar_port',
      #  default_value='/dev/ttyUSB0',
      #  description='激光雷达串口路径'
    #)

    # =============================================================================
    # 参数配置加载
    # =============================================================================

    param_file = LaunchConfiguration('param_file')

    namespace = LaunchConfiguration('namespace')
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    #lidar_port = LaunchConfiguration('lidar_port')

    # =============================================================================
    # 通信与状态管理节点
    # =============================================================================

  

    # 状态处理节点
    usv_status_node = Node(
        package='usv_comm',
        executable='usv_status_node',
        name='usv_status_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 自动设置home点节点
    auto_set_home_node = Node(
        package='usv_comm',
        executable='auto_set_home_node',
        name='auto_set_home_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # =============================================================================
    # 控制相关节点
    # =============================================================================

    # 避障节点
    # usv_avoidance_node = Node(
    #     package='usv_control',
    #     executable='usv_avoidance_node',
    #     name='usv_avoidance_node',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[param_file]
    # )

    # mode和arm切换节点
    usv_command_node = Node(
        package='usv_control',
        executable='usv_command_node',
        name='usv_command_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 控制器节点
    usv_control_node = Node(
        package='usv_control',
        executable='usv_control_node',
        name='usv_control_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # =============================================================================
    # 传感器驱动节点
    # =============================================================================

    # UWB定位节点
    usv_uwb_node = Node(
        package='usv_drivers',
        executable='usv_uwb_node',
        name='usv_uwb_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 激光雷达节点
    usv_laserscan_node = Node(
        package='usv_drivers',
        executable='usv_laserscan_node',
        name='usv_laserscan_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 超声波传感器节点
    usv_ultrasonic_node = Node(
        package='usv_drivers',
        executable='usv_ultrasonic_node',
        name='usv_ultrasonic_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # SU04超声波传感器节点
    usv_su04_node = Node(
        package='usv_drivers',
        executable='usv_su04_node',
        name='usv_su04_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 超声波雷达节点
    usv_ultrasonic_radar_node = Node(
        package='usv_drivers',
        executable='usv_ultrasonic_radar_node',
        name='usv_ultrasonic_radar_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # =============================================================================
    # 辅助功能节点
    # =============================================================================

    # LED控制节点
    usv_led_node = Node(
        package='usv_led',
        executable='usv_led_node',
        name='usv_led_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 声音控制节点
    usv_sound_node = Node(
        package='usv_sound',
        executable='usv_sound_node',
        name='usv_sound_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 风扇控制节点
    usv_fan_node = Node(
        package='usv_fan',
        executable='usv_fan_node',
        name='usv_fan_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 鸭头动作控制节点
    usv_head_action_node = Node(
        package='usv_action',
        executable='usv_head_action_node',
        name='usv_head_action_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # =============================================================================
    # 飞控通信节点
    # =============================================================================

    # MAVROS节点
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                # 核心连接参数
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                # MAVLink身份配置（关键修复：确保使用正确的系统ID）
                'system_id': 2,  # MAVROS自身系统ID
                'component_id': 191,  # MAVROS自身组件ID
                'target_system_id': 2,  # 目标飞控系统ID
                'target_component_id': 1,  # 目标飞控组件ID
            },
            param_file,  # 其他参数（plugin_allowlist等）从YAML加载
        ]
    )

    # =============================================================================
    # 传感器相关节点
    # =============================================================================

    # RPLIDAR节点
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_composition',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                #'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'scan_frequency': 10.0,
                'frame_id': [
                    TextSubstitution(text='laser_frame_'), 
                    LaunchConfiguration('namespace')
                ]
            }
        ]
    )

    # =============================================================================
    # 坐标变换节点
    # =============================================================================

    # 静态坐标变换发布器
    static_tf_laser_node = Node(
        package='usv_tf',
        executable='static_tf_laser_node',
        name='static_tf_laser_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 里程计到TF转换节点
    odom_to_tf = Node(
        package='usv_tf',
        executable='odom_to_tf',
        name='odom_to_tf',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # =============================================================================
    # 启动顺序控制：等待 MAVROS 连接后再启动依赖节点
    # =============================================================================
    
    # 第一批：延迟 0.5 秒启动 auto_set_home_node（尽早设置 EKF 原点）
    # 关键优化：减少延迟，在 EKF 初始化前发送 SET_GPS_GLOBAL_ORIGIN
    delayed_home_node = TimerAction(
        period=0.5,  # 仅等待 MAVROS 启动（0.5秒足够 topic 就绪）
        actions=[
           auto_set_home_node,    # 自动设置 EKF Origin（必须在 EKF 初始化前）
        ]
    )
    
    # 第二批：延迟 3 秒启动其他节点（确保 EKF 原点已设置）
    # 计算：0.5s (auto_set_home启动) + 1s (GPS就绪) + 1s (auto_set_home延迟) + 0.5s (余量) = 3s
    delayed_control_nodes = TimerAction(
        period=3.0,
        actions=[
            usv_status_node,       # 状态管理（依赖 MAVROS）
            usv_control_node,      # 核心控制器（依赖 MAVROS 和 EKF 原点）
            usv_command_node,      # 命令处理（依赖 MAVROS）
            # usv_avoidance_node,  # 避障功能（已注释）
        ]
    )
    
    # =============================================================================
    # 启动描述配置
    # =============================================================================

    # 启动顺序（优化后总时间约 3 秒）：
    # 阶段 1 (t=0s):   启动 MAVROS（开始连接飞控）
    # 阶段 2 (t=0.5s): 启动 auto_set_home_node（尽早发送 SET_GPS_GLOBAL_ORIGIN）
    # 阶段 3 (t=3s):   启动控制节点（确保 EKF 原点已设置，避免 "no origin" 警告）
    # 阶段 4 (t=0s):   辅助功能节点并行启动（不依赖 MAVROS）
    return LaunchDescription([
        # 基础参数配置
        namespace_arg,
        param_file_arg,
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        #lidar_port_arg,
        
        # 阶段 1：立即启动 MAVROS
        mavros_node,           # 飞控通信（优先启动）
       
        
        # 阶段 2：延迟 0.5 秒启动 EKF Origin 设置（关键优化）
        delayed_home_node,
        
        # 阶段 3：延迟 3 秒启动控制节点
        delayed_control_nodes,
        
        # 第三阶段：辅助功能节点（不依赖 MAVROS，可并行启动）
        usv_led_node,          # LED控制
        usv_sound_node,        # 声音控制
        usv_fan_node,          # 风扇控制
        #usv_ultrasonic_radar_node,  # 超声波雷达
        usv_head_action_node,       # 鸭头动作控制
        
        # 可选节点（根据硬件配置启用）
        # usv_uwb_node,             # UWB定位
        # usv_laserscan_node,       # 激光雷达
        # usv_ultrasonic_node,      # 超声波传感器
        # usv_su04_node,            # SU04超声波传感器
        # rplidar_node,             # RPLIDAR激光雷达
        # static_tf_laser_node,     # 激光雷达坐标变换
        # odom_to_tf,               # 里程计坐标变换
    ])
