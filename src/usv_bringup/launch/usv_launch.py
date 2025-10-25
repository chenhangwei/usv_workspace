"""
无人船(Ultra Short Wave Vehicle, USV)启动文件
该文件用于启动完整的USV系统，包括飞控通信、传感器驱动、控制逻辑等模块
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
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
        default_value='udp://:14570@192.168.68.53:14550',
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
    tgt_system = LaunchConfiguration('tgt_system')
    tgt_component = LaunchConfiguration('tgt_component')

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
    usv_avoidance_node = Node(
        package='usv_control',
        executable='usv_avoidance_node',
        name='usv_avoidance_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

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
            param_file,
            {
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                # 保留旧键以兼容（若 MAVROS 忽略将无害）
                'tgt_system': tgt_system,
                'tgt_component': tgt_component,
                # ROS 2 MAVROS 常用参数名：设置自身与目标的 MAVLink ID
                'system_id': tgt_system,
                'component_id': tgt_component,
                # 兼容某些版本用于目标 FCU 的参数命名
                'target_system_id': tgt_system,
                'target_component_id': tgt_component,
                
                # ==================== 性能优化：只加载必需的插件 ====================
                # 插件白名单配置，大幅减少启动时间（从55秒降至3-5秒）
                # 只加载 USV 控制必需的插件，避免加载 60+ 个不需要的插件
                'plugin_allowlist': [
                    'sys_status',      # 系统状态（必需）
                    'sys_time',        # 时间同步（必需）
                    'command',         # 命令接口（解锁/模式切换）
                    'param',           # 参数读写
                    'local_position',  # 本地位置（导航必需）
                    'setpoint_raw',    # 原始设定点（控制必需）
                    'global_position', # GPS 全局位置（新增）
                    'gps_status',      # GPS 状态和卫星数（新增）
                    # 'altitude',      # 高度信息（可选，已包含在 global_position 中）
                    # 'imu',           # IMU 数据（可选）
                ],
                
                # 禁用视觉定位（如不使用外部定位系统）
                'vision_pose.enable': False
                # ==================== END 性能优化 ====================
            }
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
    # 启动描述配置
    # =============================================================================

    # 当前启用的节点列表
    # 注意：部分节点被注释是因为在当前部署环境中不使用
    # 如需启用，请取消相应注释并确保硬件连接正确
    return LaunchDescription([
    # 基础参数配置
    namespace_arg,
    param_file_arg,
    fcu_url_arg,
    gcs_url_arg,
    tgt_system_arg,
    tgt_component_arg,
    #lidar_port_arg,
        
        # 核心功能节点
        mavros_node,           # 飞控通信
        usv_status_node,       # 状态管理
        usv_control_node,      # 核心控制器
        usv_command_node,      # 命令处理
        usv_avoidance_node,    # 避障功能
        auto_set_home_node,    # 自动设置home点
        
        # 辅助功能节点
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