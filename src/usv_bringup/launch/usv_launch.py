from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():
    # 声明命名空间参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='usv_01',# 默认命名空间
        description='无人船节点的命名空间'
    )
    # 声明参数文件路径参数
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('usv_bringup'),
            'config',
            'usv_params.yaml'
        ]),
        description='设备站的参数文件路径'

     ) 
    

    # 加载参数文件
    param_file = LaunchConfiguration('param_file')
    namespace = LaunchConfiguration('namespace')

    

    #状态处理
    usv_status_node = Node(
        package='usv_comm',
        executable='usv_status_node',
        name='usv_status_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )
    # 自动设置home点
    auto_set_home_node = Node(
        package='usv_comm',
        executable='auto_set_home_node',
        name='auto_set_home_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

    #避障
    usv_avoidance_node= Node(
        package='usv_control',
        executable='usv_avoidance_node',
        name='usv_avoidance_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

    # mode和arm切换
    usv_command_node= Node(
        package='usv_control',
        executable='usv_command_node',
        name='usv_command_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

    # 控制器
    usv_control_node= Node(
        package='usv_control',
        executable='usv_control_node',
        name='usv_control_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

    # uwb发布
    usv_uwb_node= Node(
        package='usv_drivers',
        executable='usv_uwb_node',
        name='usv_uwb_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

    # 激光雷达
    usv_laserscan_node= Node(
        package='usv_drivers',
        executable='usv_laserscan_node',
        name='usv_laserscan_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

    # 超声波
    usv_ultrasonic_node= Node(
        package='usv_drivers',
        executable='usv_ultrasonic_node',
        name='usv_ultrasonic_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )
    # su04超声波
    usv_su04_node= Node(
        package='usv_drivers',
        executable='usv_su04_node',
        name='usv_su04_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

    #  LED控制
    usv_led_node = Node(
        package='usv_led',
        executable='usv_led_node',
        name='usv_led_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

    # 声音控制
    usv_sound_node = Node(
        package='usv_sound',
        executable='usv_sound_node',
        name='usv_sound_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

    # fan控制
    usv_fan_node = Node(
        package='usv_fan',
        executable='usv_fan_node',
        name='usv_fan_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )
    # 超声波雷达节点
    usv_ultrasonic_radar_node = Node(
        package='usv_drivers',
        executable='usv_ultrasonic_radar_node',
        name='usv_ultrasonic_radar_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

    # 鸭头转动节点
    usv_head_action_node = Node(
        package='usv_action',
        executable='usv_head_action_node',
        name='usv_head_action_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

    # 定义 MAVROS 节点
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',  
        #name='mavros',      
        namespace=namespace ,  # 使用相同的命名空间
        output='screen',
        parameters=[
            param_file,  # 加载参数文件
            {'fcu_url': 'serial:///dev/ttyACM0:921600'},  # 飞控串口和波特率
            {'gcs_url': 'udp://:14550@192.168.68.51:14550'},  # 禁用 GCS 代理
            {'tgt_system': 1},  # 飞控的系统 ID
            {'tgt_component': 1},  # 飞控的组件 ID
            # {'plugin_blacklist': ['global_position']},  # 禁用 global_position 插件        
        ]
    )

    # 启动 RPLIDAR 节点
    rplidar_node = Node(
        package='rplidar_ros',
        # executable='rplidar_node',
        executable='rplidar_node',
        name='rplidar_composition',
        namespace=namespace ,  # 使用相同的命名空间
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
            {'serial_baudrate': 115200},
            {'scan_frequency': 10.0},
            {'frame_id': [TextSubstitution(text='laser_frame_'), LaunchConfiguration('namespace')]}
        ]
    )
    # 启动静态变换发布器
    static_tf_laser_node=Node(
        package='usv_tf',
        executable='static_tf_laser_node',
        name='static_tf_laser_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )
    # 启动 odom 到 TF 的转换节点
    odom_to_tf=Node(
        package='usv_tf',
        executable='odom_to_tf',
        name='odom_to_tf',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )



    return LaunchDescription([
        namespace_arg,
        param_file_arg,  # 添加参数文件声明
        mavros_node,  # 添加 MAVROS 节点   
        usv_status_node,# 状态处理节点
        usv_control_node,# 控制器节点
        # usv_uwb_node,#不再使用机载计算机读取定位
        usv_command_node,# 命令和arm切换节点
        usv_avoidance_node,# 避障节点
        # usv_laserscan_node,# 激光雷达节点
        # usv_ultrasonic_node,# 超声波节点
        auto_set_home_node,# 自动设置home点节点
        # rplidar_node,# RPLIDAR 节点
        # static_tf_laser_node,# 静态变换发布器节点
        # odom_to_tf,# odom 到 TF 的转换节点
        usv_led_node,# LED 控制节点
        usv_sound_node, # 声音控制节点
        # usv_su04_node,  # SU04 超声波节点  
        usv_fan_node,  # 风扇控制节点  
        usv_ultrasonic_radar_node,  # 超声波雷达节点
        usv_head_action_node,  # 鸭头转动节点
    ]
    )