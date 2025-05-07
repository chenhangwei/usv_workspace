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

          # uwb避障
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

     # 发布器
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

    # uwb激光雷达
    usv_laserscan_node= Node(
        package='usv_drivers',
        executable='usv_laserscan_node',
        name='usv_laserscan_node',
        namespace=namespace ,  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

      # uwb超声波
    usv_ultrasonic_node= Node(
        package='usv_drivers',
        executable='usv_ultrasonic_node',
        name='usv_ultrasonic_node',
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
            {'gcs_url': ''},  # 禁用 GCS 代理
            {'tgt_system': 1},  # 飞控的系统 ID
            {'tgt_component': 1},  # 飞控的组件 ID
            # {'plugin_blacklist': ['global_position']},  # 禁用 global_position 插件        
        ]
    )

    # 启动 RPLIDAR 节点
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        namespace=namespace ,  # 使用相同的命名空间
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
            {'serial_baudrate': 115200},
            {'scan_frequency': 10.0},
            {'frame_id': [TextSubstitution(text='laser_frame_'), LaunchConfiguration('namespace')]}
        ]
    )

    static_tf_laser_node=Node(
        package='usv_tf',
        executable='static_tf_laser_node',
        name='static_tf_laser_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

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
        usv_status_node,
        usv_control_node,
        # usv_uwb_node,#不再使用机载计算机读取定位
        usv_command_node,
        usv_avoidance_node,
        # usv_laserscan_node,
        # usv_ultrasonic_node,
        auto_set_home_node,
        rplidar_node,
        static_tf_laser_node,
        odom_to_tf,
        # usv_led_node,
        # usv_sound_node,       
    ]
    )