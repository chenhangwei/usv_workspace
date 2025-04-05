from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
     # 声明命名空间参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='usv_1',# 默认命名空间
        description='无人船节点的命名空间'
    )
      # 声明参数文件路径参数
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('usv_bringup'),
            'config',
            'usv_station_params.yaml'
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



      # 定义 MAVROS 节点
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',  
        # name='mavros',      
        namespace=namespace ,  # 使用相同的命名空间
        output='screen',
        parameters=[
            param_file,  # 加载参数文件
            {'fcu_url': 'serial:///dev/ttyACM0:57600'},  # 飞控串口和波特率
            {'gcs_url': ''},  # 禁用 GCS 代理
            {'tgt_system': 1},  # 飞控的系统 ID
            {'tgt_component': 1},  # 飞控的组件 ID
        ]
    )


    return LaunchDescription([
        namespace_arg,
        param_file_arg,  # 添加参数文件声明
        mavros_node,  # 添加 MAVROS 节点

    
        usv_status_node,
        usv_control_node,
        usv_uwb_node,
        usv_command_node,
        usv_avoidance_node,
        usv_laserscan_node,
        usv_ultrasonic_node,
        
    ]
    )