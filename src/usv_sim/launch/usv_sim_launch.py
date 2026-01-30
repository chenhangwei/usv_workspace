from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    # Args
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='usv_03',
        description='Namespace for the USV'
    )
    
    # 仿真类型：simple 或 sitl
    sim_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='simple', 
        description='Simulation mode: simple (internal script) or sitl (ArduPilot+Gazebo)'
    )

    namespace = LaunchConfiguration('namespace')
    sim_mode = LaunchConfiguration('mode')

    # =========================================================================
    # 1. 简单仿真模式: 启动 simple_sim_node
    # =========================================================================
    
    simple_sim_node = Node(
        package='usv_sim',
        executable='simple_sim_node',
        name='usv_sim_node',
        namespace=namespace,
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'simple'),
        parameters=[{
            'start_x': 0.0,
            'start_y': 0.0,
            'start_yaw': 0.0,
            'update_rate': 20.0
        }]
    )

    # =========================================================================
    # 2. 启动核心控制栈 (USV Bringup)
    # =========================================================================
    
    # 简单模式: 只通过 usv_launch 启动控制栈，不启动 mavros (usv_launch 内部会根据 mode=simple 决定)
    usv_bringup_simple = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_bringup'),
                'launch',
                'usv_launch.py'
            ])
        ]),
        condition=LaunchConfigurationEquals('mode', 'simple'),
        launch_arguments={
            'namespace': namespace,
            'simulation_mode': 'simple',
        }.items()
    )

    # SITL模式: 启动 usv_launch 并覆盖参数以连接 SITL
    usv_bringup_sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_bringup'),
                'launch',
                'usv_launch.py'
            ])
        ]),
        condition=LaunchConfigurationEquals('mode', 'sitl'),
        launch_arguments={
            'namespace': namespace,
            'simulation_mode': 'sitl',
            'fcu_url': 'udp://127.0.0.1:14551@14555', # 尝试连接本地 SITL
            # gcs_url 这里设为空，或者让 MAVROS 转发到本地 QGC
            'gcs_url': 'udp://@localhost:14550' 
        }.items()
    )

    return LaunchDescription([
        namespace_arg,
        sim_mode_arg,
        usv_bringup_simple,
        usv_bringup_sitl,
        simple_sim_node,
        LogInfo(msg=["Starting USV Simulation in mode: ", sim_mode]),
    ])
