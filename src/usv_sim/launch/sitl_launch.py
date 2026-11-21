"""
单载具 PX4 SITL 位置控制模式独立启动入口（对齐 usv_rl 分支 sitl_launch.py）

默认以 nav_mode:=position 启动单台载具，也可传 nav_mode:=rl 切换至 RL 控制：
  ros2 launch usv_sim sitl_launch.py namespace:=usv_01 vehicle_type:=usv_diff
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('namespace', default_value='__auto__', description='载具命名空间 (如 usv_01, usv_02)'),
        DeclareLaunchArgument('instance', default_value='__auto__', description='兼容 usv_rl 风格 instance 索引 (0->usv_01)'),
        DeclareLaunchArgument('instance_id', default_value='__auto__', description='PX4 实例编号 (__auto__ 从 namespace 提取)'),
        DeclareLaunchArgument(
            'vehicle_type',
            default_value='usv_diff',
            description='载具类型: usv_diff | flying_sphere | bionic_airship | bionic_dolphin',
        ),
        DeclareLaunchArgument('nav_mode', default_value='position', description="导航模式 (默认 'position'，可改 'rl')"),
        DeclareLaunchArgument('rl_model', default_value='__auto__', description='RL 模型路径或文件名'),
        DeclareLaunchArgument('rl_policy_model', default_value='__auto__', description='rl_model 完整参数名'),
        DeclareLaunchArgument('spawn_x', default_value='__auto__', description='出生点 ENU X (m)'),
        DeclareLaunchArgument('spawn_y', default_value='__auto__', description='出生点 ENU Y (m)'),
        DeclareLaunchArgument('spawn_z', default_value='__auto__', description='出生点 ENU Z (m)'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='出生点偏航角 (rad)'),
        DeclareLaunchArgument('domain_id', default_value='99', description='ROS_DOMAIN_ID'),
        DeclareLaunchArgument('headless', default_value='__auto__', description='Gazebo 无头模式 (true/false)'),
        DeclareLaunchArgument('gui', default_value='__auto__', description='是否显示 Gazebo GUI (true/false)'),
        DeclareLaunchArgument('gz_world', default_value='default', description='Gazebo 世界名称'),
        DeclareLaunchArgument('start_agent', default_value='__auto__', description='是否启动 MicroXRCEAgent (__auto__ 自动检测复用)'),
        DeclareLaunchArgument('standalone_world', default_value='__auto__', description='是否附着已有 Gazebo (__auto__ 自动检测)'),
    ]

    forwarded = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('usv_bringup'),
                'launch',
                'usv_sitl_agent_launch.py',
            ])
        ),
        launch_arguments=forwarded.items(),
    )

    return LaunchDescription([*args, agent_launch])
