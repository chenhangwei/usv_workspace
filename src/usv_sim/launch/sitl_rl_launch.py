"""
单载具 PX4 SITL + RL 独立启动入口（对齐 usv_rl 分支用法）

每个终端独立启动一台载具，定义命名空间、载具类型与 RL 模型后，
PX4 自动加载对应机架与 Gazebo 模型，ROS 2 自动加载对应 RL 策略模型：

示例：
  # 差速无人船（默认 vehicle_type:=usv_diff，自动加载 4025 usv_boat + usv_rl_policy.pt）
  ros2 launch usv_sim sitl_rl_launch.py namespace:=usv_01 vehicle_type:=usv_diff

  # 指定自定义 RL 模型启动第 2 艘差速无人船（自动附着同一 Gazebo 世界并错开出生点）
  ros2 launch usv_sim sitl_rl_launch.py namespace:=usv_02 vehicle_type:=usv_diff rl_model:=usv_rl_policy.pt

  # 启动三维无人球 / 仿生飞艇 / 仿生海豚
  ros2 launch usv_sim sitl_rl_launch.py namespace:=usv_03 vehicle_type:=flying_sphere
  ros2 launch usv_sim sitl_rl_launch.py namespace:=usv_04 vehicle_type:=bionic_airship
  ros2 launch usv_sim sitl_rl_launch.py namespace:=usv_05 vehicle_type:=bionic_dolphin
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
        DeclareLaunchArgument(
            'rl_model',
            default_value='__auto__',
            description='RL 模型路径或文件名 (__auto__ 按 vehicle_type 自动匹配，如 usv_rl_policy.pt)',
        ),
        DeclareLaunchArgument('rl_policy_model', default_value='__auto__', description='rl_model 完整参数名'),
        DeclareLaunchArgument('rl_policy_device', default_value='cpu', description='RL 推理设备 (cpu / cuda)'),
        DeclareLaunchArgument('rl_publish_rate', default_value='10.0', description='RL 推理频率 (Hz)'),
        DeclareLaunchArgument('rl_cruise_speed', default_value='__auto__', description='RL 巡航速度 (m/s)'),
        DeclareLaunchArgument('rl_encounter_type', default_value='auto', description='RL 会遇模式'),
        DeclareLaunchArgument('nav_mode', default_value='rl', description="导航模式 (默认 'rl')"),
        DeclareLaunchArgument('spawn_x', default_value='__auto__', description='出生点 ENU X (m)'),
        DeclareLaunchArgument('spawn_y', default_value='__auto__', description='出生点 ENU Y (m)'),
        DeclareLaunchArgument('spawn_z', default_value='__auto__', description='出生点 ENU Z (m)'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='出生点偏航角 (rad)'),
        DeclareLaunchArgument('x', default_value='__auto__', description='出生点 ENU X (m) 简写别名'),
        DeclareLaunchArgument('y', default_value='__auto__', description='出生点 ENU Y (m) 简写别名'),
        DeclareLaunchArgument('z', default_value='__auto__', description='出生点 ENU Z (m) 简写别名'),
        DeclareLaunchArgument('yaw', default_value='__auto__', description='出生点偏航角 (rad) 简写别名'),
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
