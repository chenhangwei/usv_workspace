from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
   
  

    

    # gs 参数文件（包含 area_center）
    import os
    # 默认优先使用 workspace 下的 src 配置（便于源码直接运行），如不存在再由运行时解析做回退
    default_gs_params = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'gs_bringup', 'config', 'gs_params.yaml'))
    gs_param_arg = DeclareLaunchArgument(
        'gs_param_file',
        default_value=default_gs_params,
        description='Ground station parameters file'
    )

    # USV fleet 配置文件（用于Domain隔离架构）
    default_fleet_config = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'gs_bringup', 'config', 'usv_fleet.yaml'))
    fleet_config_arg = DeclareLaunchArgument(
        'fleet_config_file',
        default_value=default_fleet_config,
        description='USV fleet configuration file (for Domain isolation architecture)'
    )

    gs_param_file = LaunchConfiguration('gs_param_file')
    fleet_config_file = LaunchConfiguration('fleet_config_file')

    def _resolve_gs_param_file(context, *args, **kwargs):
        """Resolve gs_params.yaml path at runtime: prefer package share, fallback to workspace src."""
        try:
            # try installed package share path
            pkg_share = FindPackageShare('usv_bringup').perform(context)
            candidate = os.path.join(pkg_share, 'config', 'gs_params.yaml')
            if os.path.isfile(candidate):
                return [SetLaunchConfiguration('gs_param_file', candidate)]
        except Exception:
            pass
        # fallback: look into workspace src relative to current working dir
        try:
            wd = os.getcwd()
            candidate2 = os.path.abspath(os.path.join(wd, 'src', 'usv_bringup', 'config', 'gs_params.yaml'))
            if os.path.isfile(candidate2):
                return [SetLaunchConfiguration('gs_param_file', candidate2)]
        except Exception:
            pass
        # final fallback: leave as-is (the LaunchConfiguration default may point to non-existent file)
        return []

    # 状态处理 / GUI
    main_gui_app = Node(
        package='gs_gui',
        executable='main_gui_app',
        name='main_gui_app',
        output='screen',
        parameters=[
            gs_param_file,  # 从文件加载参数
            {
                'use_sim_time': False,
                'fleet_config_file': default_fleet_config,  # 直接使用字符串路径
            }
        ]
    )

    # ⚠️ Domain Bridge 已移除 - 请使用独立脚本管理
    # Domain Bridge 应该独立启动，避免与手动启动冲突:
    #   启动: ./src/gs_bringup/scripts/domain_bridge.sh start
    #   停止: ./src/gs_bringup/scripts/domain_bridge.sh stop
    #   状态: ./src/gs_bringup/scripts/domain_bridge.sh status
    #
    # 原因: Domain Bridge 需要在地面站启动前先运行，且不应该随地面站重启而重启
    # 配置文件: ~/domain_bridge/domain_bridge.yaml

    return LaunchDescription([
        gs_param_arg,
        fleet_config_arg,  # 添加fleet配置参数
        main_gui_app,
        # domain_bridge_launch,  # 已注释 - 请独立启动
    ])