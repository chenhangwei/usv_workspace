from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
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

    gs_param_file = LaunchConfiguration('gs_param_file')

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
            {'use_sim_time': False},
            gs_param_file
        ]
    )


    return LaunchDescription([
        gs_param_arg,
        main_gui_app,
    ])