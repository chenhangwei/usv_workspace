from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
   
  

    

    # gs 参数文件（包含 area_center）
    gs_param_arg = DeclareLaunchArgument(
        'gs_param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('usv_bringup'),
            'config',
            'gs_params.yaml'
        ]),
        description='Ground station parameters file'
    )

    gs_param_file = LaunchConfiguration('gs_param_file')

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