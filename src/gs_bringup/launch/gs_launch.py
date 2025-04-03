from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gs_gui',
            executable='main_gui_app',
            name='ground_station_gui',
            output='screen',
        ),
    ])