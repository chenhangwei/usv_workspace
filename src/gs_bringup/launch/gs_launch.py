from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   
  

    

    #状态处理
    main_gui_app= Node(
        package='gs_gui',
        executable='main_gui_app',
        name='main_gui_app',

        output='screen',
     
    )


    return LaunchDescription([

  
       main_gui_app,

    ])