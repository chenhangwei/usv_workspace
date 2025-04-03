from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
     # 声明命名空间参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='usv_2',  # 默认命名空间
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
    

    #状态处理
    status_reporter_node = Node(
        package='usv_comm',
        executable='status_reporter_node',
        name='status_reporter_node',
        namespace=LaunchConfiguration('namespace'),  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

     # 本地规划
    local_planner_node= Node(
        package='usv_control',
        executable='local_planner_node',
        name='local_planner_node',
        namespace=LaunchConfiguration('namespace'),  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )

      # uwb发布
    uwb_publisher_node= Node(
        package='usv_drivers',
        executable='uwb_publisher_node',
        name='uwb_publisher_node',
        namespace=LaunchConfiguration('namespace'),  # 使用命名空间
        output='screen',
        parameters=[param_file],  # 加载参数文件
    )



      # 定义 MAVROS 节点
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace=LaunchConfiguration('namespace'),  # 使用相同的命名空间
        output='screen',
        parameters=[
            param_file,
            {'fcu_url': 'serial:///dev/ttyACM0:57600'},  # 飞控串口和波特率
            {'gcs_url': ''},  # 禁用 GCS 代理
            {'tgt_system': 1},  # 飞控的系统 ID
            {'tgt_component': 1},  # 飞控的组件 ID
        ]
    )


    return LaunchDescription([
        namespace_arg,
        param_file_arg,  # 添加参数文件声明
        status_reporter_node,
        local_planner_node,
        uwb_publisher_node,
        mavros_node,  # 添加 MAVROS 节点
    ])