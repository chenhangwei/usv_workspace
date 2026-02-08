from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Args
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='usv_03',
        description='Namespace for the USV'
    )
    
    # SITL connection string
    # ⚠️ sim_vehicle.py 启动时 MAVProxy 会占用 tcp:5760，所以必须用 --out 转发给 MAVROS
    # SITL 启动命令需加: --out=udp:127.0.0.1:14551
    # MAVROS 通过 UDP 监听 14551 端口接收转发的数据
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14551@',
        description='MAVROS FCU URL (UDP监听14551端口接收MAVProxy转发)'
    )

    # GCS URL - 转发到本地 QGroundControl (可选)
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@localhost:14550',
        description='MAVROS GCS URL (转发到本地QGC)'
    )

    namespace = LaunchConfiguration('namespace')
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')

    # Include usv_bringup with SITL params
    usv_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_bringup'),
                'launch',
                'usv_launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
            'fcu_url': fcu_url,
            'gcs_url': gcs_url,
            'simulation_mode': 'sitl',
        }.items()
    )

    return LaunchDescription([
        LogInfo(msg="Starting USV in SITL Mode..."),
        namespace_arg,
        fcu_url_arg,
        gcs_url_arg,
        usv_bringup
    ])
