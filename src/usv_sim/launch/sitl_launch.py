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
    # Default ArduPilot SITL uses UDP port 14550 for GCS, but allows connection on other ports?
    # MAVROS usually connects to bind port 14540 (local) or connects remote.
    # Standard: 'udp://127.0.0.1:14551@14555' (MAVROS bind 14551, target 14555)
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://127.0.0.1:14551@14555',
        description='MAVROS FCU URL for SITL'
    )

    # GCS URL
    # Can bridge to local QGC on 14550
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@localhost:14550',
        description='MAVROS GCS URL'
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
            # We might want to disable some hardware specific nodes if they fail gracefully?
            # Or assume SITL replaces FCU and other hardware is just missing/ignored.
            # usv_launch doesn't have many flags to disable drivers yet, except lidar maybe.
        }.items()
    )

    return LaunchDescription([
        LogInfo(msg="Starting USV in SITL Mode..."),
        namespace_arg,
        fcu_url_arg,
        gcs_url_arg,
        usv_bringup
    ])
