from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import re


def _launch_setup(context, *args, **kwargs):
    """
    支持两种用法:
      1. ros2 launch usv_sim sitl_launch.py namespace:=usv_03
         → 从 namespace 自动推导 instance=2, sysid=3
      2. ros2 launch usv_sim sitl_launch.py instance:=2
         → 自动生成 namespace=usv_03, sysid=3
    """
    instance_str = LaunchConfiguration('instance').perform(context)
    namespace_str = LaunchConfiguration('namespace').perform(context)

    if namespace_str:
        # 用户传了 namespace，从中提取编号
        match = re.search(r'(\d+)', namespace_str)
        if match:
            sysid = int(match.group(1))
            instance = sysid - 1
        else:
            # namespace 不含数字(如 "leader")，用 instance 参数
            instance = int(instance_str)
            sysid = instance + 1
        namespace = namespace_str
    else:
        # 用户没传 namespace，从 instance 推导
        instance = int(instance_str)
        sysid = instance + 1
        namespace = f'usv_{sysid:02d}'

    mavlink_port = 5760 + instance * 10
    fcu_url_auto = f'tcp://127.0.0.1:{mavlink_port}'

    user_fcu = LaunchConfiguration('fcu_url').perform(context)
    fcu_url_final = fcu_url_auto if user_fcu == '__auto__' else user_fcu

    return [
        LogInfo(msg=f'Starting USV: namespace={namespace}, instance={instance}, '
                    f'sysid={sysid}, fcu_url={fcu_url_final}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('usv_bringup'),
                    'launch',
                    'usv_launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': namespace,
                'fcu_url': fcu_url_final,
                'gcs_url': '',
                'simulation_mode': 'sitl',
                'target_system_id': str(sysid),
            }.items()
        ),
    ]


def generate_launch_description():
    """
    单艘 USV SITL 启动文件

    用法:
        # 方式1: 直接指定 namespace (推荐, 自动推导端口)
        ros2 launch usv_sim sitl_launch.py namespace:=usv_01
        ros2 launch usv_sim sitl_launch.py namespace:=usv_03
        ros2 launch usv_sim sitl_launch.py namespace:=usv_10

        # 方式2: 指定 instance 编号
        ros2 launch usv_sim sitl_launch.py instance:=0   # → usv_01
        ros2 launch usv_sim sitl_launch.py instance:=2   # → usv_03

        # 方式3: 自定义 namespace + instance
        ros2 launch usv_sim sitl_launch.py instance:=0 namespace:=leader

    自动映射关系:
        namespace  →  instance  →  SYSID  →  TCP port
        usv_01     →  0         →  1      →  5760
        usv_02     →  1         →  2      →  5770
        usv_03     →  2         →  3      →  5780
        ...
        usv_10     →  9         →  10     →  5850
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'instance',
            default_value='0',
            description='SITL 实例编号 (0-9), 传 namespace 时可省略'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='USV namespace (如 usv_03), 自动推导 instance 和端口'
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='__auto__',
            description='自定义 FCU URL (默认自动计算)'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
