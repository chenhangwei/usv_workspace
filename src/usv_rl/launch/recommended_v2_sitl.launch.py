from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from usv_rl.recommended import RECOMMENDED_MODEL_RELATIVE_PATH


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    instance = LaunchConfiguration('instance')
    fcu_url = LaunchConfiguration('fcu_url')
    disable_controller_param = LaunchConfiguration('disable_controller_param')
    inference_delay = LaunchConfiguration('rl_inference_delay')
    rl_policy_model = LaunchConfiguration('rl_policy_model')
    rl_policy_kind = LaunchConfiguration('rl_policy_kind')
    rl_control_mode = LaunchConfiguration('rl_control_mode')
    rl_policy_device = LaunchConfiguration('rl_policy_device')
    rl_publish_rate = LaunchConfiguration('rl_publish_rate')
    rl_max_neighbors = LaunchConfiguration('rl_max_neighbors')

    recommended_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_sim'),
                'launch',
                'sitl_rl_launch.py',
            ])
        ]),
        launch_arguments={
            'instance': instance,
            'namespace': namespace,
            'fcu_url': fcu_url,
            'rl_policy_enabled': 'true',
            'rl_policy_model': rl_policy_model,
            'rl_policy_kind': rl_policy_kind,
            'rl_control_mode': rl_control_mode,
            'rl_policy_device': rl_policy_device,
            'rl_publish_rate': rl_publish_rate,
            'rl_max_neighbors': rl_max_neighbors,
            'disable_controller_param': disable_controller_param,
            'rl_inference_delay': inference_delay,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('instance', default_value='0', description='SITL instance index.'),
        DeclareLaunchArgument('namespace', default_value='usv_03', description='USV namespace for the current recommended online candidate.'),
        DeclareLaunchArgument('fcu_url', default_value='__auto__', description='FCU URL passed through to sitl_launch.py.'),
        DeclareLaunchArgument('rl_policy_model', default_value=RECOMMENDED_MODEL_RELATIVE_PATH, description='Default recommended policy model path.'),
        DeclareLaunchArgument('rl_policy_kind', default_value='bc', description='Recommended policy type.'),
        DeclareLaunchArgument('rl_control_mode', default_value='pure', description='Deprecated compatibility flag passed to the online inference node.'),
        DeclareLaunchArgument('rl_policy_device', default_value='cpu', description='Inference device for the recommended policy.'),
        DeclareLaunchArgument('rl_publish_rate', default_value='10.0', description='Policy publish rate in Hz.'),
        DeclareLaunchArgument('rl_max_neighbors', default_value='4', description='Maximum number of neighbors encoded into the observation.'),
        DeclareLaunchArgument('disable_controller_param', default_value='false', description='Avoid toggling controller params from the policy node if needed.'),
        DeclareLaunchArgument('rl_inference_delay', default_value='7.0', description='Delay before starting the policy node.'),
        recommended_stack,
    ])