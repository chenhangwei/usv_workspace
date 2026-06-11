from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_policy_node(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    rl_policy_model = LaunchConfiguration('rl_policy_model').perform(context)
    rl_policy_kind = LaunchConfiguration('rl_policy_kind').perform(context)
    rl_control_mode = LaunchConfiguration('rl_control_mode').perform(context)
    rl_policy_device = LaunchConfiguration('rl_policy_device').perform(context)
    rl_publish_rate = LaunchConfiguration('rl_publish_rate').perform(context)
    rl_max_neighbors = LaunchConfiguration('rl_max_neighbors').perform(context)
    disable_controller_param = LaunchConfiguration('disable_controller_param').perform(context)
    rl_encounter_type = LaunchConfiguration('rl_encounter_type').perform(context)
    clear_ahead_route_gate = LaunchConfiguration('clear_ahead_route_gate').perform(context)

    node_args = [
        '--namespace', namespace,
        '--model', rl_policy_model,
        '--policy', rl_policy_kind,
        '--rl-control-mode', rl_control_mode,
        '--device', rl_policy_device,
        '--publish-rate', rl_publish_rate,
        '--max-neighbors', rl_max_neighbors,
        '--encounter-type', rl_encounter_type,
    ]
    if disable_controller_param.lower() in ('1', 'true', 'yes', 'on'):
        node_args.append('--disable-controller-param')
    if clear_ahead_route_gate.lower() in ('1', 'true', 'yes', 'on'):
        node_args.append('--clear-ahead-route-gate')

    return [
        Node(
            package='usv_rl',
            executable='policy_inference_node',
            name='policy_inference_node',
            namespace=namespace,
            output='screen',
            arguments=node_args,
            condition=IfCondition(LaunchConfiguration('rl_policy_enabled')),
        )
    ]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    instance = LaunchConfiguration('instance')
    fcu_url = LaunchConfiguration('fcu_url')
    inference_delay = LaunchConfiguration('rl_inference_delay')
    enable_log_collector = LaunchConfiguration('enable_log_collector')

    sitl_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_sim'),
                'launch',
                'sitl_launch.py',
            ])
        ]),
        launch_arguments={
            'instance': instance,
            'namespace': namespace,
            'fcu_url': fcu_url,
            'enable_log_collector': enable_log_collector,
        }.items(),
    )

    delayed_policy_node = TimerAction(
        period=inference_delay,
        actions=[OpaqueFunction(function=_launch_policy_node)],
        condition=IfCondition(LaunchConfiguration('rl_policy_enabled')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'instance',
            default_value='0',
            description='SITL instance index; ignored when namespace implies a USV id.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='usv_03',
            description='USV namespace for the SITL stack and RL policy node.',
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='__auto__',
            description='FCU URL passed through to sitl_launch.py.',
        ),
        DeclareLaunchArgument(
            'rl_policy_enabled',
            default_value='true',
            description='Whether to launch the policy inference node.',
        ),
        DeclareLaunchArgument(
            'rl_policy_model',
            default_value='',
            description='Absolute or workspace-relative model path for the RL policy.',
        ),
        DeclareLaunchArgument(
            'rl_policy_kind',
            default_value='auto',
            description='Policy type: auto, bc, ppo, mappo, or zero.',
        ),
        DeclareLaunchArgument(
            'rl_control_mode',
            default_value='pure',
            description='RL control mode. Pure policy mode publishes final commands directly.',
        ),
        DeclareLaunchArgument(
            'rl_policy_device',
            default_value='cpu',
            description='Inference device for PPO or MAPPO policies.',
        ),
        DeclareLaunchArgument(
            'rl_publish_rate',
            default_value='10.0',
            description='Policy action publish rate in Hz.',
        ),
        DeclareLaunchArgument(
            'rl_max_neighbors',
            default_value='2',
            description='Maximum number of neighbors encoded into the observation.',
        ),
        DeclareLaunchArgument(
            'disable_controller_param',
            default_value='false',
            description='Pass through to policy_inference_node to avoid toggling controller parameters.',
        ),
        DeclareLaunchArgument(
            'enable_log_collector',
            default_value='true',
            description='Whether to launch the per-USV navigation log collector.',
        ),
        DeclareLaunchArgument(
            'rl_inference_delay',
            default_value='7.0',
            description='Delay before starting the policy node so the SITL control stack is up.',
        ),
        DeclareLaunchArgument(
            'rl_encounter_type',
            default_value='auto',
            description='Encounter type for scenario-conditioned models (auto, none, head_on, crossing, overtaking).',
        ),
        DeclareLaunchArgument(
            'clear_ahead_route_gate',
            default_value='false',
            description='Enable online clear-ahead route gate in policy_inference_node.',
        ),
        sitl_stack,
        delayed_policy_node,
    ])