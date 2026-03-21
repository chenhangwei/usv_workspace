from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    model_path = LaunchConfiguration('model')
    policy_kind = LaunchConfiguration('policy')
    rl_control_mode = LaunchConfiguration('rl_control_mode')
    disable_controller_param = LaunchConfiguration('disable_controller_param')
    device = LaunchConfiguration('device')
    publish_rate = LaunchConfiguration('publish_rate')
    max_neighbors = LaunchConfiguration('max_neighbors')
    encounter_enabled = LaunchConfiguration('encounter_enabled')
    encounter_scenario = LaunchConfiguration('encounter_scenario')
    encounter_goal_distance = LaunchConfiguration('encounter_goal_distance')
    encounter_neighbor_speed = LaunchConfiguration('encounter_neighbor_speed')
    encounter_publish_rate = LaunchConfiguration('encounter_publish_rate')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    goal_nav_mode = LaunchConfiguration('goal_nav_mode')
    goal_delay = LaunchConfiguration('goal_delay')

    param_file = PathJoinSubstitution([
        FindPackageShare('usv_bringup'),
        'config',
        'usv_params.yaml',
    ])
    sitl_param_file = PathJoinSubstitution([
        FindPackageShare('usv_bringup'),
        'config',
        'usv_params_sitl.yaml',
    ])

    simple_sim_node = Node(
        package='usv_rl',
        executable='simple_sim_node',
        name='simple_sim_node',
        namespace=namespace,
        output='screen',
        arguments=['--namespace', namespace],
    )

    navigate_to_point_node = Node(
        package='usv_comm',
        executable='navigate_to_point_node',
        name='navigate_to_point_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file, sitl_param_file],
    )

    velocity_controller_node = Node(
        package='usv_control',
        executable='velocity_controller_node',
        name='velocity_controller_node',
        namespace=namespace,
        output='screen',
        parameters=[
            param_file,
            sitl_param_file,
            {
                'rl_policy_enabled': True,
            },
        ],
    )

    common_policy_arguments = [
        '--namespace', namespace,
        '--model', model_path,
        '--policy', policy_kind,
        '--rl-control-mode', rl_control_mode,
        '--device', device,
        '--publish-rate', publish_rate,
        '--max-neighbors', max_neighbors,
    ]

    policy_node = Node(
        package='usv_rl',
        executable='policy_inference_node',
        name='policy_inference_node',
        namespace=namespace,
        output='screen',
        arguments=common_policy_arguments + ['--disable-controller-param'],
        condition=IfCondition(disable_controller_param),
    )

    policy_node_with_controller_param = Node(
        package='usv_rl',
        executable='policy_inference_node',
        name='policy_inference_node',
        namespace=namespace,
        output='screen',
        arguments=common_policy_arguments,
        condition=UnlessCondition(disable_controller_param),
    )

    synthetic_neighbors_node = Node(
        package='usv_rl',
        executable='publish_synthetic_neighbors',
        name='publish_synthetic_neighbors',
        namespace=namespace,
        output='screen',
        arguments=[
            '--namespace', namespace,
            '--scenario', encounter_scenario,
            '--goal-distance', encounter_goal_distance,
            '--neighbor-speed', encounter_neighbor_speed,
            '--publish-rate', encounter_publish_rate,
        ],
        condition=IfCondition(encounter_enabled),
    )

    goal_publisher_node = Node(
        package='usv_rl',
        executable='publish_nav_goal_once',
        name='publish_nav_goal_once',
        namespace=namespace,
        output='screen',
        arguments=[
            '--namespace', namespace,
            '--x', goal_x,
            '--y', goal_y,
            '--yaw', goal_yaw,
            '--nav-mode', goal_nav_mode,
        ],
    )

    delayed_policy = TimerAction(period=1.5, actions=[policy_node, policy_node_with_controller_param])
    delayed_goal = TimerAction(period=goal_delay, actions=[goal_publisher_node])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='usv_03', description='USV namespace for validation.'),
        DeclareLaunchArgument('model', description='Policy model path (.npz, .zip, or .pt).'),
        DeclareLaunchArgument('policy', default_value='bc', description='Policy backend: auto, bc, ppo, mappo, zero.'),
        DeclareLaunchArgument('rl_control_mode', default_value='pure', description='Deprecated compatibility flag. Pure final-command control is used.'),
        DeclareLaunchArgument('disable_controller_param', default_value='false', description='Avoid toggling controller params from the policy node if needed.'),
        DeclareLaunchArgument('device', default_value='cpu', description='Inference device for PPO or MAPPO policies.'),
        DeclareLaunchArgument('publish_rate', default_value='10.0', description='Policy publish rate in Hz.'),
        DeclareLaunchArgument('max_neighbors', default_value='4', description='Max neighbors encoded into the observation.'),
        DeclareLaunchArgument('encounter_enabled', default_value='false', description='Whether to publish synthetic neighbors for an encounter scenario.'),
        DeclareLaunchArgument('encounter_scenario', default_value='head_on', description='Synthetic encounter scenario: head_on, crossing_starboard, overtaking.'),
        DeclareLaunchArgument('encounter_goal_distance', default_value='8.0', description='Goal distance used when constructing the synthetic encounter geometry.'),
        DeclareLaunchArgument('encounter_neighbor_speed', default_value='0.3', description='Synthetic neighbor speed in m/s.'),
        DeclareLaunchArgument('encounter_publish_rate', default_value='10.0', description='Synthetic neighbor publish rate in Hz.'),
        DeclareLaunchArgument('goal_x', default_value='8.0', description='Validation goal X in meters.'),
        DeclareLaunchArgument('goal_y', default_value='2.0', description='Validation goal Y in meters.'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0', description='Validation goal yaw in radians.'),
        DeclareLaunchArgument('goal_nav_mode', default_value='0', description='NavigationGoal.nav_mode for validation. Default 0 keeps control in velocity_controller_node.'),
        DeclareLaunchArgument('goal_delay', default_value='2.5', description='Delay before publishing the validation navigation goal.'),
        simple_sim_node,
        navigate_to_point_node,
        velocity_controller_node,
        synthetic_neighbors_node,
        delayed_policy,
        delayed_goal,
    ])