import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
import rclpy
import torch

from usv_rl.action_projection import project_policy_action
from usv_rl.config import ActionBounds, RewardConfig
from usv_rl.multi_agent_env import MultiAgentEnv, MultiAgentEnvConfig
from usv_rl.policy_inference_node import MappoActorPolicyRuntime, PolicyInferenceNode, _load_solo_expert
from usv_rl.types import NeighborObservation, UsvObservation
from common_interfaces.msg import NavigationFeedback, NavigationGoal, NavigationResult
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Bool, String


class PolicyInferenceContractTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        actor = torch.nn.Sequential(torch.nn.Linear(82, 2))
        with torch.no_grad():
            actor[0].weight.zero_()
            actor[0].bias.copy_(torch.tensor([0.5, 0.4]))
        self.checkpoint = {
            'local_observation_size': 82, 'action_dim': 2, 'hidden_sizes': [],
            'actor_state_dict': actor.state_dict(), 'action_mode': 'speed_scale',
            'action_speed_scale_min': 0.3, 'cruise_speed': 0.34,
            'max_angular_velocity': 0.7,
            'action_bounds': {'linear_delta': 0.3, 'angular_delta': 0.7},
            'heading_omega_deadband': 0.09, 'heading_omega_reference': 0.95,
            'angular_authority_power': 1.3, 'angular_accel_limit': 1.05,
            'angular_decel_limit': 3.0, 'conflict_turn_relief': 0.75,
            'angular_authority_floor': 0.45,
            'goal_tolerance': 0.8, 'goal_proximity_relief_distance': 3.0,
            'goal_proximity_speed_relief': 0.5,
        }
        self.model_path = str(Path(self.directory.name) / 'policy.pt')
        torch.save(self.checkpoint, self.model_path)

    def observation(self, heading=0.17, distance=10.0):
        return UsvObservation(
            pose_x=0.0, pose_y=0.0, yaw=0.0, speed=0.2,
            distance_to_goal=distance, heading_error=heading,
            raw_linear_x=0.34, raw_angular_z=0.0,
            final_linear_x=0.17, final_angular_z=0.0,
            cross_track_error=0.25, raw_cross_track_error=0.25,
        )

    def test_runtime_matches_environment_before_postprocessing(self):
        policy = MappoActorPolicyRuntime(self.model_path)
        env = MultiAgentEnv.__new__(MultiAgentEnv)
        env.config = MultiAgentEnvConfig(
            action_mode='speed_scale', action_speed_scale_min=0.3,
            cruise_speed=0.34, max_angular_velocity=0.7,
            action_bounds=ActionBounds(0.3, 0.7), control_dt=0.1,
            heading_omega_deadband=0.09, heading_omega_reference=0.95,
            angular_authority_power=1.3, angular_accel_limit=1.05,
            angular_decel_limit=3.0, conflict_turn_relief=0.75,
            angular_authority_floor=0.35, goal_tolerance=0.8,
            goal_proximity_relief_distance=3.0, goal_proximity_speed_relief=0.5,
            reward=RewardConfig(),
        )
        for distance in (0.5, 0.8, 1.9, 3.0, 12.0):
            for scale in (-0.2, 0.3, 0.5, 0.75, 1.0, 1.2):
                for heading in (-1.0, -0.17, 0.0, 0.17, 1.0):
                    with self.subTest(distance=distance, scale=scale, heading=heading):
                        obs = self.observation(heading, distance)
                        env._latest_observations = {'usv_03': obs}
                        action = np.array([scale, math.copysign(0.4, heading)])
                        projected = project_policy_action(
                            action, action_mode='full', linear_delta_limit=policy.action_bounds.linear_delta,
                            angular_delta_limit=policy.action_bounds.angular_delta,
                            heading_error=heading, current_angular_z=0.0, control_dt=0.1,
                            **{name: getattr(policy, name) for name in (
                                'heading_omega_deadband', 'heading_omega_reference', 'angular_authority_power',
                                'angular_accel_limit', 'angular_decel_limit', 'conflict_turn_relief',
                            )},
                        )
                        actual = policy.action_contract.expand(action, projected, distance)
                        expected = env.expand_policy_action('usv_03', action)
                        np.testing.assert_allclose(actual, expected, atol=1e-7)

    def make_node(self, solo_expert_model='', init_ros=True, **kwargs):
        if init_ros:
            rclpy.init()
            self.addCleanup(rclpy.shutdown)
        node = PolicyInferenceNode(
            namespace='contract_test', model_path=self.model_path, policy_kind='mappo',
            rl_control_mode='pure', device='cpu', publish_rate=10.0, max_neighbors=4,
            enable_controller_param=False, policy_trace_dir=self.directory.name,
            solo_expert_model=solo_expert_model, **kwargs,
        )
        self.addCleanup(node.destroy_node)
        node._action_pub = Mock()
        node._encounter_type_pub = Mock()
        node._active_goal_id = 1
        node._route_start = (0.0, 0.0)
        node._route_goal = (10.0, 0.0)
        node._feedback_msg = NavigationFeedback()
        node._feedback_msg.heading_error = 0.17
        node._feedback_msg.distance_to_goal = 10.0
        node._feedback_msg.goal_id = 1
        node._feedback_msg.timestamp = node.get_clock().now().to_msg()
        node._feedback_callback(node._feedback_msg)
        final = PositionTarget()
        final.header.stamp = node.get_clock().now().to_msg()
        node._final_cmd_callback(final)
        return node

    def test_solo_route_features_match_environment_and_preserve_geometry(self):
        node = self.make_node()
        node._pose_msg = PoseStamped()
        node._pose_msg.pose.orientation.w = 1.0
        node._velocity_msg = TwistStamped()
        node._velocity_msg.twist.linear.x = 0.2
        node._final_cmd_msg = PositionTarget()
        node._final_cmd_msg.velocity.x = 0.17
        node._feedback_msg.goal_id = 1
        env = MultiAgentEnv.__new__(MultiAgentEnv)
        env.config = MultiAgentEnvConfig()
        env._scenario_conflict_point = lambda: None
        for agent_id in ('usv_01', 'usv_02', 'usv_03'):
            node._usv_id = agent_id
            env._scenario = SimpleNamespace(
                name='free',
                agent_spawns={agent_id: SimpleNamespace(x=0.0, y=0.0)},
                agent_goals={agent_id: SimpleNamespace(x=10.0, y=0.0)},
            )
            for virtual in (False, True):
                node._virtual_goal_active = virtual
                for position in ((1.0, -1.0), (5.0, 1.0), (9.0, -4.0)):
                    with self.subTest(agent_id=agent_id, virtual=virtual, position=position):
                        node._pose_msg.pose.position.x, node._pose_msg.pose.position.y = position
                        node._neighbor_states = {}
                        solo = node._build_observation()
                        expected = self.observation()
                        expected.pose_x, expected.pose_y = position
                        env._annotate_route_features_for_observations({agent_id: expected})
                        for field in ('conflict_phase', 'conflict_eta', 'crossing_priority',
                                      'crossing_eta_gap', 'cross_track_error', 'raw_cross_track_error',
                                      'cross_track_overflow', 'route_progress'):
                            self.assertAlmostEqual(getattr(solo, field), getattr(expected, field))
                        np.testing.assert_allclose(solo.to_vector(4)[13:17], [0.0, 1.0, 0.0, 1.0])
                        node._neighbor_states = {'peer': SimpleNamespace(
                            usv_id='peer', x=3.0, y=2.0, vx=0.0, vy=0.0,
                        )}
                        with_neighbor = node._build_observation()
                        self.assertAlmostEqual(with_neighbor.conflict_phase, (position[0] - 5.0) / 5.0)
                        along_speed = 0.2 * math.cos(node._feedback_msg.heading_error)
                        self.assertAlmostEqual(with_neighbor.conflict_eta,
                                               max(0.0, 5.0 - position[0]) / along_speed / 25.0)
                        self.assertEqual(with_neighbor.crossing_priority,
                                         {'usv_01': -1.0, 'usv_02': 0.0, 'usv_03': 1.0}[agent_id])
                        with_neighbor.neighbors = []
                        unchanged_slots = np.ones(82, dtype=bool)
                        unchanged_slots[13:16] = False
                        np.testing.assert_allclose(solo.to_vector(4)[unchanged_slots],
                                                   with_neighbor.to_vector(4)[unchanged_slots])

    def refresh_navigation_inputs(self, node):
        feedback = NavigationFeedback()
        feedback.goal_id = node._active_goal_id
        feedback.heading_error = 0.17
        feedback.distance_to_goal = 10.0
        feedback.timestamp = node.get_clock().now().to_msg()
        node._feedback_callback(feedback)
        final = PositionTarget()
        final.header.stamp = node.get_clock().now().to_msg()
        node._final_cmd_callback(final)

    def assert_last_command_stopped(self, node):
        command = node._action_pub.publish.call_args.args[0]
        self.assertEqual(command.twist.linear.x, 0.0)
        self.assertEqual(command.twist.angular.z, 0.0)

    def test_stale_feedback_stops_without_predicting_and_recovers(self):
        node = self.make_node()
        node._build_observation = self.observation
        predict = Mock(wraps=node._policy.predict)
        node._policy.predict = predict
        node._navigation_inputs['feedback']['received'] -= 2.0
        node._publish_action()
        predict.assert_not_called()
        self.assert_last_command_stopped(node)
        records = [json.loads(line) for line in node._policy_trace.path.read_text().splitlines()]
        self.assertEqual(records[-1]['type'], 'inhibited')
        self.assertEqual(records[-1]['navigation_input_status']['reason'], 'feedback_stale_or_missing')
        self.refresh_navigation_inputs(node)
        node._publish_action()
        predict.assert_called_once()
        self.assertAlmostEqual(node._action_pub.publish.call_args.args[0].twist.linear.x, 0.17)

    def test_repeated_feedback_stamp_does_not_refresh_source(self):
        node = self.make_node()
        node._build_observation = self.observation
        node._navigation_inputs['feedback']['updated'] -= 2.0
        node._feedback_callback(node._feedback_msg)
        status = node._navigation_input_status()
        self.assertLess(status['inputs']['feedback']['receipt_age_s'], 0.1)
        self.assertGreater(status['inputs']['feedback']['source_update_age_s'], 1.5)
        node._publish_action()
        self.assert_last_command_stopped(node)
        self.refresh_navigation_inputs(node)
        self.assertIsNone(node._navigation_input_status()['reason'])

    def test_stale_source_stamp_and_missing_final_command_block(self):
        node = self.make_node()
        node._build_observation = self.observation
        node._navigation_inputs['feedback']['stamp'] -= 5.0
        node._publish_action()
        self.assert_last_command_stopped(node)
        self.refresh_navigation_inputs(node)
        node._navigation_inputs['final_command']['received'] -= 2.0
        node._publish_action()
        self.assert_last_command_stopped(node)
        self.assertEqual(node._navigation_input_status()['reason'], 'final_command_stale_or_missing')

    def test_future_or_zero_stamp_cannot_poison_recovery(self):
        node = self.make_node()
        node._navigation_inputs.pop('feedback')
        feedback = NavigationFeedback()
        feedback.goal_id = 1
        node._feedback_callback(feedback)
        self.assertNotIn('feedback', node._navigation_inputs)
        feedback.timestamp = node.get_clock().now().to_msg()
        feedback.timestamp.sec += 60
        node._feedback_callback(feedback)
        self.assertNotIn('feedback', node._navigation_inputs)
        self.refresh_navigation_inputs(node)
        self.assertIsNone(node._navigation_input_status()['reason'])

    def test_completion_blocks_late_feedback_until_explicit_new_goal(self):
        node = self.make_node()
        old_feedback = node._feedback_msg
        result = NavigationResult()
        result.goal_id = 1
        result.success = True
        node._nav_result_callback(result)
        self.assertIsNone(node._active_goal_id)
        self.assert_last_command_stopped(node)
        old_feedback.timestamp = node.get_clock().now().to_msg()
        node._feedback_callback(old_feedback)
        self.assertIsNone(node._active_goal_id)
        goal = NavigationGoal()
        goal.goal_id = 2
        goal.target_pose.pose.position.x = 10.0
        node._nav_goal_callback(goal)
        node._nav_result_callback(result)
        node._feedback_callback(old_feedback)
        self.assertEqual(node._active_goal_id, 2)
        self.assertIsNone(node._feedback_msg)
        self.refresh_navigation_inputs(node)
        self.assertIsNone(node._navigation_input_status()['reason'])

    def test_stop_and_failure_latch_but_pass_through_does_not(self):
        node = self.make_node()
        result = NavigationResult()
        result.goal_id = 1
        result.success = True
        result.message = '\u5df2\u901a\u8fc7\u822a\u70b9'
        node._nav_result_callback(result)
        self.assertEqual(node._active_goal_id, 1)
        result.success = False
        node._nav_result_callback(result)
        self.assertEqual(node._navigation_terminal_reason, 'navigation_failed')
        self.assert_last_command_stopped(node)
        goal = NavigationGoal()
        goal.goal_id = 2
        node._nav_goal_callback(goal)
        stop = Bool()
        node._stop_navigation_callback(stop)
        self.assertEqual(node._active_goal_id, 2)
        stop.data = True
        node._stop_navigation_callback(stop)
        self.assertEqual(node._navigation_terminal_reason, 'navigation_stopped')
        self.assert_last_command_stopped(node)

    def test_ros_feedback_loss_recovery_and_result_stop(self):
        import time

        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data

        node = self.make_node()
        node._build_observation = self.observation
        node._navigation_inputs.clear()
        node._action_pub = node.create_publisher(TwistStamped, 'rl_policy/cmd_vel', qos_profile_sensor_data)
        peer = Node('feedback_fixture', namespace='contract_test')
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(peer)
        commands = []
        feedback_pub = peer.create_publisher(NavigationFeedback, 'navigation_feedback', qos_profile_sensor_data)
        final_pub = peer.create_publisher(PositionTarget, 'setpoint_raw/local', qos_profile_sensor_data)
        result_pub = peer.create_publisher(NavigationResult, 'navigation_result', 10)
        peer.create_subscription(TwistStamped, 'rl_policy/cmd_vel', commands.append, qos_profile_sensor_data)

        def feedback_tick():
            feedback = NavigationFeedback()
            feedback.goal_id = 1
            feedback.distance_to_goal = 10.0
            feedback.heading_error = 0.17
            feedback.timestamp = peer.get_clock().now().to_msg()
            feedback_pub.publish(feedback)

        def final_tick():
            command = PositionTarget()
            command.header.stamp = peer.get_clock().now().to_msg()
            final_pub.publish(command)

        def wait_until(predicate):
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
                if predicate():
                    return
            self.fail('ROS navigation guard transition timed out')

        feedback_timer = peer.create_timer(0.1, feedback_tick)
        peer.create_timer(0.1, final_tick)
        try:
            wait_until(lambda: commands and commands[-1].twist.linear.x > 0.0)
            feedback_timer.cancel()
            wait_until(lambda: commands and commands[-1].twist.linear.x == 0.0
                       and node._navigation_guard_reason == 'feedback_stale_or_missing')
            self.assertEqual(commands[-1].twist.angular.z, 0.0)
            feedback_timer.reset()
            wait_until(lambda: commands and commands[-1].twist.linear.x > 0.0
                       and node._navigation_guard_reason is None)
            result = NavigationResult()
            result.goal_id = 1
            result.success = True
            result_pub.publish(result)
            wait_until(lambda: commands and commands[-1].twist.linear.x == 0.0
                       and node._navigation_terminal_reason == 'navigation_completed')
            final_deadline = time.monotonic() + 0.3
            while time.monotonic() < final_deadline:
                executor.spin_once(timeout_sec=0.02)
            self.assertIsNone(node._active_goal_id)
            self.assertEqual(commands[-1].twist.linear.x, 0.0)
        finally:
            executor.shutdown()
            peer.destroy_node()

    def test_node_records_actual_actions_and_turn_assist(self):
        node = self.make_node(safety_layers_enabled=True)
        obs = self.observation()
        node._build_observation = lambda: obs
        node._publish_action()
        first = node._action_pub.publish.call_args.args[0]
        self.assertAlmostEqual(first.twist.linear.x, 0.17)
        obs.heading_error = math.radians(60.0)
        node._publish_action()
        final = node._action_pub.publish.call_args.args[0]
        records = [json.loads(line) for line in node._policy_trace.path.read_text().splitlines()]
        self.assertEqual(records[0]['runtime_action_contract_version'], 1)
        self.assertTrue(records[0]['solo_conflict_features_neutralized'])
        self.assertEqual(records[0]['navigation_input_guard_version'], 1)
        self.assertIsNone(records[1]['navigation_input_status']['reason'])
        self.assertEqual(records[0]['action_mode'], 'speed_scale')
        self.assertEqual(records[0]['checkpoint_angular_authority_floor'], 0.45)
        self.assertEqual(records[0]['projection']['angular_authority_floor'], 0.35)
        self.assertEqual(len(records[1]['observation_vector']), 82)
        self.assertAlmostEqual(records[1]['actor_action'][0], 0.5)
        self.assertAlmostEqual(records[1]['projected_action'][0], 0.34)
        self.assertAlmostEqual(records[1]['expanded_action'][0], 0.17)
        self.assertFalse(records[1]['turn_assist_active'])
        self.assertTrue(records[2]['turn_assist_active'])
        self.assertEqual(records[2]['published_action'], [final.twist.linear.x, final.twist.angular.z])
        self.assertGreater(final.twist.angular.z, records[2]['expanded_action'][1])
        self.assertIsNone(records[2]['nearest_neighbor_distance'])

    def solo_expert_path(self, **overrides):
        actor = torch.nn.Sequential(torch.nn.Linear(82, 2))
        with torch.no_grad():
            actor[0].weight.zero_()
            actor[0].bias.copy_(torch.tensor([0.9, -0.3]))
        path = Path(self.directory.name) / f'solo_{abs(hash(repr(sorted(overrides.items()))))}.pt'
        torch.save({**self.checkpoint, 'actor_state_dict': actor.state_dict(), **overrides}, path)
        return str(path)

    def test_solo_expert_routes_unless_neighbour_conflict(self):
        node = self.make_node(self.solo_expert_path())
        obs = self.observation(heading=0.0)
        node._build_observation = lambda: obs
        node._publish_action()
        obs.neighbors = [NeighborObservation(
            usv_id='peer', rel_x=6.0, rel_y=0.0, rel_vx=0.0, rel_vy=0.0,
            distance=6.0, bearing=0.0, role_index=0,
        )]
        node._publish_action()
        obs.neighbors = [NeighborObservation(
            usv_id='peer', rel_x=4.0, rel_y=0.0, rel_vx=-0.45, rel_vy=0.0,
            distance=4.0, bearing=0.0, role_index=0,
        )]
        node._publish_action()
        records = [json.loads(line) for line in node._policy_trace.path.read_text().splitlines()]
        self.assertTrue(records[0]['solo_expert']['enabled'])
        self.assertEqual(records[1]['policy_expert'], 'solo')
        np.testing.assert_allclose(records[1]['actor_action'], [0.9, -0.3], atol=1e-6)
        self.assertEqual(records[2]['policy_expert'], 'solo')
        self.assertEqual(records[2]['observation_vector'], records[1]['observation_vector'])
        self.assertEqual(records[3]['policy_expert'], 'base')
        np.testing.assert_allclose(records[3]['actor_action'], [0.5, 0.4], atol=1e-6)

    def test_default_node_never_routes_and_rejects_mismatched_expert(self):
        node = self.make_node()
        node._build_observation = self.observation
        node._publish_action()
        records = [json.loads(line) for line in node._policy_trace.path.read_text().splitlines()]
        self.assertFalse(records[0]['solo_expert']['enabled'])
        self.assertEqual(records[1]['policy_expert'], 'base')
        for overrides in ({'cruise_speed': 0.5}, {'angular_accel_limit': 2.0}, {'action_speed_scale_min': 0.2}):
            with self.subTest(overrides=overrides), self.assertRaises(RuntimeError):
                _load_solo_expert(self.solo_expert_path(**overrides), 'cpu', MappoActorPolicyRuntime(self.model_path))

    def test_canonical_expert_sees_shared_route_frame_and_base_stays_legacy(self):
        from usv_rl.route_frame import canonical_route_fields
        node = self.make_node(self.solo_expert_path(observation_frame='route_canonical_v1'))
        node._route_start = (20.0, -1.0)
        node._route_goal = (-5.0, -12.0)
        node._pose_msg = PoseStamped()
        node._pose_msg.pose.position.x = 9.0
        node._pose_msg.pose.position.y = -3.0
        node._pose_msg.pose.orientation.z = math.sin(1.4)
        node._pose_msg.pose.orientation.w = math.cos(1.4)
        node._velocity_msg = TwistStamped()
        node._velocity_msg.twist.linear.x = 0.1
        node._final_cmd_msg = PositionTarget()
        node._feedback_msg.distance_to_goal = 14.5
        node._feedback_msg.heading_error = 0.6
        node._virtual_goal_active = True
        legacy = node._build_observation()
        node._publish_action()
        records = [json.loads(line) for line in node._policy_trace.path.read_text().splitlines()]
        self.assertEqual(records[0]['solo_expert']['observation_frame'], 'route_canonical_v1')
        self.assertEqual(records[1]['policy_expert'], 'solo')
        expected = canonical_route_fields(
            pose_x=9.0, pose_y=-3.0, yaw=2.8, feedback_distance=14.5, feedback_heading_error=0.6,
            route_start=(20.0, -1.0), route_goal=(-5.0, -12.0))
        vector = records[1]['observation_vector']
        for index, name in ((0, 'pose_x'), (1, 'pose_y'), (2, 'yaw'), (4, 'distance_to_goal'), (12, 'route_progress')):
            self.assertAlmostEqual(vector[index], expected[name], places=5, msg=name)
        self.assertAlmostEqual(math.atan2(vector[5], vector[6]), expected['heading_error'], places=5)
        self.assertAlmostEqual(vector[17], legacy.raw_cross_track_error, places=5)
        self.assertNotAlmostEqual(vector[0], legacy.pose_x, places=3)
        node._route_start = None
        node._publish_action()
        records = [json.loads(line) for line in node._policy_trace.path.read_text().splitlines()]
        self.assertEqual(records[-1]['policy_expert'], 'base')

    def test_unknown_observation_frame_rejected(self):
        with self.assertRaises(RuntimeError):
            MappoActorPolicyRuntime(self.solo_expert_path(observation_frame='world_v9'))

    def test_environment_and_node_build_identical_canonical_vectors(self):
        from usv_rl.multi_agent_scenarios import AgentGoalConfig, AgentSpawnConfig, FleetScenario
        from usv_rl.multi_agent_types import AgentLocalObservation
        node = self.make_node(self.solo_expert_path(observation_frame='route_canonical_v1'))
        node._route_start, node._route_goal = (20.0, -1.0), (-5.0, -12.0)
        node._pose_msg = PoseStamped()
        node._pose_msg.pose.position.x, node._pose_msg.pose.position.y = 9.0, -3.0
        node._pose_msg.pose.orientation.z, node._pose_msg.pose.orientation.w = math.sin(1.4), math.cos(1.4)
        node._velocity_msg = TwistStamped()
        node._velocity_msg.twist.linear.x = 0.1
        node._final_cmd_msg = PositionTarget()
        node._feedback_msg.distance_to_goal, node._feedback_msg.heading_error = 14.5, 0.6
        node._publish_action()
        node_vector = np.asarray(json.loads(node._policy_trace.path.read_text().splitlines()[-1])['observation_vector'])
        legacy = node._build_observation()
        env = MultiAgentEnv.__new__(MultiAgentEnv)
        env.config = MultiAgentEnvConfig(agent_namespaces=('usv_03',), observation_frame='route_canonical_v1')
        env._active_agent_ids = ('usv_03',)
        env._scenario = FleetScenario(
            name='solo_navigation', duration=40.0,
            agent_spawns={'usv_03': AgentSpawnConfig(20.0, -1.0, 0.0)},
            agent_goals={'usv_03': AgentGoalConfig(-5.0, -12.0)})
        observation = AgentLocalObservation(
            agent_id='usv_03', pose_x=9.0, pose_y=-3.0, yaw=2.8, speed=legacy.speed,
            distance_to_goal=14.5, heading_error=0.6, raw_linear_x=legacy.raw_linear_x,
            raw_angular_z=legacy.raw_angular_z, final_linear_x=legacy.final_linear_x,
            final_angular_z=legacy.final_angular_z, cross_track_error=legacy.cross_track_error,
            raw_cross_track_error=legacy.raw_cross_track_error, cross_track_overflow=legacy.cross_track_overflow,
            conflict_eta=legacy.conflict_eta, crossing_eta_gap=legacy.crossing_eta_gap)
        env._apply_canonical_route_frame({'usv_03': observation})
        np.testing.assert_allclose(observation.to_vector(4), node_vector, atol=1e-5)

    def test_default_publishes_model_action_without_safety_layers(self):
        node = self.make_node()
        obs = self.observation(heading=0.0)
        obs.neighbors = [NeighborObservation(
            usv_id='peer', rel_x=2.0, rel_y=0.0, rel_vx=0.0, rel_vy=0.0,
            distance=2.0, bearing=0.0, role_index=0,
        )]
        node._build_observation = lambda: obs
        node._publish_action()
        record = json.loads(node._policy_trace.path.read_text().splitlines()[-1])
        self.assertEqual(record['published_action'], record['expanded_action'])

    def test_node_preserves_close_neighbor_postprocessing(self):
        node = self.make_node(safety_layers_enabled=True)
        obs = self.observation(heading=0.0)
        obs.neighbors = [NeighborObservation(
            usv_id='peer', rel_x=2.0, rel_y=0.0, rel_vx=0.0, rel_vy=0.0,
            distance=2.0, bearing=0.0, role_index=0,
        )]
        node._build_observation = lambda: obs
        node._publish_action()
        final = node._action_pub.publish.call_args.args[0]
        self.assertAlmostEqual(final.twist.linear.x, 0.17 * 2.0 / 3.0)
        self.assertFalse(node._turn_assist_active)

    def test_router_leaves_every_conflict_step_identical(self):
        published = {}
        experts = (('plain', ''), ('routed', self.solo_expert_path(observation_frame='route_canonical_v1')))
        for index, (name, expert) in enumerate(experts):
            node = self.make_node(expert, init_ros=index == 0)
            commands = []
            for heading, distance in ((0.0, 2.0), (0.9, 4.0), (-1.6, 5.0), (2.5, 1.2)):
                obs = self.observation(heading=heading)
                obs.neighbors = [NeighborObservation(
                    usv_id='peer', rel_x=distance, rel_y=0.5, rel_vx=-0.45, rel_vy=0.0,
                    distance=distance, bearing=0.2, role_index=0,
                )]
                node._build_observation = lambda obs=obs: obs
                node._publish_action()
                message = node._action_pub.publish.call_args.args[0]
                commands.append((message.twist.linear.x, message.twist.angular.z))
            published[name] = commands
        self.assertEqual(published['plain'], published['routed'])

    def test_telemetry_status_is_recorded_without_changing_control(self):
        node = self.make_node()
        node._build_observation = self.observation
        status = {'healthy': False, 'sources': {'pose': {'source_hz': 0.53}}}
        message = String()
        message.data = json.dumps(status)
        node._telemetry_status_callback(message)
        node._publish_action()
        records = [json.loads(line) for line in node._policy_trace.path.read_text().splitlines()]
        self.assertEqual(records[-1]['telemetry_status'], status)
        self.assertLess(records[-1]['telemetry_status_age_s'], 1.0)
        self.assertAlmostEqual(node._action_pub.publish.call_args.args[0].twist.linear.x, 0.17)
        message.data = '{"healthy":true,"value":NaN}'
        node._telemetry_status_callback(message)
        self.assertEqual(node._telemetry_status, status)


if __name__ == '__main__':
    unittest.main()