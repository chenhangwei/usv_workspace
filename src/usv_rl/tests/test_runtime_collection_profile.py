import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from usv_rl.config import ActionBounds
from usv_rl.collect_multi_agent_teacher_dataset import _build_env, parse_args
from usv_rl.multi_agent_env import MultiAgentEnvConfig
from usv_rl.runtime_collection_profile import collection_config_from_metadata


class RuntimeCollectionProfileTests(unittest.TestCase):
    def setUp(self):
        self.config = MultiAgentEnvConfig(
            agent_namespaces=('usv_01',), action_mode='speed_scale', action_speed_scale_min=.3,
            cruise_speed=.34, max_angular_velocity=.7, action_bounds=ActionBounds(.3, .7),
        )
        self.metadata = {
            'runtime_action_contract_version': 1, 'action_mode': 'speed_scale', 'obs_dim': 82,
            'checkpoint_angular_authority_floor': .45,
            'action_bounds': {'linear': .34, 'angular': .7},
            'action_contract': {'scale_min': .3, 'cruise_speed': .34, 'minimum_speed': .28,
                                'goal_tolerance': .8, 'relief_distance': 2., 'speed_relief': 0.},
            'projection': {'control_dt': .1, 'heading_omega_deadband': .09, 'heading_omega_reference': .95,
                           'angular_authority_power': 1.3, 'angular_accel_limit': 1.05,
                           'angular_decel_limit': 3., 'conflict_turn_relief': .75,
                           'angular_authority_floor': .35, 'min_forward_speed_floor': 0.},
        }

    def test_uses_effective_runtime_values_without_mutating_defaults(self):
        original_reward = copy.deepcopy(self.config.reward)
        result = collection_config_from_metadata(self.config, self.metadata)
        for name, value in self.metadata['projection'].items():
            self.assertEqual(getattr(result, name), value)
        self.assertEqual(result.angular_authority_floor, .35)
        self.assertEqual(self.config.control_dt, .2)
        self.assertEqual(self.config.min_forward_speed_floor, .08)
        self.assertEqual(result.reward.desired_conflict_speed, .28)
        self.assertEqual(result.reward.conflict_speed_floor, .28)
        self.assertEqual(self.config.reward, original_reward)

    def test_rejects_nonfinite_and_invalid_projection_values(self):
        for name, value in (('control_dt', 0), ('control_dt', float('nan')),
                            ('angular_authority_floor', 2), ('angular_accel_limit', -1),
                            ('heading_omega_reference', .01), ('min_forward_speed_floor', .5)):
            metadata = copy.deepcopy(self.metadata)
            metadata['projection'][name] = value
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                collection_config_from_metadata(self.config, metadata)

    def test_rejects_different_action_units_bounds_and_neighbor_layout(self):
        for name, value in (('action_mode', 'full'), ('obs_dim', 67), ('runtime_action_contract_version', 2)):
            metadata = {**self.metadata, name: value}
            with self.subTest(name=name), self.assertRaises(ValueError):
                collection_config_from_metadata(self.config, metadata)
        self.config.cruise_speed = .5
        with self.assertRaises(ValueError):
            collection_config_from_metadata(self.config, self.metadata)

    def test_collector_imports_trace_header_before_starting_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trace.jsonl'
            path.write_text(json.dumps({**self.metadata, 'type': 'metadata'}) + '\nignored action tail\n')
            argv = ['collector', '--output', str(Path(directory) / 'data.npz'), '--num-agents', '1',
                    '--max-angular-velocity', '.7', '--angular-delta-limit', '.7', '--runtime-policy-trace', str(path)]
            with patch('sys.argv', argv):
                args = parse_args()
            with patch('usv_rl.collect_multi_agent_teacher_dataset.MultiAgentEnv', side_effect=lambda config: config):
                config = _build_env(args, ('solo_navigation',))
            self.assertEqual(config.control_dt, .1)
            self.assertEqual(config.angular_authority_power, 1.3)
            profile = json.loads(args.runtime_collection_profile_json)
            self.assertEqual(profile['scope'], 'projection_and_speed_expansion_only')
        self.config.cruise_speed = .34
        self.config.agent_namespaces = ('usv_01', 'usv_02')
        with self.assertRaises(ValueError):
            collection_config_from_metadata(self.config, self.metadata)


if __name__ == '__main__':
    unittest.main()