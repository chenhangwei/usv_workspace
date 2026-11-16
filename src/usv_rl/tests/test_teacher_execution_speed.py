import math
import unittest
from unittest.mock import patch

import numpy as np

from usv_rl.collect_multi_agent_teacher_dataset import _cap_execution_speed, _episode_spec, parse_args


class TeacherExecutionSpeedTests(unittest.TestCase):
    def parse(self, *options):
        with patch('sys.argv', ['collector', '--output', '/tmp/unused.npz', *options]):
            return parse_args()

    def test_defaults_disable_speed_perturbation(self):
        args = self.parse()
        self.assertEqual(args.exec_speed_scale_min, 1.0)
        self.assertEqual(args.exec_speed_scale_max, 1.0)
        self.assertEqual(args.solo_recovery_progress, 0.0)

    def test_cap_preserves_teacher_labels_and_omega(self):
        teacher = np.asarray([1.0, .45], dtype=np.float32)
        executed = _cap_execution_speed(teacher, .35)
        np.testing.assert_allclose(executed, [.35, .45])
        np.testing.assert_allclose(teacher, [1.0, .45])
        self.assertFalse(np.shares_memory(teacher, executed))

    def test_cap_does_not_increase_student_throttle(self):
        np.testing.assert_allclose(_cap_execution_speed([.3, -.7], .5), [.3, -.7])

    def test_accepts_single_agent_with_82_feature_layout(self):
        args = self.parse('--num-agents', '1', '--max-neighbors', '4',
                          '--exec-speed-scale-min', '.3', '--exec-speed-scale-max', '.4')
        self.assertEqual(args.num_agents, 1)
        self.assertEqual(args.max_neighbors, 4)
        self.assertAlmostEqual(args.exec_speed_scale_min, .3)

    def test_invalid_caps_fail_before_environment_creation(self):
        for options in (
            ('--exec-speed-scale-min', '.2'),
            ('--exec-speed-scale-min', '.8', '--exec-speed-scale-max', '.5'),
            ('--exec-speed-scale-max', '1.1'),
            ('--exec-speed-scale-min', 'nan'),
            ('--exec-speed-scale-max', 'inf'),
            ('--exec-speed-hold', '0'),
            ('--action-mode', 'full', '--exec-speed-scale-min', '.3'),
            ('--action-mode', 'angular_only', '--exec-speed-scale-min', '.3'),
        ):
            with self.subTest(options=options), self.assertRaises(SystemExit), patch('sys.stderr'):
                self.parse(*options)

    def test_recovery_pairs_stay_in_one_split_with_both_scenarios(self):
        args = self.parse('--num-agents', '1', '--episodes', '8', '--solo-recovery-cte', '1',
                          '--validation-every-blocks', '2', '--solo-recovery-progress', '.7')
        scenarios = ('solo_navigation', 'waypoint_turn')
        specs = [_episode_spec(args, scenarios, episode) for episode in range(8)]
        for episode in range(0, 8, 2):
            left, right = specs[episode:episode + 2]
            self.assertEqual(left[:2], right[:2])
            self.assertEqual(left[4:], right[4:])
            self.assertEqual(left[2]['solo_recovery_cte'], -right[2]['solo_recovery_cte'])
            self.assertEqual(left[2]['solo_recovery_heading_error'], -right[2]['solo_recovery_heading_error'])
            self.assertEqual(left[2]['solo_recovery_progress'], .7)
            self.assertEqual(right[2]['solo_recovery_progress'], .7)
        train = [spec for spec in specs if spec[-1] == 'train']
        validation = [spec for spec in specs if spec[-1] == 'validation']
        self.assertTrue({spec[0] for spec in train}.isdisjoint({spec[0] for spec in validation}))
        self.assertEqual({spec[1] for spec in train}, set(scenarios))
        self.assertEqual({spec[1] for spec in validation}, set(scenarios))

    def test_default_episode_schedule_is_unchanged(self):
        args = self.parse('--seed', '77')
        self.assertEqual(args.observation_frame, 'legacy')
        self.assertEqual(args.goal_distance, 12.0)
        self.assertEqual(args.waypoints_per_episode, 1)
        self.assertAlmostEqual(math.radians(args.waypoint_turn_max_deg), 2.9)
        self.assertAlmostEqual(math.radians(args.waypoint_switch_max_deg), math.pi * 5 / 6)
        self.assertEqual(self.parse('--num-agents', '1', '--observation-frame', 'route_canonical_v1',
                                    '--goal-distance', '28').goal_distance, 28.0)
        self.assertEqual(_episode_spec(args, ('solo_navigation',), 3),
                         (80, 'solo_navigation', {'scenario_kind': 'solo_navigation'}, 0, 3, 'train'))

    def test_invalid_recovery_configuration_rejected(self):
        for options in (
            ('--solo-recovery-cte', '1'),
            ('--num-agents', '1', '--episodes', '3', '--solo-recovery-cte', '1'),
            ('--solo-recovery-cte', 'nan'),
            ('--solo-recovery-cte', '-1'),
            ('--solo-recovery-heading-deg', 'nan'),
            ('--solo-recovery-heading-deg', '90'),
            ('--solo-recovery-progress', 'nan'),
            ('--solo-recovery-progress', 'inf'),
            ('--solo-recovery-progress', '-.1'),
            ('--solo-recovery-progress', '1'),
            ('--solo-recovery-progress', '.7'),
            ('--observation-frame', 'route_canonical_v1'),
            ('--num-agents', '1', '--observation-frame', 'world'),
            ('--goal-distance', '0'),
            ('--goal-distance', 'nan'),
            ('--waypoints-per-episode', '0'),
            ('--waypoint-turn-max-deg', '190'),
            ('--waypoint-switch-max-deg', '30'),
            ('--validation-every-blocks', '1'),
            ('--validation-every-blocks', '2'),
        ):
            with self.subTest(options=options), self.assertRaises(SystemExit), patch('sys.stderr'):
                self.parse(*options)


if __name__ == '__main__':
    unittest.main()