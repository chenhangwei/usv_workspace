import unittest

import numpy as np

from usv_rl.runtime_action_contract import RuntimeActionContract


class RuntimeActionContractTests(unittest.TestCase):
    def test_speed_scale_preserves_throttle(self):
        contract = RuntimeActionContract({'action_mode': 'speed_scale', 'action_speed_scale_min': 0.3}, 0.34)
        for scale in (0.3, 0.5, 0.75, 1.0):
            with self.subTest(scale=scale):
                actual = contract.expand([scale, 0.4], [min(scale, 0.34), 0.1788078], 10.0)
                np.testing.assert_allclose(actual, [scale * 0.34, 0.1788078], atol=1e-7)

    def test_clips_scale_and_preserves_projected_omega(self):
        contract = RuntimeActionContract({'action_mode': 'speed_scale', 'action_speed_scale_min': 0.3}, 0.34)
        for scale, expected in ((-1.0, 0.102), (2.0, 0.34)):
            actual = contract.expand([scale, -0.7], [0.0, -0.12], 10.0)
            np.testing.assert_allclose(actual, [expected, -0.12], atol=1e-7)

    def test_goal_proximity_relief(self):
        contract = RuntimeActionContract({
            'action_mode': 'speed_scale', 'goal_tolerance': 0.8,
            'goal_proximity_relief_distance': 3.0, 'goal_proximity_speed_relief': 0.5,
            'reward_config': {'conflict_speed_floor': 0.08, 'desired_conflict_speed': 0.08},
        }, 0.34)
        actual = contract.expand([0.5, 0.4], [0.34, 0.1], 1.9)
        self.assertAlmostEqual(float(actual[0]), 0.5 * 0.34 * 0.75)

    def test_full_mode_is_unchanged_and_does_not_mutate(self):
        contract = RuntimeActionContract({}, 0.34)
        projected = np.array([0.12, -0.17], dtype=np.float32)
        result = contract.expand([0.5, -0.7], projected, 0.5)
        np.testing.assert_array_equal(result, projected)
        self.assertIsNot(result, projected)

    def test_unsupported_mode_fails_explicitly(self):
        with self.assertRaisesRegex(ValueError, 'Unsupported runtime action_mode'):
            RuntimeActionContract({'action_mode': 'angular_only'}, 0.34)


if __name__ == '__main__':
    unittest.main()