import unittest

import numpy as np
import torch

from usv_rl.evaluate_mappo_policy import MappoActorPolicy
from usv_rl.policy_inference_node import MappoActorPolicyRuntime


class SaturatedActor(torch.nn.Module):
    def forward(self, observation):
        return observation[:, :2]


def configure(policy):
    policy._torch = torch
    policy._device = torch.device('cpu')
    policy._actor = SaturatedActor()
    policy._obs_normalizer = None
    policy._squash_actions = True
    policy.obs_dim = 2
    return policy


class SquashedActionParityTests(unittest.TestCase):
    def test_collection_and_runtime_clamp_raw_actions_before_tanh(self):
        evaluator = configure(object.__new__(MappoActorPolicy))
        evaluator._action_low = torch.tensor([.3, -.7])
        evaluator._action_high = torch.tensor([1., .7])
        runtime = configure(object.__new__(MappoActorPolicyRuntime))
        runtime._sq_action_low = evaluator._action_low
        runtime._sq_action_high = evaluator._action_high
        for raw in ([3.5, -4.2], [-9., 8.], [.4, -1.2]):
            with self.subTest(raw=raw):
                observation = np.asarray(raw, dtype=np.float32)
                np.testing.assert_array_equal(evaluator.predict(observation), runtime.predict(observation))
        np.testing.assert_allclose(evaluator.predict(np.asarray([0., -4.2], dtype=np.float32))[1],
                                   -.7 * np.tanh(3.), rtol=1e-6)


if __name__ == '__main__':
    unittest.main()
