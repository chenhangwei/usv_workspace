import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from usv_rl.multi_agent_bridge import MultiAgentTrainingBridge


def feedback(goal_id):
    return SimpleNamespace(goal_id=goal_id)


class BridgeFeedbackGoalFilterTests(unittest.TestCase):
    def setUp(self):
        bridge = object.__new__(MultiAgentTrainingBridge)
        bridge._agent_namespaces = ('usv_01',)
        bridge._goal_id_counter = {'usv_01': 1000}
        bridge._expected_goal_ids = {'usv_01': None}
        bridge._feedback_msgs = {'usv_01': None}
        bridge._raw_cmd_msgs = {'usv_01': None}
        bridge._final_cmd_msgs = {'usv_01': None}
        bridge._goal_publishers = {'usv_01': MagicMock(get_subscription_count=MagicMock(return_value=1))}
        bridge.get_clock = MagicMock()
        self.bridge = bridge
        self.callback = bridge._make_feedback_callback('usv_01')

    def publish_goal(self):
        with patch('usv_rl.multi_agent_bridge.time.sleep'):
            return self.bridge.publish_goals('test', {'usv_01': (1.0, 2.0)})['usv_01']

    def test_accepts_any_feedback_before_first_goal(self):
        message = feedback(7)
        self.callback(message)
        self.assertIs(self.bridge._feedback_msgs['usv_01'], message)

    def test_reset_blocks_old_feedback_until_matching_goal_feedback(self):
        old_goal = self.publish_goal()
        self.bridge.reset_episode_state()
        self.callback(feedback(old_goal))
        self.assertIsNone(self.bridge._feedback_msgs['usv_01'])
        new_goal = self.publish_goal()
        self.callback(feedback(old_goal))
        self.assertIsNone(self.bridge._feedback_msgs['usv_01'])
        current = feedback(new_goal)
        self.callback(current)
        self.assertIs(self.bridge._feedback_msgs['usv_01'], current)

    def test_clear_feedback_waits_for_current_goal(self):
        first_goal = self.publish_goal()
        self.callback(feedback(first_goal))
        second_goal = self.publish_goal()
        self.bridge.clear_navigation_feedback()
        self.assertIsNone(self.bridge._feedback_msgs['usv_01'])
        self.callback(feedback(first_goal))
        self.assertIsNone(self.bridge._feedback_msgs['usv_01'])
        current = feedback(second_goal)
        self.callback(current)
        self.assertIs(self.bridge._feedback_msgs['usv_01'], current)

    def test_midepisode_goal_switch_keeps_cached_feedback_and_rejects_late_old_feedback(self):
        first_goal = self.publish_goal()
        cached = feedback(first_goal)
        self.callback(cached)
        self.publish_goal()
        self.assertIs(self.bridge._feedback_msgs['usv_01'], cached)
        self.callback(feedback(first_goal))
        self.assertIs(self.bridge._feedback_msgs['usv_01'], cached)


if __name__ == '__main__':
    unittest.main()
