import math
import unittest
from dataclasses import replace

from usv_rl.multi_agent_scenarios import AgentGoalConfig, AgentSpawnConfig, FleetScenario
from usv_rl.solo_recovery import recovery_reset_scenario


class SoloRecoveryTests(unittest.TestCase):
    def scenario(self, angle):
        return FleetScenario(
            name='solo_navigation', duration=40.0,
            agent_spawns={'usv_01': AgentSpawnConfig(2.0, -3.0, angle)},
            agent_goals={'usv_01': AgentGoalConfig(2.0 + 12 * math.cos(angle), -3.0 + 12 * math.sin(angle))},
        )

    def test_signed_offsets_and_reference_route_preserved(self):
        for angle in (0.0, 1.3, -2.8):
            original = self.scenario(angle)
            start = original.agent_spawns['usv_01']
            goal = original.agent_goals['usv_01']
            positions = []
            for side in (-1, 1):
                with self.subTest(angle=angle, side=side):
                    reset = recovery_reset_scenario(original, side * 1.2, side * math.radians(15))
                    spawn = reset.agent_spawns['usv_01']
                    cte = ((spawn.x - start.x) * (goal.y - start.y)
                           - (spawn.y - start.y) * (goal.x - start.x)) / 12
                    error = math.atan2(goal.y - spawn.y, goal.x - spawn.x) - spawn.yaw
                    error = math.atan2(math.sin(error), math.cos(error))
                    self.assertAlmostEqual(cte, side * 1.2)
                    self.assertAlmostEqual(error, side * math.radians(15))
                    self.assertIsNot(reset.agent_spawns, original.agent_spawns)
                    self.assertEqual(original.agent_spawns['usv_01'], start)
                    positions.append((spawn.x, spawn.y))
            self.assertAlmostEqual(sum(position[0] for position in positions) / 2, start.x)
            self.assertAlmostEqual(sum(position[1] for position in positions) / 2, start.y)

    def test_midroute_geometry_preserves_reference_and_goal(self):
        for angle in (0.0, 1.3, -2.8):
            original = self.scenario(angle)
            start = original.agent_spawns['usv_01']
            goal = original.agent_goals['usv_01']
            delta_x, delta_y = goal.x - start.x, goal.y - start.y
            for progress in (.55, .7, .85):
                for side in (-1, 1):
                    with self.subTest(angle=angle, progress=progress, side=side):
                        reset = recovery_reset_scenario(original, side * 2.4, side * math.radians(15), progress)
                        spawn = reset.agent_spawns['usv_01']
                        along = ((spawn.x - start.x) * delta_x + (spawn.y - start.y) * delta_y) / 144
                        cte = ((spawn.x - start.x) * delta_y - (spawn.y - start.y) * delta_x) / 12
                        error = math.atan2(goal.y - spawn.y, goal.x - spawn.x) - spawn.yaw
                        self.assertAlmostEqual(along, progress)
                        self.assertAlmostEqual(cte, side * 2.4)
                        self.assertAlmostEqual(math.atan2(math.sin(error), math.cos(error)), side * math.radians(15))
                        self.assertAlmostEqual(math.hypot(goal.x - spawn.x, goal.y - spawn.y),
                                               math.hypot(12 * (1 - progress), 2.4))
                        self.assertIs(reset.agent_goals, original.agent_goals)
                        self.assertEqual(original.agent_spawns['usv_01'], start)

    def test_rejects_invalid_progress(self):
        for progress in (-.1, 1, 1.1, float('nan'), float('inf')):
            with self.subTest(progress=progress), self.assertRaises(ValueError):
                recovery_reset_scenario(self.scenario(0), 2.4, .1, progress)

    def test_waypoint_turn_range_default_and_extended(self):
        import numpy as np
        from usv_rl.multi_agent_scenarios import MultiAgentScenarioFactory

        def turns(**kwargs):
            values = []
            for seed in range(400):
                scenario = MultiAgentScenarioFactory.create(
                    'waypoint_turn', ('usv_01',), goal_distance=12.0, neighbor_speed=.34,
                    rng=np.random.default_rng(seed), **kwargs)
                goal = scenario.agent_goals['usv_01']
                values.append(abs(math.degrees(math.atan2(goal.y, goal.x))))
            return np.asarray(values)
        default, extended = turns(), turns(waypoint_turn_max_angle=math.pi)
        self.assertLessEqual(default.max(), math.degrees(2.9) + 1e-9)
        self.assertGreater(extended.max(), 175.0)
        self.assertGreater(np.mean(extended > 166.2), .05)

    def test_rejects_nonfinite_and_non_solo_scenarios(self):
        scenario = self.scenario(0)
        for cte, heading in ((float('nan'), 0), (0, float('inf'))):
            with self.assertRaises(ValueError):
                recovery_reset_scenario(scenario, cte, heading)
        multiple = replace(scenario, agent_spawns={**scenario.agent_spawns, 'usv_02': AgentSpawnConfig(0, 0, 0)})
        with self.assertRaises(ValueError):
            recovery_reset_scenario(multiple, 1, .1)
        zero_route = replace(scenario, agent_goals={'usv_01': AgentGoalConfig(2, -3)})
        with self.assertRaises(ValueError):
            recovery_reset_scenario(zero_route, 1, .1)


if __name__ == '__main__':
    unittest.main()