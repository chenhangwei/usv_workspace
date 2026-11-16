import math
import unittest

from usv_rl.route_frame import canonical_route_fields


def world_case(angle, origin, along, left, heading_offset, length):
    """Hull at (along, left) in a route frame rotated by angle and shifted to origin."""
    unit = (math.cos(angle), math.sin(angle))
    normal = (-unit[1], unit[0])
    start = origin
    goal = (origin[0] + unit[0] * length, origin[1] + unit[1] * length)
    pose = (origin[0] + unit[0] * along + normal[0] * left, origin[1] + unit[1] * along + normal[1] * left)
    yaw = angle + heading_offset
    feedback_distance = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
    feedback_heading = math.atan2(goal[1] - pose[1], goal[0] - pose[0]) - yaw
    return dict(pose_x=pose[0], pose_y=pose[1], yaw=yaw, feedback_distance=feedback_distance,
                feedback_heading_error=math.atan2(math.sin(feedback_heading), math.cos(feedback_heading)),
                route_start=start, route_goal=goal)


class RouteFrameTests(unittest.TestCase):
    def assertFieldsEqual(self, first, second, places=9):
        for key in first:
            self.assertAlmostEqual(first[key], second[key], places=places, msg=key)

    def test_translation_and_rotation_invariant(self):
        for length in (8.0, 12.0, 28.0):
            for along, left in ((0.0, 0.0), (3.5, -2.4), (length * .7, 2.4), (length - 1.0, .3)):
                reference = canonical_route_fields(**world_case(0.0, (0.0, 0.0), along, left, .2, length))
                for angle, origin in ((1.3, (20.4, -0.01)), (-2.8, (-5.0, 9.0)), (math.pi, (40.0, -30.0))):
                    with self.subTest(length=length, along=along, angle=angle):
                        self.assertFieldsEqual(reference, canonical_route_fields(
                            **world_case(angle, origin, along, left, .2, length)))

    def test_short_leg_uses_feedback_and_true_progress(self):
        case = world_case(.7, (3.0, -4.0), 7.0, -1.1, -.3, 10.0)
        fields = canonical_route_fields(**case)
        self.assertEqual(fields['distance_to_goal'], case['feedback_distance'])
        self.assertEqual(fields['heading_error'], case['feedback_heading_error'])
        self.assertAlmostEqual(fields['route_progress'], .7)
        self.assertAlmostEqual(fields['pose_x'], 7.0)
        self.assertAlmostEqual(fields['pose_y'], -1.1)
        self.assertAlmostEqual(fields['yaw'], -.3)

    def test_long_leg_matches_start_of_a_training_leg(self):
        long_leg = canonical_route_fields(**world_case(-.4, (20.0, 0.0), 9.0, 2.4, .1, 28.0))
        training_leg = canonical_route_fields(**world_case(0.0, (0.0, 0.0), 0.0, 2.4, .1, 12.0))
        self.assertFieldsEqual(long_leg, training_leg)
        self.assertAlmostEqual(long_leg['route_progress'], 0.0)
        self.assertAlmostEqual(long_leg['distance_to_goal'], math.hypot(12.0, 2.4))

    def test_window_reaching_goal_matches_last_training_leg(self):
        for along in (16.0, 22.0, 27.5):
            with self.subTest(along=along):
                long_leg = canonical_route_fields(**world_case(2.0, (-3.0, 5.0), along, -.8, -.25, 28.0))
                training_leg = canonical_route_fields(**world_case(0.0, (0.0, 0.0), along - 16.0, -.8, -.25, 12.0))
                self.assertFieldsEqual(long_leg, training_leg)

    def test_rejects_invalid_inputs(self):
        case = world_case(0.0, (0.0, 0.0), 1.0, 0.0, 0.0, 12.0)
        for override in ({'pose_x': float('nan')}, {'feedback_distance': float('inf')},
                         {'route_goal': (0.0, 0.0)}, {'lookahead': 0.0}):
            with self.subTest(override=override), self.assertRaises(ValueError):
                canonical_route_fields(**{**case, **override})

    def test_turn_governor_matches_original_deployment_formula(self):
        from usv_rl.route_frame import turn_governor_speed_cap
        for degrees in range(-180, 181, 5):
            heading = math.radians(degrees)
            original = max(.34 * (.45 + .55 * max(0.0, math.cos(min(abs(heading), math.pi)))), .14)
            self.assertAlmostEqual(turn_governor_speed_cap(heading), original, places=12)

    def test_teacher_turn_governor_is_opt_in_and_uses_shared_law(self):
        from usv_rl.multi_agent_teacher import MultiAgentColregsTeacher
        from usv_rl.multi_agent_types import AgentLocalObservation
        from usv_rl.route_frame import turn_governor_speed_cap
        for degrees in (0, 30, 70, 120, 170):
            observation = AgentLocalObservation(
                agent_id='usv_01', pose_x=0.0, pose_y=0.0, yaw=0.0, speed=0.2, distance_to_goal=8.0,
                heading_error=math.radians(degrees), raw_linear_x=.34, raw_angular_z=0.0,
                final_linear_x=.2, final_angular_z=0.0)
            plain = MultiAgentColregsTeacher(max_angular_velocity=.7).act(observation)
            governed = MultiAgentColregsTeacher(max_angular_velocity=.7, turn_speed_governor=True).act(observation)
            with self.subTest(degrees=degrees):
                self.assertEqual(plain[0], 1.0)
                self.assertEqual(plain[1], governed[1])
                self.assertAlmostEqual(governed[0], max(.3, turn_governor_speed_cap(math.radians(degrees)) / .34), places=6)


if __name__ == '__main__':
    unittest.main()
