import unittest

from apptirover.following import BoundedPID, FollowConfig


class FollowingPIDTests(unittest.TestCase):
    def test_persistent_error_increases_effort_but_duplicate_frames_do_not_integrate(self):
        pid = BoundedPID(2.5, 1.5, .12)
        first = pid.update(.1, 0, 10, .02)
        for index in range(1, 21):
            later = pid.update(.1, 0, 10 + index * .05, .02)
        self.assertGreater(later, first)
        integral = pid.integral
        for _ in range(100):
            self.assertEqual(pid.update(.1, 0, 11, .02), later)
        self.assertEqual(pid.integral, integral)

    def test_output_saturation_does_not_wind_up_and_zero_or_direction_change_resets(self):
        pid = BoundedPID(2.5, 1.5, .12)
        for index in range(100):
            self.assertEqual(pid.update(1, 0, 10 + index * .1, .02), 1)
        self.assertEqual(pid.integral, 0)
        self.assertLess(pid.update(-.1, 0, 20, .02), 0)
        self.assertEqual(pid.update(0, 0, 20.1, .02), 0)
        self.assertEqual(pid.integral, 0)

    def test_derivative_cannot_cancel_or_reverse_an_outstanding_position_error(self):
        pid = BoundedPID(2.5, 1.5, .12)
        self.assertGreater(pid.update(.1, -100, 10, .02), 0)
        self.assertLess(pid.update(-.1, 100, 10.1, .02), 0)

    def test_gentler_yaw_builds_integral_then_releases_it_as_error_shrinks(self):
        config = FollowConfig()
        pid = BoundedPID(config.yaw_kp, config.yaw_ki, config.yaw_kd, limit=config.turn_limit)
        first = pid.update(.1, 0, 10, .03)
        self.assertLess(first, .1)
        for index in range(1, 41):
            later = pid.update(.1, 0, 10 + index * .05, .03)
        self.assertGreater(pid.integral, config.yaw_kp * .1)
        self.assertLessEqual(later, .25)
        integral = pid.integral
        reduced = pid.update(.04, -.2, 12.05, .03)
        self.assertLess(pid.integral, integral / 2)
        self.assertLess(reduced, later / 2)
        self.assertEqual(pid.update(.02, -.1, 12.1, .03), 0)
        self.assertEqual(pid.integral, 0)

    def test_yaw_limit_is_used_by_anti_windup_and_distance_can_be_integral_led(self):
        config = FollowConfig()
        yaw = BoundedPID(config.yaw_kp, config.yaw_ki, config.yaw_kd, limit=config.turn_limit)
        for index in range(100):
            self.assertEqual(yaw.update(.45, 0, 10 + index * .05, .03), .25)
        self.assertEqual(yaw.integral, 0)
        distance = BoundedPID(config.distance_kp, config.distance_ki, config.distance_kd)
        for index in range(25):
            distance.update(.2, 0, 10 + index * .05, .095)
        self.assertGreater(distance.integral, config.distance_kp * .2)


if __name__ == '__main__':
    unittest.main()
