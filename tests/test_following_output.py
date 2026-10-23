import unittest

from apptirover.drive import FollowingOutput


class FollowingOutputTests(unittest.TestCase):
    def test_small_demand_changes_off_time_without_losing_breakaway_power(self):
        for demand in (.03, .1, .25, .35):
            with self.subTest(demand=demand):
                output = FollowingOutput(.06)
                powers, bursts, length = [], [], 0
                for index in range(1000):
                    left, right = output.update(0, demand, 1., .5, 10 + index * .02, .02)
                    powers.append(left)
                    self.assertEqual(left, -right)
                    if left:
                        self.assertAlmostEqual(left, .5)
                        length += 1
                    elif length:
                        bursts.append(length)
                        length = 0
                self.assertTrue(bursts)
                self.assertEqual(set(bursts), {3})  # 3 control ticks = 60 ms.
                self.assertAlmostEqual(sum(powers) / len(powers), demand, delta=.004)

    def test_caps_ratios_zero_minimum_and_large_continuous_demand(self):
        for maximum, minimum in ((1., .5), (.3, .5), (.8, 0.)):
            output = FollowingOutput(.06)
            samples = [output.update(.2, .1, maximum, minimum, 10 + i * .02, .02) for i in range(100)]
            self.assertTrue(any(a for a, _ in samples))
            for left, right in samples:
                self.assertLessEqual(max(abs(left), abs(right)), maximum + 1e-9)
                if left:
                    self.assertAlmostEqual(left / right, 3.)
            if not minimum:
                self.assertFalse(output.pulsed)
                self.assertAlmostEqual(samples[-1][0], .24)
        output = FollowingOutput(.06)
        samples = [output.update(1., 0, 1., .5, 10 + i * .02, .02) for i in range(100)]
        self.assertEqual(samples[-1], (1., 1.))
        self.assertFalse(output.pulsed)

    def test_zero_and_direction_change_interrupt_even_the_first_pulse(self):
        output = FollowingOutput(.06)
        self.assertEqual(output.update(0, .1, 1., .5, 10, .02), (.5, -.5))
        self.assertEqual(output.update(0, -.1, 1., .5, 10.02, .02), (0., 0.))
        self.assertEqual(output.update(0, -.1, 1., .5, 10.04, .02), (-.5, .5))
        self.assertEqual(output.update(0, 0, 1., .5, 10.06, .02), (0., 0.))
        self.assertIsNone(output.until)
        self.assertEqual(output.credit, 0)
        self.assertEqual(output.update(0, .1, 1., .5, 10.08, .02), (.5, -.5))

    def test_timing_jitter_does_not_extend_pulse_after_its_deadline(self):
        output = FollowingOutput(.06)
        self.assertEqual(output.update(0, .1, 1., .5, 10, .02), (.5, -.5))
        self.assertEqual(output.update(0, .1, 1., .5, 10.025, .025), (.5, -.5))
        self.assertEqual(output.update(0, .1, 1., .5, 10.075, .05), (0., 0.))
        self.assertEqual(output.update(0, .1, 1., .5, 10.1, .025), (0., 0.))


if __name__ == '__main__':
    unittest.main()
