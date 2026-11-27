import math
import unittest
from chassis_state.alignment import JointBuffer


class AlignmentTest(unittest.TestCase):
    def test_brackets_lio_time_and_unwraps_pi(self):
        samples = JointBuffer()
        samples.add(10., math.pi - .1, 2.)
        self.assertIsNone(samples.at(10.05))  # Wait for measurement, never stamp old data as new.
        samples.add(10.1, -math.pi + .1, 2.)
        angle, rate = samples.at(10.05)
        self.assertAlmostEqual(angle, math.pi)
        self.assertAlmostEqual(rate, 2.)
        self.assertIsNone(samples.at(9.))

    def test_reordered_feedback_and_clock_reset(self):
        samples = JointBuffer()
        samples.add(10., .1, 1.)
        samples.add(10.2, .3, 1.)
        samples.add(10.1, .2, 1.)
        self.assertAlmostEqual(samples.at(10.15)[0], .25)
        samples.add(1., 0., 0.)  # ROS time reset clears incompatible history.
        self.assertIsNone(samples.at(10.15))
