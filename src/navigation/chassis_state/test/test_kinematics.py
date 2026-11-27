import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
from chassis_state.kinematics import Geometry, recover


class KinematicsTest(unittest.TestCase):
    def test_lidar_thirty_degrees_chassis_zero(self):
        angle = np.pi / 6
        rotation = R.from_euler('z', angle)
        result = recover(Geometry(), np.zeros(3), rotation, np.zeros(3),
                         np.array([0., 0., 2.]), angle, 2.)
        np.testing.assert_allclose(result.rotation.as_matrix(), np.eye(3), atol=1e-12)
        np.testing.assert_allclose(result.angular, 0, atol=1e-12)
        np.testing.assert_allclose(result.linear, 0, atol=1e-12)

    def test_offset_tilt_and_simultaneous_motion_against_pose_difference(self):
        # Independent forward pose model; numerical derivative is the velocity oracle.
        geometry = Geometry(pivot=np.array([.2, -.1, .3]),
                            axis_rotation=R.from_euler('xyz', [.1, -.2, .3]),
                            imu_translation=np.array([.12, -.24, .08]),
                            imu_rotation=R.from_euler('xyz', [.4, -.3, .2]),
                            joint_zero=.17, joint_direction=-1.)
        for chassis_rate, joint_rate in [(0., 1.2), (.8, 0.), (.8, 1.2)]:
            def forward(t):
                base = R.from_euler('xyz', [.13, -.11, chassis_rate * t])
                joint = R.from_euler('z', -joint_rate * t + .17)
                mount = base * geometry.axis_rotation * joint
                pos = (np.array([.3 * t, -.1 * t, .2]) + base.apply(geometry.pivot)
                       + mount.apply(geometry.imu_translation))
                return pos, mount * geometry.imu_rotation
            t, h = .6, 1e-5
            p, r = forward(t)
            pm, rm = forward(t - h)
            pp, rp = forward(t + h)
            linear = r.inv().apply((pp - pm) / (2 * h))
            angular = r.inv().apply((rp * rm.inv()).as_rotvec() / (2 * h))
            result = recover(geometry, p, r, linear, angular, joint_rate * t, joint_rate)
            base = R.from_euler('xyz', [.13, -.11, chassis_rate * t])
            np.testing.assert_allclose(result.position, [.3 * t, -.1 * t, .2], atol=1e-9)
            np.testing.assert_allclose(result.rotation.as_matrix(), base.as_matrix(), atol=1e-9)
            np.testing.assert_allclose(base.apply(result.linear), [.3, -.1, 0.], atol=1e-8)
            np.testing.assert_allclose(base.apply(result.angular), [0., 0., chassis_rate], atol=1e-8)
