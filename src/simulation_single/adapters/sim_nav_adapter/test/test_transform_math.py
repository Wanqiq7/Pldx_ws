import math
import unittest

from sim_nav_adapter.transform_math import (
    compute_map_to_base,
    compute_map_to_camera,
    compose_transform,
    inverse_transform,
    planarize_transform,
    project_to_navigation_plane,
    project_twist_covariance_to_navigation_plane,
    quaternion_from_rpy,
    quaternion_from_yaw,
    transform_twist,
    transform_twist_covariance,
)


class TransformMathTest(unittest.TestCase):
    def assert_vector_almost_equal(self, actual, expected):
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=9)

    def test_tilted_sensor_extrinsic_is_projected_to_navigation_plane(self):
        base_to_tracking = (
            (0.16, 0.0, 0.256),
            quaternion_from_rpy(math.pi / 6.0, 0.0, math.pi / 2.0),
        )

        planar = planarize_transform(base_to_tracking)

        self.assert_vector_almost_equal(planar[0], base_to_tracking[0])
        self.assert_vector_almost_equal(planar[1], quaternion_from_yaw(math.pi / 2.0))

    def test_point_lio_tracking_pose_is_shifted_to_base_origin(self):
        base_to_tracking = ((0.2, -0.1, 0.3), quaternion_from_yaw(math.pi / 2.0))
        camera_to_tracking = ((1.0, 2.0, 0.5), quaternion_from_yaw(math.pi / 2.0))

        camera_to_base = compose_transform(
            camera_to_tracking, inverse_transform(base_to_tracking)
        )

        self.assert_vector_almost_equal(camera_to_base[0], (0.8, 2.1, 0.2))
        self.assert_vector_almost_equal(camera_to_base[1], (0.0, 0.0, 0.0, 1.0))

    def test_full_se3_alignment_places_initial_base_at_spawn_pose(self):
        base_to_tracking = (
            (0.3, 0.0, 0.4),
            quaternion_from_rpy(math.pi / 6.0, 0.0, math.pi / 4.0),
        )
        map_to_base_initial = ((3.4, 9.5, 0.0), quaternion_from_yaw(0.3))
        camera_to_tracking_initial = (
            (0.2, -0.4, 0.7),
            quaternion_from_rpy(-0.1, 0.2, -0.3),
        )
        map_to_camera_init = compute_map_to_camera(
            map_to_base_initial,
            base_to_tracking,
            camera_to_tracking_initial,
        )

        actual = compute_map_to_base(
            map_to_camera_init,
            camera_to_tracking_initial,
            base_to_tracking,
        )

        self.assert_vector_almost_equal(actual[0], map_to_base_initial[0])
        self.assert_vector_almost_equal(actual[1], map_to_base_initial[1])

    def test_full_se3_composition_avoids_legacy_meter_scale_xy_error(self):
        base_to_tracking = (
            (0.16, 0.0, 0.256),
            quaternion_from_rpy(math.pi / 6.0, 0.0, math.pi / 2.0),
        )
        map_to_base_initial = (
            (3.4, 9.5, 0.0),
            quaternion_from_yaw(0.0),
        )
        camera_to_tracking_initial = (
            (0.0, 0.0, 0.0),
            quaternion_from_yaw(0.0),
        )
        map_to_camera = compute_map_to_camera(
            map_to_base_initial,
            base_to_tracking,
            camera_to_tracking_initial,
        )
        expected_map_to_base = compose_transform(
            map_to_base_initial,
            ((12.0, 0.0, 0.0), quaternion_from_yaw(0.0)),
        )
        camera_to_tracking = compose_transform(
            compose_transform(inverse_transform(map_to_camera), expected_map_to_base),
            base_to_tracking,
        )

        actual = compute_map_to_base(
            map_to_camera,
            camera_to_tracking,
            base_to_tracking,
        )
        planar_base_to_tracking = planarize_transform(base_to_tracking)
        legacy_map_to_camera = compose_transform(
            map_to_base_initial, planar_base_to_tracking
        )
        legacy_map_to_base = compose_transform(
            legacy_map_to_camera,
            compose_transform(
                planarize_transform(camera_to_tracking),
                inverse_transform(planar_base_to_tracking),
            ),
        )

        full_error = math.hypot(
            actual[0][0] - expected_map_to_base[0][0],
            actual[0][1] - expected_map_to_base[0][1],
        )
        legacy_error = math.hypot(
            legacy_map_to_base[0][0] - expected_map_to_base[0][0],
            legacy_map_to_base[0][1] - expected_map_to_base[0][1],
        )
        self.assertLess(full_error, 1.0e-6)
        self.assertGreater(legacy_error, 1.5)

    def test_navigation_projection_clears_height_roll_and_pitch(self):
        projected = project_to_navigation_plane(
            (
                (1.0, 2.0, 3.0),
                quaternion_from_rpy(0.2, -0.1, 0.4),
            )
        )

        self.assert_vector_almost_equal(projected[0], (1.0, 2.0, 0.0))
        self.assert_vector_almost_equal(projected[1], quaternion_from_yaw(0.4))

    def test_twist_is_rotated_and_shifted_from_tracking_origin(self):
        base_to_tracking = ((1.0, 0.0, 0.0), quaternion_from_yaw(math.pi / 2.0))

        linear, angular = transform_twist(
            (2.0, 0.0, 0.0), (0.0, 0.0, 2.0), base_to_tracking
        )

        self.assert_vector_almost_equal(linear, (0.0, 0.0, 0.0))
        self.assert_vector_almost_equal(angular, (0.0, 0.0, 2.0))

    def test_twist_uses_full_tilted_tracking_extrinsic_before_projection(self):
        base_to_tracking = (
            (0.2, -0.1, 0.3),
            quaternion_from_rpy(math.pi / 6.0, 0.0, math.pi / 2.0),
        )

        linear, angular = transform_twist(
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
            base_to_tracking,
        )

        self.assertAlmostEqual(linear[0], -0.08660254037844384, places=9)
        self.assertAlmostEqual(linear[1], 0.9767949192431122, places=9)
        self.assertAlmostEqual(linear[2], 0.05, places=9)
        self.assertAlmostEqual(angular[0], 0.5, places=9)
        self.assertAlmostEqual(angular[1], 0.0, places=9)
        self.assertAlmostEqual(angular[2], math.sqrt(3.0) / 2.0, places=9)

    def test_navigation_twist_covariance_clears_suppressed_axes(self):
        covariance = [float(index + 1) for index in range(36)]
        transformed = transform_twist_covariance(
            covariance,
            (
                (0.2, -0.1, 0.3),
                quaternion_from_rpy(math.pi / 6.0, 0.0, math.pi / 2.0),
            ),
        )
        projected = project_twist_covariance_to_navigation_plane(transformed)

        for axis in (2, 3, 4):
            self.assertTrue(
                all(projected[axis * 6 + column] == 0.0 for column in range(6))
            )
            self.assertTrue(all(projected[row * 6 + axis] == 0.0 for row in range(6)))
        self.assertNotEqual(projected[0], 0.0)
        self.assertNotEqual(projected[35], 0.0)


if __name__ == "__main__":
    unittest.main()
