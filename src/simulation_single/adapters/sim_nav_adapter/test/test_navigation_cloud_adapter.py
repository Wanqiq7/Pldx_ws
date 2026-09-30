import math
from collections import deque
from types import SimpleNamespace
import unittest

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformException

from sim_nav_adapter.navigation_cloud_adapter import NavigationCloudAdapter


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class StaticBuffer:
    def __init__(self, transform=None, error=None, available=True):
        self.transform = transform
        self.error = error
        self.available = available

    def can_transform(self, target, source, stamp):
        return self.available

    def lookup_transform(self, target, source, stamp, timeout):
        if self.error is not None:
            raise self.error
        return self.transform


class NavigationCloudAdapterTest(unittest.TestCase):
    def make_cloud(self, frame="camera_init"):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
            PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
        ]
        header = Header(frame_id=frame)
        header.stamp.sec = 42
        header.stamp.nanosec = 123456789
        cloud = point_cloud2.create_cloud(
            header,
            fields,
            [(1.0, 2.0, 3.0, 7.5, 9)],
            point_step=32,
        )
        cloud.is_dense = True
        return cloud

    def make_transform(self):
        transform = TransformStamped()
        transform.header.frame_id = "sim_odom"
        transform.child_frame_id = "camera_init"
        transform.transform.translation.x = 10.0
        transform.transform.translation.y = 20.0
        transform.transform.translation.z = 30.0
        transform.transform.rotation.x = math.sin(math.pi / 4.0)
        transform.transform.rotation.w = math.cos(math.pi / 4.0)
        return transform

    def make_adapter(self, buffer):
        warnings = []
        adapter = object.__new__(NavigationCloudAdapter)
        adapter.input_frame = "camera_init"
        adapter.output_frame = "sim_odom"
        adapter.readiness_frame = "front_mid360"
        adapter.transform_timeout = object()
        adapter.tf_buffer = buffer
        adapter.pending_clouds = deque()
        adapter.pending_queue_size = 8
        adapter._warn_throttled = lambda key, message: warnings.append((key, message))
        return adapter, warnings

    def test_full_transform_preserves_stamp_and_non_xyz_fields(self):
        adapter, warnings = self.make_adapter(StaticBuffer(self.make_transform()))
        publisher = RecordingPublisher()

        NavigationCloudAdapter._cloud_callback(adapter, self.make_cloud(), publisher)

        self.assertEqual(warnings, [])
        self.assertEqual(len(publisher.messages), 1)
        output = publisher.messages[0]
        self.assertEqual(output.header.frame_id, "sim_odom")
        self.assertEqual(output.header.stamp.sec, 42)
        self.assertEqual(output.header.stamp.nanosec, 123456789)
        self.assertTrue(output.is_dense)
        self.assertEqual(output.point_step, 17)
        self.assertEqual(
            [field.name for field in output.fields],
            ["x", "y", "z", "intensity", "tag"],
        )
        point = tuple(point_cloud2.read_points(output))[0]
        self.assertAlmostEqual(float(point["x"]), 11.0, places=6)
        self.assertAlmostEqual(float(point["y"]), 17.0, places=6)
        self.assertAlmostEqual(float(point["z"]), 32.0, places=6)
        self.assertAlmostEqual(float(point["intensity"]), 7.5, places=6)
        self.assertEqual(int(point["tag"]), 9)

    def test_wrong_input_frame_is_dropped_with_warning(self):
        adapter, warnings = self.make_adapter(StaticBuffer(self.make_transform()))
        publisher = RecordingPublisher()

        NavigationCloudAdapter._cloud_callback(
            adapter, self.make_cloud(frame="body"), publisher
        )

        self.assertEqual(publisher.messages, [])
        self.assertEqual(warnings[0][0], "input_frame")

    def test_missing_transform_is_dropped_with_warning(self):
        adapter, warnings = self.make_adapter(
            StaticBuffer(error=TransformException("not available"))
        )
        publisher = RecordingPublisher()

        NavigationCloudAdapter._cloud_callback(adapter, self.make_cloud(), publisher)

        self.assertEqual(publisher.messages, [])
        self.assertEqual(warnings[0][0], "transform")

    def test_cloud_waits_for_sensor_origin_transform_before_publication(self):
        buffer = StaticBuffer(self.make_transform(), available=False)
        adapter, warnings = self.make_adapter(buffer)
        publisher = RecordingPublisher()

        NavigationCloudAdapter._cloud_callback(adapter, self.make_cloud(), publisher)

        self.assertEqual(publisher.messages, [])
        self.assertEqual(len(adapter.pending_clouds), 1)

        buffer.available = True
        NavigationCloudAdapter._publish_ready_clouds(adapter)

        self.assertEqual(warnings, [])
        self.assertEqual(len(adapter.pending_clouds), 0)
        self.assertEqual(len(publisher.messages), 1)

    def test_warnings_are_rate_limited_per_error_key(self):
        warnings = []
        clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0))
        logger = SimpleNamespace(warning=warnings.append)
        adapter = SimpleNamespace(
            warning_period_ns=2_000_000_000,
            last_warning_ns={},
            get_clock=lambda: clock,
            get_logger=lambda: logger,
        )

        NavigationCloudAdapter._warn_throttled(adapter, "frame", "first")
        NavigationCloudAdapter._warn_throttled(adapter, "frame", "suppressed")
        clock.now = lambda: SimpleNamespace(nanoseconds=2_000_000_000)
        NavigationCloudAdapter._warn_throttled(adapter, "frame", "second")

        self.assertEqual(warnings, ["first", "second"])


if __name__ == "__main__":
    unittest.main()
