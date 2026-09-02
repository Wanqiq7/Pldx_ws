#!/usr/bin/env python3
"""Check the simulation Global Costmap for a false wall in front of the robot."""

from collections import deque
import math
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


def _yaw(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def transform_xy(transform, point):
    yaw = _yaw(transform.rotation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        transform.translation.x + cos_yaw * point[0] - sin_yaw * point[1],
        transform.translation.y + sin_yaw * point[0] + cos_yaw * point[1],
    )


def grid_cell(grid, point):
    origin = grid.info.origin
    yaw = _yaw(origin.orientation)
    dx = point[0] - origin.position.x
    dy = point[1] - origin.position.y
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    column = math.floor(local_x / grid.info.resolution)
    row = math.floor(local_y / grid.info.resolution)
    if not (0 <= column < grid.info.width and 0 <= row < grid.info.height):
        return None
    return column, row


def spanning_false_wall(false_cells, local_y_bounds, y_min, y_max, resolution):
    remaining = set(false_cells)
    while remaining:
        start = remaining.pop()
        component = {start}
        queue = deque([start])
        while queue:
            column, row = queue.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    neighbor = (column + dx, row + dy)
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        component.add(neighbor)
                        queue.append(neighbor)
        component_y_min = min(local_y_bounds[cell][0] for cell in component)
        component_y_max = max(local_y_bounds[cell][1] for cell in component)
        if (
            component_y_min <= y_min + resolution
            and component_y_max >= y_max - resolution
        ):
            return True
    return False


class SimulationCostmapCheck(Node):
    def __init__(self):
        super().__init__("check_sim_costmap")
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("monitor_duration_sec", 0.0)
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("x_min", 0.3)
        self.declare_parameter("x_max", 1.5)
        self.declare_parameter("y_min", -0.5)
        self.declare_parameter("y_max", 0.5)
        self.declare_parameter("max_false_lethal", 64)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.static_map = None
        self.global_costmap = None
        self.costmap_generation = 0
        self.registered_cloud_received = False
        self.registered_full_cloud_received = False
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, latched_qos
        )
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._costmap_callback,
            latched_qos,
        )
        self.registered_cloud_subscription = self.create_subscription(
            PointCloud2,
            "/simulation/cloud_registered_nav",
            self._registered_cloud_callback,
            qos_profile_sensor_data,
        )
        self.registered_full_cloud_subscription = self.create_subscription(
            PointCloud2,
            "/simulation/cloud_registered_full_nav",
            self._registered_full_cloud_callback,
            qos_profile_sensor_data,
        )
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def _map_callback(self, message):
        self.static_map = message

    def _costmap_callback(self, message):
        self.global_costmap = message
        self.costmap_generation += 1

    def _registered_cloud_callback(self, message):
        self.registered_cloud_received = True

    def _registered_full_cloud_callback(self, message):
        self.registered_full_cloud_received = True

    def run(self):
        deadline = time.monotonic() + float(self.get_parameter("timeout_sec").value)
        monitor_duration = float(self.get_parameter("monitor_duration_sec").value)
        reference_frame = str(self.get_parameter("reference_frame").value)
        monitor_started = None
        last_generation = -1
        checked_updates = 0
        maximum_false_cells = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.static_map is None
                or self.global_costmap is None
                or not self.registered_cloud_received
                or not self.registered_full_cloud_received
            ):
                continue
            if self.costmap_generation == last_generation:
                continue
            try:
                map_to_reference = self.buffer.lookup_transform(
                    self.global_costmap.header.frame_id,
                    reference_frame,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
            except Exception:
                continue
            last_generation = self.costmap_generation
            checked_updates += 1
            false_count, has_wall = self._measure(map_to_reference.transform)
            maximum_false_cells = max(maximum_false_cells, false_count)
            if not self._report_result(false_count, has_wall, log_success=False):
                return False

            if monitor_started is None:
                monitor_started = time.monotonic()
            if monitor_duration <= 0.0:
                self._report_result(false_count, has_wall, log_success=True)
                return True
            if time.monotonic() - monitor_started >= monitor_duration:
                self.get_logger().info(
                    f"PASS: monitored {checked_updates} Costmap updates for "
                    f"{monitor_duration:.1f} s in {reference_frame}; maximum false "
                    f"lethal cells={maximum_false_cells}"
                )
                return True

        self.get_logger().error(
            "Timed out waiting for both adapted clouds, /map, "
            f"/global_costmap/costmap, and Costmap -> {reference_frame}"
        )
        return False

    def _measure(self, map_to_reference):
        x_min = float(self.get_parameter("x_min").value)
        x_max = float(self.get_parameter("x_max").value)
        y_min = float(self.get_parameter("y_min").value)
        y_max = float(self.get_parameter("y_max").value)
        resolution = self.global_costmap.info.resolution
        false_cells = set()
        local_y_bounds = {}

        x = x_min + resolution / 2.0
        while x < x_max:
            y = y_min + resolution / 2.0
            while y < y_max:
                map_point = transform_xy(map_to_reference, (x, y))
                cost_cell = grid_cell(self.global_costmap, map_point)
                static_cell = grid_cell(self.static_map, map_point)
                if cost_cell is not None and static_cell is not None:
                    cost_index = (
                        cost_cell[1] * self.global_costmap.info.width + cost_cell[0]
                    )
                    static_index = (
                        static_cell[1] * self.static_map.info.width + static_cell[0]
                    )
                    if (
                        self.static_map.data[static_index] == 0
                        and self.global_costmap.data[cost_index] >= 100
                    ):
                        false_cells.add(cost_cell)
                        previous_bounds = local_y_bounds.get(cost_cell, (y, y))
                        local_y_bounds[cost_cell] = (
                            min(previous_bounds[0], y),
                            max(previous_bounds[1], y),
                        )
                y += resolution
            x += resolution

        has_wall = spanning_false_wall(
            false_cells,
            local_y_bounds,
            y_min,
            y_max,
            resolution,
        )
        return len(false_cells), has_wall

    def _report_result(self, false_count, has_wall, log_success):
        maximum = int(self.get_parameter("max_false_lethal").value)
        if false_count > maximum or has_wall:
            self.get_logger().error(
                f"False obstacle check failed: cells={false_count} "
                f"(limit {maximum}), spanning_wall={has_wall}"
            )
            return False

        if log_success:
            self.get_logger().info(
                f"PASS: false lethal cells={false_count} <= {maximum}; "
                "no 8-connected lateral wall"
            )
        return True


def main():
    rclpy.init()
    node = SimulationCostmapCheck()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
