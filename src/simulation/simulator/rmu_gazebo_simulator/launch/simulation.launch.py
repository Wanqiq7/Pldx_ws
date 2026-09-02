# Copyright 2026 navi workspace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import fcntl
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


_SESSION_LOCK_PATH = "/tmp/rmu_gazebo_simulator.lock"
_SESSION_LOCK_HANDLE = None

CONTROLLER_RAW_TOPIC = "/simulation/controller_cmd_vel_global_raw"
CONTROLLER_SMOOTHED_TOPIC = "/simulation/controller_cmd_vel_global_smoothed"
BEHAVIOR_TOPIC = "/simulation/behavior_cmd_vel_body"


def _acquire_session_lock():
    """Prevent two simulation worlds from publishing the same ROS topics."""
    global _SESSION_LOCK_HANDLE

    lock_handle = open(_SESSION_LOCK_PATH, "a+")
    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock_handle.close()
        raise RuntimeError(
            "Another rmu_gazebo_simulator instance is already running; "
            "stop it before starting a second simulation"
        ) from error

    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write(f"pid={os.getpid()}\n")
    lock_handle.flush()
    _SESSION_LOCK_HANDLE = lock_handle


def generate_launch_description():
    _acquire_session_lock()
    simulator = get_package_share_directory("rmu_gazebo_simulator")
    adapter = get_package_share_directory("sim_nav_adapter")
    navi2 = get_package_share_directory("navi2")
    point_lio = get_package_share_directory("point_lio")
    use_rviz = LaunchConfiguration("use_rviz")
    start_paused = LaunchConfiguration("start_paused")
    headless = LaunchConfiguration("headless")
    params_file = LaunchConfiguration("params_file")
    point_lio_params_file = LaunchConfiguration("point_lio_params_file")
    map_file = LaunchConfiguration("map")

    world_config_path = os.path.join(simulator, "config", "gz_world.yaml")
    with open(world_config_path) as file:
        world_config = yaml.safe_load(file)
    selected_world = world_config["world"]
    robots = world_config["robots"][selected_world]
    if len(robots) != 1:
        raise RuntimeError(
            "The navigation simulation requires exactly one robot to own the global TF tree"
        )
    robot = robots[0]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "start_paused",
                default_value="false",
                description="Start Gazebo paused instead of running immediately",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo server-only without starting the GUI",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    simulator, "config", "simulation_nav2_params.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "point_lio_params_file",
                default_value=os.path.join(adapter, "config", "point_lio_sim.yaml"),
            ),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(simulator, "maps", "rmuc_2025.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(simulator, "launch", "bringup_sim.launch.py")
                ),
                launch_arguments={
                    "start_paused": start_paused,
                    "headless": headless,
                }.items(),
            ),
            Node(
                package="ign_sim_pointcloud_tool",
                executable="ign_sim_pointcloud_tool_node",
                name="ign_sim_pointcloud_tool",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "pcd_topic": "/livox/lidar",
                        "n_scan": 32,
                        "horizon_scan": 1875,
                        "ang_bottom": 7.0,
                        "ang_res_y": 1.84375,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(point_lio, "launch", "point_lio.launch.py")
                ),
                launch_arguments={
                    "rviz": "false",
                    "imu_topic": "/livox/imu",
                    "point_lio_cfg_dir": point_lio_params_file,
                }.items(),
            ),
            Node(
                package="sim_nav_adapter",
                executable="navigation_cloud_adapter.py",
                name="navigation_cloud_adapter",
                output="screen",
                parameters=[
                    os.path.join(adapter, "config", "navigation_cloud_adapter_sim.yaml")
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(adapter, "launch", "point_lio_odom.launch.py")
                ),
                launch_arguments={
                    "spawn_x": robot["x_pose"],
                    "spawn_y": robot["y_pose"],
                    "spawn_yaw": robot["yaw"],
                }.items(),
            ),
            Node(
                package="sim_nav_adapter",
                executable="cmd_vel_adapter.py",
                name="sim_cmd_vel_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "controller_raw_topic": CONTROLLER_RAW_TOPIC,
                        "controller_smoothed_topic": CONTROLLER_SMOOTHED_TOPIC,
                        "behavior_topic": BEHAVIOR_TOPIC,
                        "output_topic": "/cmd_vel",
                        "odom_topic": "/odom",
                        "global_frame": "sim_odom",
                        "base_frame": "base_link",
                        "source_timeout_sec": 0.2,
                        "odom_timeout_sec": 0.2,
                        "watchdog_period_sec": 0.02,
                    }
                ],
            ),
            GroupAction(
                scoped=True,
                actions=[
                    # The shared Nav2 launch collapses controller and behavior
                    # output into one cmd_vel topic. These node-specific rules
                    # split the semantic streams before they reach the adapter.
                    SetRemap(src="controller_server:cmd_vel", dst=CONTROLLER_RAW_TOPIC),
                    SetRemap(
                        src="velocity_smoother:cmd_vel",
                        dst=CONTROLLER_RAW_TOPIC,
                    ),
                    SetRemap(
                        src="velocity_smoother:cmd_vel_smoothed",
                        dst=CONTROLLER_SMOOTHED_TOPIC,
                    ),
                    SetRemap(src="behavior_server:cmd_vel", dst=BEHAVIOR_TOPIC),
                    TimerAction(
                        period=10.0,
                        actions=[
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(
                                        navi2, "launch", "navigation2.launch.py"
                                    )
                                ),
                                launch_arguments={
                                    "use_sim_time": "true",
                                    "use_rviz": use_rviz,
                                    "params_file": params_file,
                                    "map": map_file,
                                    "controller_odom_topic": "/odom",
                                }.items(),
                            )
                        ],
                    ),
                ],
            ),
        ]
    )
