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

from pathlib import Path
import fcntl
import importlib.util
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_nav2_params():
    path = PACKAGE_ROOT / "config" / "simulation_nav2_params.yaml"
    with path.open() as stream:
        return yaml.safe_load(stream)


def read_pgm_size(path):
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P5":
            raise ValueError("expected a raw PGM map")
        line = stream.readline()
        while line.startswith(b"#"):
            line = stream.readline()
        width, height = (int(value) for value in line.split())
        return width, height


class SimulationMapConfigTest(unittest.TestCase):
    def test_simulation_launch_lock_rejects_a_second_instance(self):
        launch_path = PACKAGE_ROOT / "launch" / "simulation.launch.py"
        spec = importlib.util.spec_from_file_location("simulation_launch", launch_path)
        simulation_launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(simulation_launch)

        with tempfile.NamedTemporaryFile() as lock_file:
            simulation_launch._SESSION_LOCK_PATH = lock_file.name
            simulation_launch._acquire_session_lock()

            second_handle = open(lock_file.name, "a+")
            try:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(second_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            finally:
                second_handle.close()
                simulation_launch._SESSION_LOCK_HANDLE.close()

    def test_launch_map_argument_is_not_shadowed_by_empty_node_parameter(self):
        nav2_params = load_nav2_params()

        map_server_params = nav2_params["map_server"]["ros__parameters"]

        self.assertNotIn("yaml_filename", map_server_params)

    def test_costmaps_use_stvl_with_navigation_cloud_and_physical_origin(self):
        nav2_params = load_nav2_params()
        expected_decay = {"local_costmap": 0.6, "global_costmap": 1.5}

        for costmap_name, voxel_decay in expected_decay.items():
            parameters = nav2_params[costmap_name][costmap_name]["ros__parameters"]
            self.assertIn("stvl_layer", parameters["plugins"])
            self.assertNotIn("obstacle_layer", parameters["plugins"])
            layer = parameters["stvl_layer"]
            self.assertEqual(
                layer["plugin"],
                "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer",
            )
            self.assertAlmostEqual(layer["voxel_decay"], voxel_decay)
            self.assertEqual(
                layer["mid360"]["topic"],
                "/simulation/cloud_registered_nav",
            )
            self.assertEqual(layer["mid360"]["sensor_frame"], "front_mid360")

    def test_global_stvl_does_not_replace_static_prior_with_unknown(self):
        parameters = load_nav2_params()["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]

        self.assertTrue(parameters["track_unknown_space"])
        self.assertFalse(parameters["stvl_layer"]["track_unknown_space"])
        self.assertEqual(parameters["stvl_layer"]["combination_method"], 1)

    def test_stvl_rejects_elevated_navigation_floor_returns(self):
        nav2_params = load_nav2_params()

        for costmap_name in ("local_costmap", "global_costmap"):
            source = nav2_params[costmap_name][costmap_name]["ros__parameters"][
                "stvl_layer"
            ]["mid360"]
            self.assertGreaterEqual(source["min_obstacle_height"], 0.3)

    def test_nav2_start_is_delayed_for_simulation_tf_initialization(self):
        launch_text = (PACKAGE_ROOT / "launch" / "simulation.launch.py").read_text()
        self.assertIn("TimerAction", launch_text)
        self.assertIn("period=10.0", launch_text)

    def test_headless_argument_reaches_gazebo(self):
        simulation_launch = (PACKAGE_ROOT / "launch" / "simulation.launch.py").read_text()
        bringup_launch = (PACKAGE_ROOT / "launch" / "bringup_sim.launch.py").read_text()
        gazebo_launch = (PACKAGE_ROOT / "launch" / "gazebo.launch.py").read_text()

        self.assertIn('"headless"', simulation_launch)
        self.assertIn('"headless": headless', simulation_launch)
        self.assertIn('"headless": headless', bringup_launch)
        self.assertIn("\"'-s ' if '\"", gazebo_launch)

    def test_runtime_acceptance_uses_headless_gazebo(self):
        runtime_script = (
            PACKAGE_ROOT / "test" / "check_sim_navigation_runtime.sh"
        ).read_text()
        self.assertIn("use_rviz:=false headless:=true", runtime_script)

    def test_simulation_command_streams_are_isolated_before_chassis_adapter(self):
        launch_text = (PACKAGE_ROOT / "launch" / "simulation.launch.py").read_text()

        for topic in (
            "/simulation/controller_cmd_vel_global_raw",
            "/simulation/controller_cmd_vel_global_smoothed",
            "/simulation/behavior_cmd_vel_body",
        ):
            self.assertIn(topic, launch_text)
        self.assertIn("SetRemap", launch_text)
        self.assertIn("cmd_vel_adapter.py", launch_text)

    def test_simulation_controller_uses_matching_mpc_and_control_periods(self):
        nav2_params = load_nav2_params()
        controller = nav2_params["controller_server"]["ros__parameters"]
        follow_path = controller["FollowPath"]
        self.assertAlmostEqual(controller["controller_frequency"], 20.0)
        self.assertAlmostEqual(follow_path["dt"], 0.05)
        self.assertAlmostEqual(
            1.0 / controller["controller_frequency"], follow_path["dt"]
        )

    def test_simulation_nav_stack_does_not_duplicate_tracking_point_lever_arm(self):
        nav2_params = load_nav2_params()
        self.assertEqual(
            nav2_params["controller_server"]["ros__parameters"]["FollowPath"][
                "lidar_offset_y"
            ],
            0.0,
        )
        self.assertEqual(
            nav2_params["planner_server"]["ros__parameters"]["GridBased"][
                "lidar_offset_y"
            ],
            0.0,
        )

    def test_velocity_smoother_has_an_explicit_fail_closed_timeout(self):
        smoother = load_nav2_params()["velocity_smoother"]["ros__parameters"]
        self.assertAlmostEqual(smoother["velocity_timeout"], 0.2)
        self.assertGreater(smoother["smoothing_frequency"], 0.0)

    def test_mecanum_model_declares_the_gazebo_command_timeout(self):
        model_text = (
            PACKAGE_ROOT.parents[1]
            / "description"
            / "pb2025_robot_description"
            / "resource"
            / "xmacro"
            / "simulation_robot.sdf.xmacro"
        ).read_text()
        self.assertIn("<command_timeout>0.25</command_timeout>", model_text)

    def test_point_lio_adapter_owns_the_sim_odom_navigation_frame(self):
        adapter_launch = (
            PACKAGE_ROOT.parents[1]
            / "adapters"
            / "sim_nav_adapter"
            / "launch"
            / "point_lio_odom.launch.py"
        )
        launch_text = adapter_launch.read_text()
        self.assertIn('"odom_frame": "sim_odom"', launch_text)
        self.assertNotIn("planar_navigation", launch_text)

    def test_navigation_uses_sim_odom_as_local_odom_frame(self):
        nav2_params = load_nav2_params()

        self.assertEqual(
            nav2_params["amcl"]["ros__parameters"]["odom_frame_id"], "sim_odom"
        )
        self.assertEqual(
            nav2_params["controller_server"]["ros__parameters"]["FollowPath"][
                "odom_frame"
            ],
            "sim_odom",
        )
        self.assertEqual(
            nav2_params["local_costmap"]["local_costmap"]["ros__parameters"][
                "global_frame"
            ],
            "sim_odom",
        )

        planner_params = nav2_params["planner_server"]["ros__parameters"]
        self.assertEqual(planner_params["planner_plugins"], ["GridBased"])
        planner = planner_params["GridBased"]
        self.assertEqual(planner["plugin"], "minco_planner/MincoPlanner")
        self.assertEqual(planner["frames"]["rog_frame"], "sim_odom")
        self.assertEqual(planner["rog_map"]["frame_id"], "sim_odom")
        self.assertEqual(
            planner["rog_map"]["ros_callback"]["cloud_topic"],
            "/simulation/cloud_registered_full_nav",
        )
        self.assertEqual(planner["rog_map"]["visualization"]["frame_id"], "sim_odom")

    def test_bt_navigator_uses_jazzy_builtin_plugins(self):
        nav2_params = load_nav2_params()
        bt_params = nav2_params["bt_navigator"]["ros__parameters"]

        self.assertNotIn("plugin_lib_names", bt_params)

    def test_default_bt_recovery_action_servers_are_configured(self):
        nav2_params = load_nav2_params()
        behavior_params = nav2_params["behavior_server"]["ros__parameters"]
        expected_plugins = {
            "spin": "nav2_behaviors::Spin",
            "backup": "nav2_behaviors::BackUp",
            "wait": "nav2_behaviors::Wait",
        }

        self.assertEqual(
            set(behavior_params["behavior_plugins"]), set(expected_plugins)
        )
        for name, plugin_type in expected_plugins.items():
            self.assertEqual(behavior_params[name]["plugin"], plugin_type)
        self.assertEqual(behavior_params["local_frame"], "sim_odom")

    def test_simulation_launch_uses_tf_navigation_cloud_adapter(self):
        launch_text = (PACKAGE_ROOT / "launch" / "simulation.launch.py").read_text()

        self.assertIn("navigation_cloud_adapter.py", launch_text)
        self.assertIn("navigation_cloud_adapter_sim.yaml", launch_text)
        self.assertNotIn("cloud_leveling", launch_text)
        self.assertIn('"global_frame": "sim_odom"', launch_text)

    def test_simulation_packages_declare_cloud_and_stvl_dependencies(self):
        simulation_root = PACKAGE_ROOT.parents[1]
        adapter_package = ET.parse(
            simulation_root / "adapters" / "sim_nav_adapter" / "package.xml"
        ).getroot()
        simulator_package = ET.parse(PACKAGE_ROOT / "package.xml").getroot()

        adapter_dependencies = {
            element.text
            for element in adapter_package
            if element.tag in {"depend", "exec_depend"}
        }
        simulator_dependencies = {
            element.text
            for element in simulator_package
            if element.tag in {"depend", "exec_depend"}
        }
        self.assertIn("tf2_sensor_msgs", adapter_dependencies)
        self.assertIn("spatio_temporal_voxel_layer", simulator_dependencies)

    def test_robot_spawn_is_inside_occupancy_grid(self):
        with (PACKAGE_ROOT / "config" / "gz_world.yaml").open() as stream:
            world_config = yaml.safe_load(stream)
        with (PACKAGE_ROOT / "maps" / "rmuc_2025.yaml").open() as stream:
            map_config = yaml.safe_load(stream)

        world = world_config["world"]
        robot = world_config["robots"][world][0]
        width, height = read_pgm_size(PACKAGE_ROOT / "maps" / map_config["image"])
        resolution = float(map_config["resolution"])
        origin_x, origin_y, _ = map_config["origin"]
        maximum_x = origin_x + width * resolution
        maximum_y = origin_y + height * resolution

        self.assertLessEqual(origin_x, float(robot["x_pose"]))
        self.assertLess(float(robot["x_pose"]), maximum_x)
        self.assertLessEqual(origin_y, float(robot["y_pose"]))
        self.assertLess(float(robot["y_pose"]), maximum_y)

    def test_rog_map_simulation_throughput_profile_is_enabled(self):
        planner = load_nav2_params()["planner_server"]["ros__parameters"]["GridBased"]
        rog_map = planner["rog_map"]

        self.assertGreaterEqual(rog_map["point_filt_num"], 4)
        self.assertTrue(rog_map["raycasting"]["parallel_enable"])
        self.assertEqual(rog_map["raycasting"]["num_threads"], 2)
        self.assertEqual(rog_map["performance"]["raycast_num_threads"], 2)
        self.assertEqual(rog_map["field"]["interpolation"], "bilinear")
        self.assertLessEqual(
            rog_map["raycasting"]["local_update_box"][0], rog_map["map_size"][0]
        )
        self.assertLessEqual(
            rog_map["raycasting"]["local_update_box"][1], rog_map["map_size"][1]
        )
        self.assertLessEqual(rog_map["visualization"]["rate"], 2.0)


if __name__ == "__main__":
    unittest.main()
