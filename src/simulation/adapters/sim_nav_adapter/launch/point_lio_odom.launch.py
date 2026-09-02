from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            Node(
                package="sim_nav_adapter",
                executable="point_lio_odom.py",
                name="point_lio_odom_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/aft_mapped_to_init",
                        "output_topic": "/odom",
                        "input_parent_frame": "camera_init",
                        "input_child_frame": "body",
                        "map_frame": "map",
                        "odom_frame": "sim_odom",
                        "base_frame": "base_link",
                        "tracking_frame": "front_mid360",
                        "spawn_x": LaunchConfiguration("spawn_x"),
                        "spawn_y": LaunchConfiguration("spawn_y"),
                        "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                    }
                ],
            ),
        ]
    )
