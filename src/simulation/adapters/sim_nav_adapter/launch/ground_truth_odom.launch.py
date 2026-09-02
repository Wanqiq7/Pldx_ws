from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="red_standard_robot1"),
        Node(
            package="sim_nav_adapter",
            executable="ground_truth_odom.py",
            namespace=namespace,
            output="screen",
            parameters=[{
                "input_topic": "ground_truth/odom",
                "output_topic": "odom",
                "odom_frame": "odom",
                "base_frame": "base_link",
            }],
        ),
    ])
