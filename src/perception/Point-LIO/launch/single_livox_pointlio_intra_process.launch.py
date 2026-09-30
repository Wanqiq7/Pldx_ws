from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_body_to_livox_frame",
                arguments=[
                    "0.039512", "0", "0.537619",
                    "1", "0", "0", "0",
                    "body", "livox_frame",
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("point_lio"),
                            "launch",
                            "livox_pointlio_intra_process.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "driver_params_file": PathJoinSubstitution(
                        [
                            FindPackageShare("livox_ros_driver2"),
                            "config",
                            "single_MID360_component.yaml",
                        ]
                    ),
                    "pointlio_params_file": PathJoinSubstitution(
                        [FindPackageShare("point_lio"), "config", "mid360.yaml"]
                    ),
                    "pointlio_imu_topic": "livox/imu",
                    "use_intra_process": "true",
                    "container_name": "livox_pointlio_container",
                    "log_level": "info",
                }.items(),
            )
        ]
    )
