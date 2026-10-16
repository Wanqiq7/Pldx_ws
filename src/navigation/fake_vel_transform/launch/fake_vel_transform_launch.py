from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    fake_vel_transform_node = Node(
        package="fake_vel_transform",
        executable="fake_vel_transform_node",
        name="fake_vel_transform",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odom_frame": "camera_init",
                "body_frame": "body",
                "robot_base_frame": "base_link",
                "fake_robot_base_frame": "base_link_fake",
                "joint_yaw_sign": 1.0,
                "body_to_base_x": 0.0,
                "body_to_base_y": 0.20,
                "body_to_base_z": 0.0,
            }
        ],
    )

    return LaunchDescription([fake_vel_transform_node])
