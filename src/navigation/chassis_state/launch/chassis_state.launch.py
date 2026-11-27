from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('geometry', default_value=os.path.join(
            get_package_share_directory('chassis_state'), 'config', 'cad_initial.yaml')),
        Node(package='chassis_state', executable='chassis_state', output='screen',
             parameters=[LaunchConfiguration('geometry'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
