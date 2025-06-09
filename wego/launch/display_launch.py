import launch
from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    wego_share_dir = get_package_share_directory('wego')
    rviz_config_path = os.path.join(wego_share_dir, 'rviz', 'display.rviz')

    return launch.LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
  ])