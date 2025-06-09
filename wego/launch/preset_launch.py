from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # scan filter
    scan_filter=IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('wego'), 'launch', 'scan_filter_launch.py']))
    
    # joystick
    joystick_control=IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('wego'), 'launch', 'joystick_launch.py'])
        )

    return LaunchDescription([
        scan_filter,
        joystick_control,
    ])