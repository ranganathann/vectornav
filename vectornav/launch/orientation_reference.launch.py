import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    this_dir = get_package_share_directory('vectornav')
    
    # Orientation Reference Node
    start_orientation_reference_cmd = Node(
        package='vectornav', 
        executable='orientation_reference_node',
        output='screen',
        parameters=[{
            'calibration_duration': 3.0,
            'calibration_samples': 60,
            'gravity_magnitude': 9.81
        }])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(start_orientation_reference_cmd)

    return ld 