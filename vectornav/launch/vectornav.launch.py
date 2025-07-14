import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    this_dir = get_package_share_directory('vectornav')
    
    # Vectornav
    start_vectornav_cmd = Node(
        package='vectornav', 
        executable='vectornav',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'vn_100_400hz.yaml')])
    
    start_vectornav_sensor_msgs_cmd = Node(
        package='vectornav', 
        executable='vn_sensor_msgs',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'vn_100_400hz.yaml')])

    # Orientation Reference Node
    start_orientation_reference_cmd = Node(
        package='vectornav', 
        executable='orientation_reference_node',
        output='screen',
        parameters=[{
            'calibration_duration': 3.0,
            'calibration_samples': 60,
        }])

    # Compensated IMU Publisher Node
    start_compensated_imu_publisher_cmd = Node(
        package='vectornav', 
        executable='compensated_imu_publisher',
        output='screen',
        parameters=[{
            'publish_orientation': True,
            'publish_angular_velocity': True,
            'publish_linear_acceleration': True,
            'frame_id': 'vectornav'
        }])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(start_vectornav_cmd)
    ld.add_action(start_vectornav_sensor_msgs_cmd)
    ld.add_action(start_orientation_reference_cmd)
    ld.add_action(start_compensated_imu_publisher_cmd)

    return ld
