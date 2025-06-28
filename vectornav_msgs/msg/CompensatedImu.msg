# Compensated IMU data with gravity removed and gyro zeroed relative to initial orientation
std_msgs/Header header

# Compensated acceleration (gravity removed)
geometry_msgs/Vector3 linear_acceleration

# Compensated angular velocity (zeroed relative to initial orientation)
geometry_msgs/Vector3 angular_velocity

# Original IMU data for reference
geometry_msgs/Vector3 original_acceleration
geometry_msgs/Vector3 original_angular_velocity

# Reference orientation used for compensation
geometry_msgs/Quaternion reference_orientation

# Gravity vector in sensor frame
geometry_msgs/Vector3 gravity_vector

# Rotation from current to reference frame
geometry_msgs/Quaternion rotation_to_reference 