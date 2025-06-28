#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vectornav_msgs.srv import OrientationReference
from vectornav_msgs.msg import CompensatedImu
import time

class OrientationReferenceTester(Node):
    def __init__(self):
        super().__init__('orientation_reference_tester')
        
        # Create service client
        self.orientation_client = self.create_client(
            OrientationReference, 'vectornav/orientation_reference')
        
        # Create subscriber for compensated IMU data
        self.imu_subscriber = self.create_subscription(
            CompensatedImu, 'vectornav/compensated_imu', self.imu_callback, 10)
        
        self.get_logger().info("Orientation Reference Tester started")
        
    def imu_callback(self, msg):
        """Callback for compensated IMU data"""
        self.get_logger().info(
            f"Compensated IMU - Linear accel: [{msg.linear_acceleration.x:.3f}, "
            f"{msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f}]")
    
    def call_orientation_service(self, reset_reference=False, tare_gyro=False):
        """Call the orientation reference service"""
        request = OrientationReference.Request()
        request.reset_reference = reset_reference
        request.tare_gyro = tare_gyro
        
        future = self.orientation_client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            response = future.result()
            self.get_logger().info(f"Service response: {response.message}")
            self.get_logger().info(f"Success: {response.success}")
            if response.success:
                self.get_logger().info(
                    f"Reference orientation: [{response.reference_orientation.x:.3f}, "
                    f"{response.reference_orientation.y:.3f}, "
                    f"{response.reference_orientation.z:.3f}, "
                    f"{response.reference_orientation.w:.3f}]")
                self.get_logger().info(
                    f"Reference gravity vector: [{response.reference_gravity_vector[0]:.3f}, "
                    f"{response.reference_gravity_vector[1]:.3f}, "
                    f"{response.reference_gravity_vector[2]:.3f}]")
            return response
        else:
            self.get_logger().error("Service call timed out")
            return None

def main():
    rclpy.init()
    
    tester = OrientationReferenceTester()
    
    # Wait for the service to be available
    while not tester.orientation_client.wait_for_service(timeout_sec=1.0):
        tester.get_logger().info("Waiting for orientation reference service...")
    
    tester.get_logger().info("Service is available!")
    
    try:
        # Wait a bit for initial calibration
        time.sleep(5)
        
        # Get current reference information
        tester.get_logger().info("Getting current reference information...")
        response = tester.call_orientation_service()
        
        if response and response.success:
            # Test tare functionality
            tester.get_logger().info("Testing tare functionality...")
            tare_response = tester.call_orientation_service(tare_gyro=True)
            
            if tare_response and tare_response.success:
                tester.get_logger().info("Tare test successful!")
            else:
                tester.get_logger().warn("Tare test failed")
        
        # Keep the node running to receive IMU data
        tester.get_logger().info("Listening for compensated IMU data...")
        rclpy.spin(tester)
        
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 