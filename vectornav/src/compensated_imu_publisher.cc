#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "rclcpp/rclcpp.hpp"
#include "vectornav_msgs/msg/compensated_imu.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace vectornav {

class CompensatedImuPublisher : public rclcpp::Node {
public:
    CompensatedImuPublisher() : Node("compensated_imu_publisher") {
        // Parameters
        publish_orientation_ = declare_parameter<bool>("publish_orientation", true);
        publish_angular_velocity_ = declare_parameter<bool>("publish_angular_velocity", true);
        publish_linear_acceleration_ = declare_parameter<bool>("publish_linear_acceleration", true);
        frame_id_ = declare_parameter<std::string>("frame_id", "vectornav");
        
        // Publishers
        compensated_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "vectornav/compensated_imu", 10);
        
        // Subscribers
        compensated_imu_sub_ = create_subscription<vectornav_msgs::msg::CompensatedImu>(
            "vectornav/compensated_imu_raw", 10,
            std::bind(&CompensatedImuPublisher::compensatedImuCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Compensated IMU Publisher started");
        RCLCPP_INFO(get_logger(), "Publishing to: /vectornav/compensated_imu");
    }

private:
    void compensatedImuCallback(const vectornav_msgs::msg::CompensatedImu::SharedPtr msg) {
        auto imu_msg = sensor_msgs::msg::Imu();
        
        // Copy header
        imu_msg.header = msg->header;
        imu_msg.header.frame_id = frame_id_;
        
        // Publish orientation if enabled and available
        if (publish_orientation_ && msg->rotation_to_reference.w != 0.0) {
            imu_msg.orientation = msg->rotation_to_reference;
            
            // Set orientation covariance (you may want to adjust these values)
            imu_msg.orientation_covariance[0] = 0.01;  // x
            imu_msg.orientation_covariance[4] = 0.01;  // y
            imu_msg.orientation_covariance[8] = 0.01;  // z
        } else {
            // Set orientation covariance to -1 to indicate no orientation data
            imu_msg.orientation_covariance[0] = -1;
        }
        
        // Publish angular velocity if enabled
        if (publish_angular_velocity_) {
            imu_msg.angular_velocity = msg->angular_velocity;
            
            // Set angular velocity covariance
            imu_msg.angular_velocity_covariance[0] = 0.01;  // x
            imu_msg.angular_velocity_covariance[4] = 0.01;  // y
            imu_msg.angular_velocity_covariance[8] = 0.01;  // z
        } else {
            imu_msg.angular_velocity_covariance[0] = -1;
        }
        
        // Publish linear acceleration if enabled
        if (publish_linear_acceleration_) {
            imu_msg.linear_acceleration = msg->linear_acceleration;
            
            // Set linear acceleration covariance
            imu_msg.linear_acceleration_covariance[0] = 0.01;  // x
            imu_msg.linear_acceleration_covariance[4] = 0.01;  // y
            imu_msg.linear_acceleration_covariance[8] = 0.01;  // z
        } else {
            imu_msg.linear_acceleration_covariance[0] = -1;
        }
        
        // Publish the IMU message
        compensated_imu_pub_->publish(imu_msg);
    }
    
    // Parameters
    bool publish_orientation_;
    bool publish_angular_velocity_;
    bool publish_linear_acceleration_;
    std::string frame_id_;
    
    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr compensated_imu_pub_;
    rclcpp::Subscription<vectornav_msgs::msg::CompensatedImu>::SharedPtr compensated_imu_sub_;
};

} // namespace vectornav

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vectornav::CompensatedImuPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 