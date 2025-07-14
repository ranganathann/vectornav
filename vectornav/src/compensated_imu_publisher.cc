#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
        imu_compensated_accel_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "vectornav/imu_compensated_accel", 10);
        
        // Subscribers
        compensated_imu_sub_ = create_subscription<vectornav_msgs::msg::CompensatedImu>(
            "vectornav/compensated_imu_raw", 10,
            std::bind(&CompensatedImuPublisher::compensatedImuCallback, this, std::placeholders::_1));
        imu_uncompensated_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "vectornav/imu", 10,
            std::bind(&CompensatedImuPublisher::imuUncompensatedCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Compensated IMU Publisher started");
        RCLCPP_INFO(get_logger(), "Publishing to: /vectornav/compensated_imu and /vectornav/imu_compensated_accel");
    }

private:
    void compensatedImuCallback(const vectornav_msgs::msg::CompensatedImu::SharedPtr msg) {
        auto imu_msg = sensor_msgs::msg::Imu();
        
        // Copy header
        imu_msg.header = msg->header;
        imu_msg.header.frame_id = frame_id_;
        
        // Publish current platform orientation (relative to reference level)
        if (publish_orientation_ && msg->rotation_to_reference.w != 0.0) {
            // Calculate current platform orientation relative to reference (level)
            // This is the inverse of rotation_to_reference to get current tilt
            tf2::Quaternion rotation_to_ref(
                msg->rotation_to_reference.x,
                msg->rotation_to_reference.y,
                msg->rotation_to_reference.z,
                msg->rotation_to_reference.w);
            
            // Invert to get current platform orientation relative to reference
            tf2::Quaternion current_platform_orientation = rotation_to_ref.inverse();
            current_platform_orientation.normalize();
            
            // Convert back to geometry_msgs::Quaternion
            imu_msg.orientation.x = current_platform_orientation.x();
            imu_msg.orientation.y = current_platform_orientation.y();
            imu_msg.orientation.z = current_platform_orientation.z();
            imu_msg.orientation.w = current_platform_orientation.w();
            
            // Set orientation covariance
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

        // --- Publish IMU with uncompensated orientation and compensated acceleration ---
        if (latest_uncompensated_orientation_) {
            sensor_msgs::msg::Imu imu_combined;
            imu_combined.header = msg->header;
            imu_combined.header.frame_id = frame_id_;
            // Use the latest uncompensated orientation
            imu_combined.orientation = latest_uncompensated_orientation_->orientation;
            for (int i = 0; i < 9; ++i) {
                imu_combined.orientation_covariance[i] = latest_uncompensated_orientation_->orientation_covariance[i];
            }
            // Use compensated acceleration
            imu_combined.linear_acceleration = msg->linear_acceleration;
            imu_combined.linear_acceleration_covariance[0] = 0.01;
            imu_combined.linear_acceleration_covariance[4] = 0.01;
            imu_combined.linear_acceleration_covariance[8] = 0.01;
            // Use raw angular velocity from latest uncompensated IMU
            imu_combined.angular_velocity = latest_uncompensated_orientation_->angular_velocity;
            for (int i = 0; i < 9; ++i) {
                imu_combined.angular_velocity_covariance[i] = latest_uncompensated_orientation_->angular_velocity_covariance[i];
            }
            imu_compensated_accel_pub_->publish(imu_combined);
        }
    }

    void imuUncompensatedCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        latest_uncompensated_orientation_ = msg;
    }
    
    // Parameters
    bool publish_orientation_;
    bool publish_angular_velocity_;
    bool publish_linear_acceleration_;
    std::string frame_id_;
    
    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr compensated_imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_compensated_accel_pub_;
    rclcpp::Subscription<vectornav_msgs::msg::CompensatedImu>::SharedPtr compensated_imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_uncompensated_sub_;
    // Store latest uncompensated orientation
    sensor_msgs::msg::Imu::SharedPtr latest_uncompensated_orientation_;
};

} // namespace vectornav

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vectornav::CompensatedImuPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 