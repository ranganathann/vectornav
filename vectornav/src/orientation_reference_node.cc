#include <memory>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/compensated_imu.hpp"
#include "vectornav_msgs/srv/orientation_reference.hpp"
#include "vectornav_msgs/srv/tare.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace vectornav {

class OrientationReferenceNode : public rclcpp::Node {
public:
    OrientationReferenceNode() : Node("orientation_reference") {
        // Parameters
        calibration_duration_ = declare_parameter<double>("calibration_duration", 3.0);
        calibration_samples_ = declare_parameter<int>("calibration_samples", 60);
        
        // Publishers
        compensated_imu_pub_ = create_publisher<vectornav_msgs::msg::CompensatedImu>(
            "vectornav/compensated_imu_raw", 10);
        

        
        // Subscribers
        imu_sub_ = create_subscription<vectornav_msgs::msg::ImuGroup>(
            "vectornav/raw/imu", 10,
            std::bind(&OrientationReferenceNode::imuCallback, this, std::placeholders::_1));
        
        attitude_sub_ = create_subscription<vectornav_msgs::msg::AttitudeGroup>(
            "vectornav/raw/attitude", 10,
            std::bind(&OrientationReferenceNode::attitudeCallback, this, std::placeholders::_1));
        
        // Service
        orientation_service_ = create_service<vectornav_msgs::srv::OrientationReference>(
            "vectornav/orientation_reference",
            std::bind(&OrientationReferenceNode::handleOrientationService, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Tare service client
        tare_client_ = create_client<vectornav_msgs::srv::Tare>("vectornav/tare");
        
        // Initialize state
        reference_orientation_set_ = false;
        calibration_start_time_ = this->now();
        calibration_samples_collected_ = 0;
        attitude_received_ = false;
        
        RCLCPP_INFO(get_logger(), "Orientation reference node started. Calibrating for %.1f seconds...", 
                   calibration_duration_);
    }

private:
    void imuCallback(const vectornav_msgs::msg::ImuGroup::SharedPtr msg) {
        if (!reference_orientation_set_) {
            // Still in calibration phase
            auto current_time = this->now();
            bool duration_expired = (current_time - calibration_start_time_).seconds() >= calibration_duration_;
            bool samples_complete = calibration_samples_collected_ >= calibration_samples_;
            
            if (!duration_expired && !samples_complete) {
                // Collect acceleration samples for gravity vector calculation
                if (msg->group_fields & vectornav_msgs::msg::ImuGroup::IMUGROUP_ACCEL) {
                    gravity_samples_.push_back(msg->accel);
                    
                    if (samples_complete || duration_expired) {
                        calculateReferenceOrientation();
                    }
                }
            } else if (duration_expired || samples_complete) {
                calculateReferenceOrientation();
            }
        } else {
            // Publish compensated IMU data
            publishCompensatedImu(msg);
        }
    }
    
    void attitudeCallback(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg) {
        if (reference_orientation_set_ && 
            (msg->group_fields & vectornav_msgs::msg::AttitudeGroup::ATTITUDEGROUP_QUATERNION)) {
            current_attitude_ = msg->quaternion;
            attitude_received_ = true;
        }
    }
    
    void calculateReferenceOrientation() {
        if (gravity_samples_.empty()) {
            RCLCPP_ERROR(get_logger(), "No gravity samples collected!");
            return;
        }
        
        // Calculate average gravity vector
        geometry_msgs::msg::Vector3 avg_gravity;
        avg_gravity.x = 0.0;
        avg_gravity.y = 0.0;
        avg_gravity.z = 0.0;
        
        for (const auto& sample : gravity_samples_) {
            avg_gravity.x += sample.x;
            avg_gravity.y += sample.y;
            avg_gravity.z += sample.z;
        }
        
        avg_gravity.x /= gravity_samples_.size();
        avg_gravity.y /= gravity_samples_.size();
        avg_gravity.z /= gravity_samples_.size();
        
        // Store reference gravity vector as-is (no normalization)
        // The sensor's measured gravity is the "true gravity" from its perspective
        reference_gravity_vector_ = avg_gravity;
        
        // Calculate reference orientation from gravity vector
        // The gravity vector should point "down" in the world frame
        // We'll create a quaternion that rotates the sensor frame to align with world frame
        tf2::Vector3 gravity_vec(avg_gravity.x, avg_gravity.y, avg_gravity.z);
        gravity_vec.normalize();
        
        // Create a rotation that aligns the negative Z-axis with the gravity vector
        tf2::Vector3 world_down(0, 0, -1);
        tf2::Quaternion rotation = quaternionFromVectors(world_down, gravity_vec);
        
        // Store the INVERSE of the reference quaternion (body -> world)
        // This makes the math consistent in publishCompensatedImu
        tf2::Quaternion inverse_rotation = rotation.inverse();
        
        // Convert to geometry_msgs::Quaternion
        reference_orientation_.x = inverse_rotation.x();
        reference_orientation_.y = inverse_rotation.y();
        reference_orientation_.z = inverse_rotation.z();
        reference_orientation_.w = inverse_rotation.w();
        
        reference_orientation_set_ = true;
        
        // Calculate measured gravity magnitude for logging
        double measured_gravity_magnitude = std::sqrt(
            avg_gravity.x * avg_gravity.x + 
            avg_gravity.y * avg_gravity.y + 
            avg_gravity.z * avg_gravity.z);
        
        RCLCPP_INFO(get_logger(), "Reference orientation calculated:");
        RCLCPP_INFO(get_logger(), "  Gravity vector: [%.3f, %.3f, %.3f] (magnitude: %.3f)", 
                   avg_gravity.x, avg_gravity.y, avg_gravity.z, measured_gravity_magnitude);
        RCLCPP_INFO(get_logger(), "  Reference quaternion: [%.3f, %.3f, %.3f, %.3f]",
                   reference_orientation_.x, reference_orientation_.y, 
                   reference_orientation_.z, reference_orientation_.w);
        
        // Clear samples to free memory
        // gravity_samples_.clear();
    }
    
    void publishCompensatedImu(const vectornav_msgs::msg::ImuGroup::SharedPtr msg) {
        auto compensated_msg = vectornav_msgs::msg::CompensatedImu();
        compensated_msg.header = msg->header;
        
        // Store original data
        if (msg->group_fields & vectornav_msgs::msg::ImuGroup::IMUGROUP_ACCEL) {
            compensated_msg.original_acceleration = msg->accel;
        }
        if (msg->group_fields & vectornav_msgs::msg::ImuGroup::IMUGROUP_ANGULARRATE) {
            compensated_msg.original_angular_velocity = msg->angularrate;
        }
        
        // Calculate compensated acceleration (remove gravity)
        if (msg->group_fields & vectornav_msgs::msg::ImuGroup::IMUGROUP_ACCEL) {
            // Dynamically compensate for gravity using current attitude
            if (attitude_received_) {
                // Convert current attitude to tf2 quaternion and normalize
                tf2::Quaternion current_q(current_attitude_.x, current_attitude_.y, 
                                        current_attitude_.z, current_attitude_.w);
                current_q.normalize();
                
                // Calculate measured gravity magnitude from reference vector
                double measured_gravity_magnitude = std::sqrt(
                    reference_gravity_vector_.x * reference_gravity_vector_.x +
                    reference_gravity_vector_.y * reference_gravity_vector_.y +
                    reference_gravity_vector_.z * reference_gravity_vector_.z);
                
                // Direct gravity compensation: rotate world gravity to body frame
                // Use measured gravity magnitude instead of fixed 9.81
                tf2::Vector3 g_world(0, 0, -measured_gravity_magnitude);
                tf2::Vector3 g_body = tf2::quatRotate(current_q.inverse(), g_world);
                
                // Remove the rotated gravity from acceleration
                compensated_msg.linear_acceleration.x = msg->accel.x - g_body.x();
                compensated_msg.linear_acceleration.y = msg->accel.y - g_body.y();
                compensated_msg.linear_acceleration.z = msg->accel.z - g_body.z();
            } else {
                // Fallback to static compensation if no current attitude available
                compensated_msg.linear_acceleration.x = msg->accel.x - reference_gravity_vector_.x;
                compensated_msg.linear_acceleration.y = msg->accel.y - reference_gravity_vector_.y;
                compensated_msg.linear_acceleration.z = msg->accel.z - reference_gravity_vector_.z;
            }
        }
        
        // Calculate compensated angular velocity (zeroed relative to initial orientation)
        if (msg->group_fields & vectornav_msgs::msg::ImuGroup::IMUGROUP_ANGULARRATE) {
            // For now, just use the original angular velocity
            // In a more sophisticated implementation, you might want to account for
            // the rotation from the current attitude to the reference orientation
            compensated_msg.angular_velocity = msg->angularrate;
        }
        
        // Store reference information
        compensated_msg.reference_orientation = reference_orientation_;
        compensated_msg.gravity_vector = reference_gravity_vector_;
        
        // Calculate rotation from current to reference frame
        if (attitude_received_) {
            tf2::Quaternion current_q(current_attitude_.x, current_attitude_.y, 
                                    current_attitude_.z, current_attitude_.w);
            current_q.normalize();
            
            tf2::Quaternion reference_q(reference_orientation_.x, reference_orientation_.y,
                                      reference_orientation_.z, reference_orientation_.w);
            reference_q.normalize();
            
            // Calculate rotation from current to reference frame
            tf2::Quaternion rotation_to_ref = reference_q * current_q.inverse();
            rotation_to_ref.normalize();
            
            compensated_msg.rotation_to_reference.x = rotation_to_ref.x();
            compensated_msg.rotation_to_reference.y = rotation_to_ref.y();
            compensated_msg.rotation_to_reference.z = rotation_to_ref.z();
            compensated_msg.rotation_to_reference.w = rotation_to_ref.w();
        }
        
        compensated_imu_pub_->publish(compensated_msg);
    }
    
    void handleOrientationService(
        const std::shared_ptr<vectornav_msgs::srv::OrientationReference::Request> request,
        std::shared_ptr<vectornav_msgs::srv::OrientationReference::Response> response) {
        
        if (request->reset_reference) {
            // Reset reference orientation
            reference_orientation_set_ = false;
            calibration_samples_collected_ = 0;
            gravity_samples_.clear();
            calibration_start_time_ = this->now();
            attitude_received_ = false;
            
            response->success = true;
            response->message = "Reference orientation reset. Recalibrating...";
            RCLCPP_INFO(get_logger(), "Reference orientation reset by service call");
        }
        
        if (request->tare_gyro) {
            // Send tare command to Vectornav
            response->success = sendTareCommand();
            if (response->success) {
                response->message = "Gyro tare command sent successfully";
                RCLCPP_INFO(get_logger(), "Gyro tare command sent to Vectornav");
            } else {
                response->message = "Failed to send gyro tare command";
                RCLCPP_ERROR(get_logger(), "Failed to send gyro tare command");
            }
        }
        
        // Always return current reference information
        response->reference_orientation = reference_orientation_;
        response->reference_gravity_vector[0] = reference_gravity_vector_.x;
        response->reference_gravity_vector[1] = reference_gravity_vector_.y;
        response->reference_gravity_vector[2] = reference_gravity_vector_.z;
    }
    
    bool sendTareCommand() {
        if (!tare_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "Tare service not available");
            return false;
        }
        
        auto request = std::make_shared<vectornav_msgs::srv::Tare::Request>();
        request->tare_gyro = true;
        request->tare_accel = false;
        request->tare_mag = false;
        
        auto future = tare_client_->async_send_request(request);
        
        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
            RCLCPP_ERROR(get_logger(), "Tare service call timed out");
            return false;
        }
        
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(get_logger(), "Tare command sent successfully: %s", response->message.c_str());
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "Tare command failed: %s", response->message.c_str());
            return false;
        }
    }
    
    tf2::Quaternion quaternionFromVectors(const tf2::Vector3& from, const tf2::Vector3& to) {
        // Robust quaternion calculation from Shoemake 1994
        // Handles parallel and anti-parallel vectors properly
        
        tf2::Vector3 v0 = from.normalized();
        tf2::Vector3 v1 = to.normalized();
        
        float dot = v0.dot(v1);
        
        // Handle parallel vectors
        if (dot > 0.999999f) {
            // Vectors are nearly parallel - return identity quaternion
            return tf2::Quaternion(0, 0, 0, 1);
        }
        
        // Handle anti-parallel vectors
        if (dot < -0.999999f) {
            // Vectors are nearly anti-parallel - find perpendicular axis
            tf2::Vector3 axis;
            if (std::abs(v0.x()) < std::abs(v0.y())) {
                if (std::abs(v0.x()) < std::abs(v0.z())) {
                    axis = tf2::Vector3(1, 0, 0);
                } else {
                    axis = tf2::Vector3(0, 0, 1);
                }
            } else {
                if (std::abs(v0.y()) < std::abs(v0.z())) {
                    axis = tf2::Vector3(0, 1, 0);
                } else {
                    axis = tf2::Vector3(0, 0, 1);
                }
            }
            axis = axis.cross(v0).normalized();
            return tf2::Quaternion(axis.x(), axis.y(), axis.z(), 0);
        }
        
        // General case: calculate rotation quaternion
        tf2::Vector3 cross = v0.cross(v1);
        float s = std::sqrt((1.0f + dot) * 2.0f);
        float invs = 1.0f / s;
        
        return tf2::Quaternion(cross.x() * invs, cross.y() * invs, cross.z() * invs, s * 0.5f);
    }
    
    // Parameters
    double calibration_duration_;
    int calibration_samples_;
    
    // State variables
    bool reference_orientation_set_;
    rclcpp::Time calibration_start_time_;
    int calibration_samples_collected_;
    std::vector<geometry_msgs::msg::Vector3> gravity_samples_;
    
    // Reference data
    geometry_msgs::msg::Quaternion reference_orientation_;
    geometry_msgs::msg::Vector3 reference_gravity_vector_;
    geometry_msgs::msg::Quaternion current_attitude_;
    
    // Publishers and subscribers
    rclcpp::Publisher<vectornav_msgs::msg::CompensatedImu>::SharedPtr compensated_imu_pub_;
    rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr imu_sub_;
    rclcpp::Subscription<vectornav_msgs::msg::AttitudeGroup>::SharedPtr attitude_sub_;
    rclcpp::Service<vectornav_msgs::srv::OrientationReference>::SharedPtr orientation_service_;
    rclcpp::Client<vectornav_msgs::srv::Tare>::SharedPtr tare_client_;
    
    // Additional state variables
    bool attitude_received_;
};

} // namespace vectornav

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vectornav::OrientationReferenceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 