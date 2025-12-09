#include "rosPublisher.hpp"
#include <chrono>
#include <stdio.h>
#include <cmath>

RosPublisher::RosPublisher() : sequence_number_(0) {
    // Initialize ROS 2
    rclcpp::init(0, nullptr);
    
    // Create node
    node_ = rclcpp::Node::make_shared("vslam_simulator");
    
    // Create publishers
    image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10);
    camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", 10);
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/ground_truth/pose", 10);
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ground_truth/twist", 10);
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
        "/imu/data", 10);
    
    // Create TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    
    RCLCPP_INFO(node_->get_logger(), "ROS 2 Image Publisher initialized");
}

RosPublisher::~RosPublisher() {
    rclcpp::shutdown();
}

void RosPublisher::publishImage(const Image& frame, const Camera3D& camera) {
    // Convert raylib Image to ROS Image message
    auto image_msg = imageToRosMsg(frame);
    // Pass camera to calculate intrinsics
    auto camera_info_msg = createCameraInfo(frame, camera);
    
    // Publish both messages
    image_pub_->publish(image_msg);
    camera_info_pub_->publish(camera_info_msg);
    sequence_number_++;
}

sensor_msgs::msg::Image RosPublisher::imageToRosMsg(const Image& frame) {
    sensor_msgs::msg::Image msg;
    
    // Create header with timestamp
    msg.header.stamp = node_->now();
    msg.header.frame_id = "camera_frame";
    
    // Set image dimensions
    msg.height = frame.height;
    msg.width = frame.width;
    
    // Determine encoding based on raylib format
    switch(frame.format) {
        case PIXELFORMAT_UNCOMPRESSED_R8G8B8:
            msg.encoding = "rgb8";
            msg.step = frame.width * 3;
            break;
        case PIXELFORMAT_UNCOMPRESSED_R8G8B8A8:
            msg.encoding = "rgba8";
            msg.step = frame.width * 4;
            break;
        case PIXELFORMAT_UNCOMPRESSED_GRAYSCALE:
            msg.encoding = "mono8";
            msg.step = frame.width;
            break;
        default:
            RCLCPP_WARN(node_->get_logger(), "Unsupported pixel format, defaulting to rgb8");
            msg.encoding = "rgb8";
            msg.step = frame.width * 3;
    }
    
    msg.is_bigendian = false;
    
    // Copy image data
    size_t data_size = msg.step * msg.height;
    msg.data.resize(data_size);
    memcpy(msg.data.data(), frame.data, data_size);
    
    return msg;
}

sensor_msgs::msg::CameraInfo RosPublisher::createCameraInfo(const Image& frame, const Camera3D& camera) {
    sensor_msgs::msg::CameraInfo msg;
    
    msg.header.stamp = node_->now();
    msg.header.frame_id = "camera_frame";
    
    // Set dimensions from the actual frame
    msg.width = frame.width;
    msg.height = frame.height;
    msg.distortion_model = "plumb_bob";
    
    // No distortion in simulated camera
    msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Calculate Focal Length (fx, fy) from FOV
    // Raylib uses vertical FOV in degrees
    double fovy_rad = camera.fovy * (M_PI / 180.0);
    double fy = (msg.height / 2.0) / tan(fovy_rad / 2.0);
    double fx = fy; // Assume square pixels
    
    // Principal point is at the center
    double cx = msg.width / 2.0;
    double cy = msg.height / 2.0;
    
    // Camera matrix K
    msg.k = {
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0
    };
    
    // Rectification matrix R (identity)
    msg.r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    
    // Projection matrix P
    msg.p = {
        fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
    
    return msg;
}

void RosPublisher::publishPose(const geometry_msgs::msg::Pose& pose)
{
    // Publish PoseStamped
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = node_->now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose = pose;
    
    pose_pub_->publish(pose_msg);
    
    // Also broadcast TF transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = pose_msg.header.stamp;
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_link";
    
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    
    transform.transform.rotation = pose.orientation;
    
    tf_broadcaster_->sendTransform(transform);
}

void RosPublisher::publishTwist(const geometry_msgs::msg::Twist& twist) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "base_link";
    msg.twist = twist;
    
    twist_pub_->publish(msg);
}

void RosPublisher::publishIMU(const Vector3& linear_accel, const Vector3& angular_vel) {
    auto imu_msg = sensor_msgs::msg::Imu();
    
    imu_msg.header.stamp = node_->now();
    imu_msg.header.frame_id = "imu_link";
    
    // Convert accelerometer data to ROS coordinates
    imu_msg.linear_acceleration.x = linear_accel.z;   // Forward
    imu_msg.linear_acceleration.y = -linear_accel.x;  // Left
    imu_msg.linear_acceleration.z = linear_accel.y;   // Up
    
    // Convert gyroscope data to ROS coordinates
    imu_msg.angular_velocity.x = angular_vel.z;
    imu_msg.angular_velocity.y = -angular_vel.x;
    imu_msg.angular_velocity.z = angular_vel.y;
    
    // Set covariance (unknown = -1)
    for (int i = 0; i < 9; i++) {
        imu_msg.linear_acceleration_covariance[i] = -1;
        imu_msg.angular_velocity_covariance[i] = -1;
        imu_msg.orientation_covariance[i] = -1;
    }
    
    imu_pub_->publish(imu_msg);
}

void RosPublisher::spin() {
    rclcpp::spin_some(node_);
}
