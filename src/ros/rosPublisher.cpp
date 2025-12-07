#include "rosPublisher.hpp"
#include <chrono>
#include <stdio.h>

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
    
    RCLCPP_INFO(node_->get_logger(), "ROS 2 Image Publisher initialized");
}

RosPublisher::~RosPublisher() {
    rclcpp::shutdown();
}

void RosPublisher::publishImage(const Image& frame) {
    // Convert raylib Image to ROS Image message
    auto image_msg = imageToRosMsg(frame);
    auto camera_info_msg = createCameraInfo();
    
    // Publish both messages
    image_pub_->publish(image_msg);
    camera_info_pub_->publish(camera_info_msg);
    //td::cout<<"published"<<std::endl;
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

sensor_msgs::msg::CameraInfo RosPublisher::createCameraInfo() {
    sensor_msgs::msg::CameraInfo msg;
    
    msg.header.stamp = node_->now();
    msg.header.frame_id = "camera_frame";
    
    // Camera intrinsics (you should calibrate these for your camera)
    msg.width = 1200;
    msg.height = 800;
    msg.distortion_model = "plumb_bob";
    
    // Placeholder values - replace with actual calibration
    msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Camera matrix K
    msg.k = {
        800.0, 0.0, 600.0,  // fx, 0, cx
        0.0, 800.0, 400.0,  // 0, fy, cy
        0.0, 0.0, 1.0
    };
    
    // Rectification matrix R (identity for monocular)
    msg.r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    
    // Projection matrix P
    msg.p = {
        800.0, 0.0, 600.0, 0.0,
        0.0, 800.0, 400.0, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
    
    return msg;
}

void RosPublisher::spin() {
    rclcpp::spin_some(node_);
}
