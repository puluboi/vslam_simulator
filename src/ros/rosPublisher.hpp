#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include "raylib.h"

class RosPublisher {
public:
    RosPublisher();
    ~RosPublisher();
    
    void publishImage(const Image& frame);
    void spin();
    
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    int sequence_number_;
    
    sensor_msgs::msg::Image imageToRosMsg(const Image& frame);
    sensor_msgs::msg::CameraInfo createCameraInfo();
};
