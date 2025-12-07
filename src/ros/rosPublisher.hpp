#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "raylib.h"

class RosPublisher
{
public:
    RosPublisher();
    ~RosPublisher();

    void publishImage(const Image& frame);
    void publishPose(const geometry_msgs::msg::Pose& pose);
    void publishTwist(const geometry_msgs::msg::Twist& twist);
    void publishIMU(const Vector3& linear_accel, const Vector3& angular_vel);
    void spin();

private:
    sensor_msgs::msg::Image imageToRosMsg(const Image& frame);
    sensor_msgs::msg::CameraInfo createCameraInfo();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    uint32_t sequence_number_;
};
