#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/core/types.hpp>
#include <vector>

class rosConsumer : public rclcpp::Node
{
public:
    rosConsumer();
    // Get intrinsic parameters in matrix form
    struct IntrinsicParams
    {
        double fx, fy, cx, cy;
        double k1, k2, p1, p2, k3; // Distortion coefficients
    };
    IntrinsicParams getIntrinsicParams() const;
    // Getters for latest data
    sensor_msgs::msg::Image::SharedPtr getLatestImage() const;
    geometry_msgs::msg::PoseStamped::SharedPtr getLatestPose() const;
    sensor_msgs::msg::CameraInfo::SharedPtr getLatestCameraInfo() const;

        // Publish 3D points as PointCloud2
    void publishPoints3D(const std::vector<cv::Point3f>& points_3d, const std::string& frame_id = "map");


private:
    // Callback functions
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;

    // Latest data storage
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
};
