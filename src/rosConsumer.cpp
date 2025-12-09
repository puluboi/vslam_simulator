#include "rosConsumer.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

rosConsumer::rosConsumer() : Node("ros_consumer_node") {
    // Subscribe to image topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&rosConsumer::imageCallback, this, std::placeholders::_1));
    
    // Subscribe to pose topic
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ground_truth/pose", 10,
        std::bind(&rosConsumer::poseCallback, this, std::placeholders::_1));
    
    // Subscribe to camera info topic
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", 10,
        std::bind(&rosConsumer::cameraInfoCallback, this, std::placeholders::_1));
    
    // Create publisher for 3D points
    points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/orb_points_3d", 10);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/tracking_viz", 10);
    
    
    RCLCPP_INFO(this->get_logger(), "ROS Consumer node initialized");
}

void rosConsumer::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    latest_image_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "Received image: %dx%d", msg->width, msg->height);
}

void rosConsumer::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    latest_pose_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "Received pose at [%.2f, %.2f, %.2f]", 
                 msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void rosConsumer::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    latest_camera_info_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "Received camera info: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                 msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
}

void rosConsumer::publishTrackingViz(const cv::Mat frame)
{
    auto cv_image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame);
    sensor_msgs::msg::Image image_msg = *(cv_image.toImageMsg());

    image_pub_->publish(image_msg);


}

sensor_msgs::msg::Image::SharedPtr rosConsumer::getLatestImage() const {
    return latest_image_;
}

geometry_msgs::msg::PoseStamped::SharedPtr rosConsumer::getLatestPose() const {
    return latest_pose_;
}

sensor_msgs::msg::CameraInfo::SharedPtr rosConsumer::getLatestCameraInfo() const {
    return latest_camera_info_;
}

rosConsumer::IntrinsicParams rosConsumer::getIntrinsicParams() const {
    IntrinsicParams params{};
    
    if (latest_camera_info_) {
        // Camera matrix K = [fx  0 cx]
        //                   [ 0 fy cy]
        //                   [ 0  0  1]
        params.fx = latest_camera_info_->k[0]; // K[0,0]
        params.fy = latest_camera_info_->k[4]; // K[1,1]
        params.cx = latest_camera_info_->k[2]; // K[0,2]
        params.cy = latest_camera_info_->k[5]; // K[1,2]
        
        // Distortion coefficients [k1, k2, p1, p2, k3]
        if (latest_camera_info_->d.size() >= 5) {
            params.k1 = latest_camera_info_->d[0];
            params.k2 = latest_camera_info_->d[1];
            params.p1 = latest_camera_info_->d[2];
            params.p2 = latest_camera_info_->d[3];
            params.k3 = latest_camera_info_->d[4];
        }
    }
    
    return params;
}

void rosConsumer::publishPoints3D(const std::vector<cv::Point3f>& points_3d, const std::string& frame_id) {
    if (points_3d.empty()) {
        return;
    }

    // Create PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = frame_id;
    
    // Set up the fields (x, y, z)
    cloud_msg.height = 1;
    cloud_msg.width = points_3d.size();
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;
    
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    
    // Create iterators for x, y, z
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    
    // Fill the point cloud
    for (const auto& pt : points_3d) {
        *iter_x = pt.x;
        *iter_y = pt.y;
        *iter_z = pt.z;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    
    // Publish the message
    points_pub_->publish(cloud_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published %zu 3D points", points_3d.size());
}
