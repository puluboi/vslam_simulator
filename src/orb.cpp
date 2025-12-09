#include "orb.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>

Orb_Basic::Orb_Basic(const int feature_count)
{
    orb = cv::ORB::create(
        feature_count,         // nfeatures - more features for robustness
        1.2f,                  // scaleFactor - better scale invariance
        8,                     // nlevels - more pyramid levels
        19,                    // edgeThreshold - balance detection on edges
        0,                     // firstLevel
        2,                     // WTA_K - use 2 for better distinctiveness
        cv::ORB::HARRIS_SCORE, // Use Harris corner score (better than FAST)
        31,                    // patchSize
        10                     // fastThreshold - lower for more features in low-texture
    );

    bf = cv::BFMatcher::create(cv::NORM_HAMMING, false);
    
    consumer_ = std::make_shared<rosConsumer>();

    clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    // Create window for displaying images
    cv::namedWindow("Latest Image", cv::WINDOW_AUTOSIZE);

    // Spin in a loop to process ROS messages and display images
    rclcpp::Rate rate(30); // 30 Hz
    while (rclcpp::ok())
    {
        rclcpp::spin_some(consumer_);
        // Get Camera Intrinsics
        auto intrinsics = consumer_->getIntrinsicParams();
        if (intrinsics.fx > 0)
        { // Check if valid data received
            K = (cv::Mat_<double>(3, 3) << intrinsics.fx, 0, intrinsics.cx,
                 0, intrinsics.fy, intrinsics.cy,
                 0, 0, 1);
        }

        // Get latest image from ROS
        auto ros_image = consumer_->getLatestImage();
        frame_id = ros_image ? ros_image->header.frame_id : "camera_link";

        if (ros_image)
        {
            try
            {
                // Convert ROS image message to OpenCV Mat

                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
                image = cv_ptr->image;
                // Update current position from ROS pose
                auto ros_pose = consumer_->getLatestPose();
                if (ros_pose)
                {
                    curr_position = Eigen::Vector3d(
                        ros_pose->pose.position.x,
                        ros_pose->pose.position.y,
                        ros_pose->pose.position.z);
                    curr_orientation = Eigen::Quaterniond(
                        ros_pose->pose.orientation.w,
                        ros_pose->pose.orientation.x,
                        ros_pose->pose.orientation.y,
                        ros_pose->pose.orientation.z);
                }

                // Initialize prev_ data if empty (First Frame)
                if (prev_descriptors.empty()) {
                    orb->detectAndCompute(image, cv::noArray(), prev_keypoints, prev_descriptors);
                    prev_position = curr_position;
                    prev_orientation = curr_orientation;
                    continue;
                }
                preProcess();
                auto pts = featureExtraction();
                Triangulation(pts.first, pts.second);
                // Publish 3D points to ROS2 in world frame
                if (!points_3d.empty())
                {
                    consumer_->publishPoints3D(points_3d, "world");
                    RCLCPP_INFO(consumer_->get_logger(), "Triangulated %zu 3D points. Example: (%.2f, %.2f, %.2f)",
                                points_3d.size(), points_3d[3].x, points_3d[3].y, points_3d[3].z);
                }
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(consumer_->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }
        // prev_position = curr_position; // Moved to Triangulation (KeyFrame update)
        // prev_orientation = curr_orientation;
        rate.sleep();
    }
}

std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> Orb_Basic::featureExtraction()
{
    // Get and update current pose from ROS before processing
    auto ros_pose = consumer_->getLatestPose();
    if (ros_pose)
    {
        curr_position = Eigen::Vector3d(
            ros_pose->pose.position.x,
            ros_pose->pose.position.y,
            ros_pose->pose.position.z);
        curr_orientation = Eigen::Quaterniond(
            ros_pose->pose.orientation.w,
            ros_pose->pose.orientation.x,
            ros_pose->pose.orientation.y,
            ros_pose->pose.orientation.z);
    }

    orb->detectAndCompute(image, cv::noArray(), curr_keypoints, curr_descriptors);

    cv::Mat output;
    std::vector<cv::Point2f> pts1, pts2;
    // Track keypoints between frames
    if (!prev_descriptors.empty() && !curr_descriptors.empty())
    {
        // Use kNN match for Ratio Test (Lowe's Ratio Test)
        std::vector<std::vector<cv::DMatch>> knn_matches;
        bf->knnMatch(prev_descriptors, curr_descriptors, knn_matches, 2);

        std::vector<cv::DMatch> good_matches;
        const float ratio_thresh = 0.45f;
        
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i].size() >= 2)
            {
                if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                {
                    good_matches.push_back(knn_matches[i][0]);
                }
            }
        }

        // Draw tracking trails on current frame
        const cv::Mat tracking_viz = image.clone();

        pts1.reserve(good_matches.size());
        pts2.reserve(good_matches.size());

        // If tracking is lost (too few matches), reset KeyFrame to current to recover
        if (good_matches.size() < 20)
        {
             prev_keypoints = curr_keypoints;
             prev_descriptors = curr_descriptors.clone();
             prev_position = curr_position;
             prev_orientation = curr_orientation;
             return std::make_pair(std::vector<cv::Point2f>(), std::vector<cv::Point2f>());
        }

        for (const auto &match : good_matches)
        {
            cv::Point2f prev_pt = prev_keypoints[match.queryIdx].pt;
            cv::Point2f curr_pt = curr_keypoints[match.trainIdx].pt;

            pts1.push_back(prev_pt);
            pts2.push_back(curr_pt);

            // Draw arrow showing motion
            cv::arrowedLine(tracking_viz, prev_pt, curr_pt,
                            cv::Scalar(0, 255, 255), 2, 8, 0, 0.3);
            cv::circle(tracking_viz, curr_pt, 3, cv::Scalar(0, 0, 255), -1);
        }

        cv::imshow("Feature Tracking", tracking_viz);

        consumer_->publishTrackingViz(tracking_viz);
    }

    // prev_keypoints = curr_keypoints; // Moved to Triangulation
    // prev_descriptors = curr_descriptors.clone();
    cv::waitKey(1);
    return std::pair(pts1, pts2);
}

void Orb_Basic::preProcess()
{
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    clahe->apply(image, image);

}


void Orb_Basic::Triangulation(const std::vector<cv::Point2f> &pts1,
                              const std::vector<cv::Point2f> &pts2)
{
    //points_3d.clear();
    
    // Check if camera intrinsics are available
    if (K.empty())
    {
        RCLCPP_WARN(consumer_->get_logger(), "Camera intrinsics not available for triangulation");
        return;
    }

    if (pts1.empty() || pts2.empty())
        return;

    // Check baseline distance - need sufficient camera motion for good triangulation
    double baseline = (curr_position - prev_position).norm();
    if (baseline < 0.5)  // Minimum 20cm baseline for stable triangulation
    {
        RCLCPP_DEBUG(consumer_->get_logger(), "Baseline too small: %.3f m. Skipping triangulation.", baseline);
        return;
    }
    
    RCLCPP_INFO(consumer_->get_logger(), "Baseline: %.3f m - triangulating", baseline);

    // Pose 1: camera-to-world (ROS Frame)
    Eigen::Matrix4d T_w_c1_ros = Eigen::Matrix4d::Identity();
    T_w_c1_ros.block<3,3>(0,0) = prev_orientation.toRotationMatrix();
    T_w_c1_ros.block<3,1>(0,3) = prev_position;
    
    // Pose 2: camera-to-world (ROS Frame)
    Eigen::Matrix4d T_w_c2_ros = Eigen::Matrix4d::Identity();
    T_w_c2_ros.block<3,3>(0,0) = curr_orientation.toRotationMatrix();
    T_w_c2_ros.block<3,1>(0,3) = curr_position;

    // Transform from ROS camera frame (Y-forward, X-right, Z-up) 
    // to OpenCV optical frame (Z-forward, X-right, Y-down)
    // -90 deg x-rotation
    Eigen::Matrix4d T_ros_opt = Eigen::Matrix4d::Identity();
    T_ros_opt.block<3,3>(0,0) << 1, 0, 0,
                                 0, 0, 1,
                                  0, -1, 0;

    Eigen::Matrix4d T_w_c1 = T_w_c1_ros * T_ros_opt;
    Eigen::Matrix4d T_w_c2 = T_w_c2_ros * T_ros_opt;
    
    // Invert to get world-to-camera
    Eigen::Matrix4d T_c1_w = T_w_c1.inverse();
    Eigen::Matrix4d T_c2_w = T_w_c2.inverse();
    
    // Extract [R|t] for projection matrices
    Eigen::Matrix3d R1 = T_c1_w.block<3,3>(0,0);
    Eigen::Vector3d t1 = T_c1_w.block<3,1>(0,3);
    
    Eigen::Matrix3d R2 = T_c2_w.block<3,3>(0,0);
    Eigen::Vector3d t2 = T_c2_w.block<3,1>(0,3);
    
    // Convert K to Eigen
    Eigen::Matrix3d K_eigen;
    K_eigen << K.at<double>(0,0), K.at<double>(0,1), K.at<double>(0,2),
               K.at<double>(1,0), K.at<double>(1,1), K.at<double>(1,2),
               K.at<double>(2,0), K.at<double>(2,1), K.at<double>(2,2);
    
    // Build projection matrices P = K[R|t]
    Eigen::Matrix<double, 3, 4> P1;
    P1.block<3,3>(0,0) = K_eigen * R1;
    P1.block<3,1>(0,3) = K_eigen * t1;
    
    Eigen::Matrix<double, 3, 4> P2;
    P2.block<3,3>(0,0) = K_eigen * R2;
    P2.block<3,1>(0,3) = K_eigen * t2;

    // Camera centers in world frame
    Eigen::Vector3d O1 = prev_position;
    Eigen::Vector3d O2 = curr_position;

    for (size_t i = 0; i < pts1.size(); ++i)
    {
        Eigen::Matrix4d A;
        // Row 0: x1 * P1_row3 - P1_row1
        A.row(0) = pts1[i].x * P1.row(2) - P1.row(0);
        // Row 1: y1 * P1_row3 - P1_row2
        A.row(1) = pts1[i].y * P1.row(2) - P1.row(1);
        // Row 2: x2 * P2_row3 - P2_row1
        A.row(2) = pts2[i].x * P2.row(2) - P2.row(0);
        // Row 3: y2 * P2_row3 - P2_row2
        A.row(3) = pts2[i].y * P2.row(2) - P2.row(1);

        Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
        Eigen::Vector4d x3Dh = svd.matrixV().col(3);

        if (std::abs(x3Dh(3)) < 1e-10)
            continue;

        // Euclidean coordinates in World Frame
        Eigen::Vector3d x3D = x3Dh.head(3) / x3Dh(3);

        // Check for finite coordinates
        if (!std::isfinite(x3D(0)) || !std::isfinite(x3D(1)) || !std::isfinite(x3D(2)))
            continue;

        // Check parallax
        Eigen::Vector3d normal1 = x3D - O1;
        double dist1 = normal1.norm();

        Eigen::Vector3d normal2 = x3D - O2;
        double dist2 = normal2.norm();

        if (dist1 < 1e-4 || dist2 < 1e-4)
            continue;

        double cosParallax = normal1.dot(normal2) / (dist1 * dist2);

        // Filter out points with low parallax (rays are nearly parallel)
        // cos(0.5 degrees) ~= 0.99996. 
        // If cosParallax is too close to 1, triangulation is unstable.
        if (cosParallax > 0.99996) 
            continue;

        // Check depth in Camera 1
        Eigen::Vector3d x3D_c1 = R1 * x3D + t1;
        if (x3D_c1(2) <= 0)
            continue;

        // Check depth in Camera 2
        Eigen::Vector3d x3D_c2 = R2 * x3D + t2;
        if (x3D_c2(2) <= 0)
            continue;

        // Reprojection Error Check
        // Project point back to image 1
        Eigen::Vector3d x3D_c1_proj = K_eigen * x3D_c1;
        double u1_proj = x3D_c1_proj(0) / x3D_c1_proj(2);
        double v1_proj = x3D_c1_proj(1) / x3D_c1_proj(2);
        double err1 = std::pow(u1_proj - pts1[i].x, 2) + std::pow(v1_proj - pts1[i].y, 2);

        // Project point back to image 2
        Eigen::Vector3d x3D_c2_proj = K_eigen * x3D_c2;
        double u2_proj = x3D_c2_proj(0) / x3D_c2_proj(2);
        double v2_proj = x3D_c2_proj(1) / x3D_c2_proj(2);
        double err2 = std::pow(u2_proj - pts2[i].x, 2) + std::pow(v2_proj - pts2[i].y, 2);

        // Threshold: 5.991 is Chi-square 95% for 2 DOF (standard in ORB-SLAM)
        // We can be a bit looser here, e.g., 4 pixels squared error
        if (err1 > 5.991 || err2 > 5.991) 
            continue;

        // Filter out points that are too far from the camera (e.g. > 50m)
        if ((x3D - curr_position).norm() > 50.0)
            continue;

        points_3d.push_back(cv::Point3f(x3D(0), x3D(1), x3D(2)));
    }

    // KEYFRAME UPDATE: Only update reference frame after successful triangulation
    prev_keypoints = curr_keypoints;
    prev_descriptors = curr_descriptors.clone();
    prev_position = curr_position;
    prev_orientation = curr_orientation;
}