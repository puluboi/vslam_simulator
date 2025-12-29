#include "orb.hpp"
#include "rosConsumer.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <cstdint>

Orb_Basic::Orb_Basic(const int feature_count, bool start_loop)
    : pose(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity())
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

    // Create matcher and helpers
    bf = cv::BFMatcher::create(cv::NORM_HAMMING, false);
    if (!bf)
        std::cout << "Orb_Basic ctor: bf is null" << std::endl;
    else
        std::cout << "Orb_Basic ctor: bf initialized" << std::endl;

    // If requested, skip ROS spin/display loop (used in tests)
    if (!start_loop)
    {
        clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        return;
    }

    consumer_ = std::make_shared<rosConsumer>();

    clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    // Create window for displaying images
    cv::namedWindow("Latest Image", cv::WINDOW_AUTOSIZE);
    // Main loop: process ROS messages and display images
    rclcpp::Rate rate(30); // 30 Hz
    int64_t t_prev = 0;
    int64_t t = 0;
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
            // std::cout << "Camera Intrinsics:\n";
            // std::cout << "fx: " << intrinsics.fx << ", fy: " << intrinsics.fy
            //           << ", cx: " << intrinsics.cx << ", cy: " << intrinsics.cy << std::endl;
            // std::cout << "K:\n" << K << std::endl;
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

                // Get pose synchronized to image timestamp
                rclcpp::Time image_stamp = ros_image->header.stamp;
                auto ros_pose = consumer_->getPoseAtTime(image_stamp);
                // Update current position from ROS pose
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

                // Initialize previous frame data on first frame
                if (prev_descriptors.empty())
                {
                    orb->detectAndCompute(image, cv::noArray(), prev_keypoints, prev_descriptors);
                    prev_position = curr_position;
                    prev_orientation = curr_orientation;
                    continue;
                }
                preProcess();
                auto pts = featureExtraction();
                auto imu = consumer_->getLatestImu();
                if (imu)
                {
                    std::cout << "acc=("
                              << imu->linear_acceleration.x << ", "
                              << imu->linear_acceleration.y << ", "
                              << imu->linear_acceleration.z << ") gyro=("
                              << imu->angular_velocity.x << ", " << imu->angular_velocity.y << ", " << imu->angular_velocity.z << ")\n";

                    // Prefer IMU header timestamp
                    rclcpp::Time now(imu->header.stamp);
                    if (t_prev == 0)
                    {
                        t_prev = now.nanoseconds();
                    }
                    t = now.nanoseconds();
                    float dt = static_cast<float>(t - t_prev) * 1e-9f;
                    std::cout << "Time:" << dt << std::endl;
                    // Update only when dt is reasonable
                    if (dt > 0.001f && dt < 1.0f)
                    {
                        Eigen::Vector3f linAcc(static_cast<float>(imu->linear_acceleration.x),
                                               static_cast<float>(imu->linear_acceleration.y),
                                               static_cast<float>(imu->linear_acceleration.z));
                        Eigen::Vector3f angVel(static_cast<float>(imu->angular_velocity.x),
                                               static_cast<float>(imu->angular_velocity.y),
                                               static_cast<float>(imu->angular_velocity.z));

                                // IMU sanity checks: ignore NaN/Inf or physically implausible values only
                                float a_norm = linAcc.norm();
                                if (!std::isfinite(a_norm)) {
                                    std::cout << "IMU: accel not finite, skipping update" << std::endl;
                                    t_prev = t; // advance timestamp to avoid dt accumulation
                                } else if (a_norm > 200.0f) {
                                    std::cout << "IMU: accel magnitude implausible (" << a_norm << "), skipping update" << std::endl;
                                    t_prev = t;
                                } else {
                                    // Accept small accelerations (static device possible)
                                    pose.update(linAcc, angVel, dt);
                                    t_prev = t;
                                }
                    }
                }
                else
                {
                    std::cout << "No IMU data yet\n";
                }
                // Run visual pose estimation
                poseEstimation(pts.first, pts.second);
                // If no IMU data was available in this loop (e.g. imu topic missing or messages filtered),
                // perform a small visual-only predict/correct step so visual measurements are applied.
                if (!imu)
                {
                    // Small fixed dt matching main loop rate; zero imu inputs
                    constexpr float vis_dt = 1.0f / 30.0f;
                    pose.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), vis_dt);
                }
                // Print estimator output; include GT when available
                Eigen::Vector3f est_pos = pose.getPosition();
                Eigen::Quaternionf est_q = pose.getOrientation();
                if (ros_pose)
                {
                    std::cout << "GT pos: " << curr_position.transpose() << " GT ori (w,x,y,z): "
                              << curr_orientation.w() << "," << curr_orientation.x() << "," << curr_orientation.y() << "," << curr_orientation.z() << "\n";
                }
                else
                {
                    std::cout << "GT pos: <unavailable> ";
                }
                std::cout << "Est pos: " << est_pos.transpose() << " Est ori (w,x,y,z): "
                          << est_q.w() << "," << est_q.x() << "," << est_q.y() << "," << est_q.z() << "\n";

                Triangulation(pts.first, pts.second);
                // Publish 3D points to ROS2 in world frame
                if (!points_3d.empty())
                {
                    consumer_->publishPoints3D(points_3d, "world");
                    // RCLCPP_INFO(consumer_->get_logger(), "Triangulated %zu 3D points. Example: (%.2f, %.2f, %.2f)",
                    //             points_3d.size(), points_3d[3].x, points_3d[3].y, points_3d[3].z);
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

void Orb_Basic::setIntrinsics(const cv::Mat &K_in)
{
    K = K_in.clone();
}

void Orb_Basic::injectLandmarks(const std::vector<cv::Point3f> &world_pts, const std::vector<cv::Point2f> &image_pts)
{
    std::lock_guard<std::mutex> lk(landmarks_mutex);
    landmarks.clear();
    match_prev_idx.clear();
    match_curr_idx.clear();
    for (size_t i = 0; i < world_pts.size(); ++i)
    {
        landmarks[(int)i] = world_pts[i];
        match_prev_idx.push_back((int)i);
        match_curr_idx.push_back((int)i);
    }
}

std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> Orb_Basic::featureExtraction()
{
    // Pose is already synchronized in main loop - no need to fetch again
    orb->detectAndCompute(image, cv::noArray(), curr_keypoints, curr_descriptors);

    cv::Mat output;
    std::vector<cv::Point2f> pts1, pts2;
    // Track keypoints between frames
    if (!prev_descriptors.empty() && !curr_descriptors.empty())
    {
        // Defensive checks: ensure descriptors have rows and compatible types
        if (prev_descriptors.rows < 2 || curr_descriptors.rows < 2)
        {
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            prev_position = curr_position;
            prev_orientation = curr_orientation;
            return std::make_pair(std::vector<cv::Point2f>(), std::vector<cv::Point2f>());
        }

        // Prepare descriptor mats to pass to matcher; BFMatcher with Hamming expects CV_8U descriptors
        cv::Mat prev_desc_for_match = prev_descriptors;
        cv::Mat curr_desc_for_match = curr_descriptors;
        if (prev_descriptors.type() != CV_8U)
            prev_descriptors.convertTo(prev_desc_for_match, CV_8U);
        if (curr_descriptors.type() != CV_8U)
            curr_descriptors.convertTo(curr_desc_for_match, CV_8U);

        // Use kNN match for Ratio Test (Lowe's Ratio Test)
        std::vector<std::vector<cv::DMatch>> knn_matches;
        // Extra defensive guards: ensure data pointers and sizes look sane
        if (prev_desc_for_match.empty() || curr_desc_for_match.empty() || prev_desc_for_match.data == nullptr || curr_desc_for_match.data == nullptr)
        {
            std::cout << "featureExtraction: descriptors empty or invalid, skipping matching" << std::endl;
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            prev_position = curr_position;
            prev_orientation = curr_orientation;
            return std::make_pair(std::vector<cv::Point2f>(), std::vector<cv::Point2f>());
        }
        if (prev_desc_for_match.rows < 1 || curr_desc_for_match.rows < 1 || prev_desc_for_match.cols < 1 || curr_desc_for_match.cols < 1)
        {
            std::cout << "featureExtraction: descriptors have invalid shape, skipping" << std::endl;
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            prev_position = curr_position;
            prev_orientation = curr_orientation;
            return std::make_pair(std::vector<cv::Point2f>(), std::vector<cv::Point2f>());
        }
        if (prev_desc_for_match.type() != curr_desc_for_match.type())
        {
            curr_desc_for_match.convertTo(curr_desc_for_match, prev_desc_for_match.type());
        }
        if (prev_desc_for_match.cols != curr_desc_for_match.cols)
        {
            std::cout << "featureExtraction: descriptor size mismatch (" << prev_desc_for_match.cols << " vs " << curr_desc_for_match.cols << "), skipping" << std::endl;
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            prev_position = curr_position;
            prev_orientation = curr_orientation;
            return std::make_pair(std::vector<cv::Point2f>(), std::vector<cv::Point2f>());
        }
        if (!bf)
        {
            std::cout << "featureExtraction: bf matcher null" << std::endl;
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            prev_position = curr_position;
            prev_orientation = curr_orientation;
            return std::make_pair(std::vector<cv::Point2f>(), std::vector<cv::Point2f>());
        }

        try
        {
            bf->knnMatch(prev_desc_for_match, curr_desc_for_match, knn_matches, 2);
        }
        catch (const cv::Exception &e)
        {
            std::cout << "featureExtraction: knnMatch failed: " << e.what() << std::endl;
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            prev_position = curr_position;
            prev_orientation = curr_orientation;
            return std::make_pair(std::vector<cv::Point2f>(), std::vector<cv::Point2f>());
        }

        std::vector<cv::DMatch> good_matches;
        const float ratio_thresh = 0.75f; // a bit more permissive
        const int TH_LOW = 80;            // Absolute threshold (allow slightly larger distances in low-texture)

        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i].size() >= 2)
            {
                int bestDist = knn_matches[i][0].distance;
                int secondDist = knn_matches[i][1].distance;

                // Both ratio test AND absolute threshold
                if (bestDist < ratio_thresh * secondDist && bestDist < TH_LOW)
                {
                    good_matches.push_back(knn_matches[i][0]);
                }
            }
        }

        // Draw tracking trails on current frame
        const cv::Mat tracking_viz = image.clone();

        pts1.reserve(good_matches.size());
        pts2.reserve(good_matches.size());
        match_prev_idx.clear();
        match_curr_idx.clear();
        match_prev_idx.reserve(good_matches.size());
        match_curr_idx.reserve(good_matches.size());

        // If tracking is degraded (too few matches), avoid immediately throwing away the previous keyframe.
        if (good_matches.size() < static_cast<size_t>(feature_min_good_matches))
        {
            visual_lost_count++;
            std::cout << "featureExtraction: too few matches (" << good_matches.size() << "), visual_lost_count=" << visual_lost_count << std::endl;
            if (visual_lost_count >= visual_lost_threshold)
            {
                // After consecutive failures, reset reference to current frame to attempt recovery
                prev_keypoints = curr_keypoints;
                prev_descriptors = curr_descriptors.clone();
                prev_position = curr_position;
                prev_orientation = curr_orientation;
                visual_lost_count = 0;
            }
            return std::make_pair(std::vector<cv::Point2f>(), std::vector<cv::Point2f>());
        }
        else
        {
            visual_lost_count = 0;
        }

        for (const auto &match : good_matches)
        {

            cv::Point2f prev_pt = prev_keypoints[match.queryIdx].pt;
            cv::Point2f curr_pt = curr_keypoints[match.trainIdx].pt;
            int octave1 = prev_keypoints[match.queryIdx].octave;
            int octave2 = curr_keypoints[match.trainIdx].octave;

            // Reject if scale difference > 1 level
            if (std::abs(octave1 - octave2) > 1)
                continue;
            pts1.push_back(prev_pt);
            pts2.push_back(curr_pt);
            // Preserve mapping: which prev-keypoint index corresponds to this match, and which current index
            match_prev_idx.push_back(match.queryIdx);
            match_curr_idx.push_back(match.trainIdx);

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
void Orb_Basic::poseEstimation(const std::vector<cv::Point2f> &pts1,
                               const std::vector<cv::Point2f> &pts2)
{
    // PnP-based pose estimation (3D-2D)
    if (K.empty())
    {
        std::cout << "poseEstimation: camera intrinsics not ready" << std::endl;
        return;
    }

    if (pts2.empty() || match_prev_idx.empty() || match_curr_idx.empty())
    {
        // Nothing to do
        return;
    }

    // Build 3D-2D correspondences using existing landmarks
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;

    {
        std::lock_guard<std::mutex> lk(landmarks_mutex);
        for (size_t i = 0; i < pts2.size() && i < match_prev_idx.size() && i < match_curr_idx.size(); ++i)
        {
            int prev_idx = match_prev_idx[i];
            // Prefer landmark keyed to the future prev_keypoint (i.e. trainIdx)
            int curr_idx = match_curr_idx[i];
            // Try find a landmark for the prev index first, else curr index
            auto it = landmarks.find(prev_idx);
            if (it == landmarks.end())
                it = landmarks.find(curr_idx);

            if (it != landmarks.end())
            {
                objectPoints.push_back(it->second);
                imagePoints.push_back(pts2[i]);
            }
        }
    }

    if ((int)objectPoints.size() < pnp_min_points)
    {
        std::cout << "poseEstimation: insufficient 3D-2D correspondences: " << objectPoints.size() << std::endl;
        return;
    }

    cv::Mat distCoeffs; // assume zero distortion
    // ensure camera matrix is double-precision for downstream math
    cv::Mat Kd;
    if (K.type() != CV_64F)
        K.convertTo(Kd, CV_64F);
    else
        Kd = K;

    cv::Mat rvec, tvec;
    cv::Mat inliers;

    bool pnp_ok = false;
    try
    {
        pnp_ok = cv::solvePnPRansac(objectPoints, imagePoints, Kd, distCoeffs,
                                    rvec, tvec, false,
                                    pnp_ransac_iters, pnp_reproj_thresh,
                                    pnp_confidence, inliers, cv::SOLVEPNP_AP3P);
    }
    catch (const cv::Exception &e)
    {
        std::cout << "solvePnPRansac failed: " << e.what() << std::endl;
        return;
    }

    if (!pnp_ok || inliers.empty())
    {
        std::cout << "poseEstimation: PnP RANSAC failed or no inliers" << std::endl;
        return;
    }

    int inlier_count = inliers.rows;
    if (inlier_count < pnp_min_inliers)
    {
        std::cout << "poseEstimation: too few inliers: " << inlier_count << std::endl;
        return;
    }

    // Optional refinement with Levenberg-Marquardt
    try
    {
        // refine only using inliers
        std::vector<cv::Point3f> obj_inl; obj_inl.reserve(inlier_count);
        std::vector<cv::Point2f> img_inl; img_inl.reserve(inlier_count);
        auto getInlierIdx = [&](int row)->int {
            // handle common inliers types robustly
            int idx = -1;
            int depth = inliers.type() & CV_MAT_DEPTH_MASK;
            if (depth == CV_8U) idx = static_cast<int>(inliers.at<unsigned char>(row, 0));
            else if (depth == CV_32S) idx = inliers.at<int>(row, 0);
            else if (depth == CV_32F) idx = static_cast<int>(inliers.at<float>(row, 0));
            else if (depth == CV_64F) idx = static_cast<int>(inliers.at<double>(row, 0));
            else idx = inliers.at<int>(row, 0);
            return idx;
        };

        for (int i = 0; i < inlier_count; ++i)
        {
            int idx = getInlierIdx(i);
            if (idx < 0 || idx >= (int)objectPoints.size()) continue;
            obj_inl.push_back(objectPoints[idx]);
            img_inl.push_back(imagePoints[idx]);
        }
        if (!obj_inl.empty())
        {
            // try LM refine, fallback to iterative solvePnP
            try { cv::solvePnPRefineLM(obj_inl, img_inl, Kd, distCoeffs, rvec, tvec); }
            catch (...) { cv::solvePnP(obj_inl, img_inl, Kd, distCoeffs, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE); }
        }
    }
    catch (...) { /* refinement best-effort */ }

    // Compute median reprojection error on inliers
    std::vector<double> errors; errors.reserve(inlier_count);
    std::vector<cv::Point2f> proj;
    cv::projectPoints(objectPoints, rvec, tvec, Kd, distCoeffs, proj);
    for (int i = 0; i < inlier_count; ++i)
    {
        int idx = -1;
        // reuse robust inlier index read
        int depth = inliers.type() & CV_MAT_DEPTH_MASK;
        if (depth == CV_8U) idx = static_cast<int>(inliers.at<unsigned char>(i, 0));
        else if (depth == CV_32S) idx = inliers.at<int>(i, 0);
        else if (depth == CV_32F) idx = static_cast<int>(inliers.at<float>(i, 0));
        else if (depth == CV_64F) idx = static_cast<int>(inliers.at<double>(i, 0));
        else idx = inliers.at<int>(i, 0);

        if (idx < 0 || idx >= (int)proj.size()) continue;
        double dx = proj[idx].x - imagePoints[idx].x;
        double dy = proj[idx].y - imagePoints[idx].y;
        errors.push_back(std::sqrt(dx * dx + dy * dy));
    }
    if (errors.empty())
    {
        std::cout << "poseEstimation: no reprojection errors computed" << std::endl;
        return;
    }
    std::nth_element(errors.begin(), errors.begin() + errors.size() / 2, errors.end());
    double median_err = errors[errors.size() / 2];

    if (median_err > pnp_reproj_thresh)
    {
        std::cout << "poseEstimation: high reprojection error " << median_err << std::endl;
        return;
    }

    // Convert rvec/tvec -> world-frame camera pose
    // Robustly convert rvec/tvec to double and build rotation matrix in double
    // Read rvec/tvec values regardless of their stored depth
    auto readMatDouble = [&](const cv::Mat &m, int idx)->double {
        int rows = m.rows, cols = m.cols;
        int depth = m.type() & CV_MAT_DEPTH_MASK;
        if (depth == CV_64F)
        {
            if (cols == 1) return m.at<double>(idx, 0);
            return m.at<double>(0, idx);
        }
        else if (depth == CV_32F)
        {
            if (cols == 1) return static_cast<double>(m.at<float>(idx, 0));
            return static_cast<double>(m.at<float>(0, idx));
        }
        else
        {
            cv::Mat tmp; m.convertTo(tmp, CV_64F);
            if (tmp.cols == 1) return tmp.at<double>(idx, 0);
            return tmp.at<double>(0, idx);
        }
    };

    cv::Mat R_cv;
    // Validate rvec components before Rodrigues
    double r0 = readMatDouble(rvec, 0);
    double r1 = readMatDouble(rvec, 1);
    double r2 = readMatDouble(rvec, 2);
    if (!std::isfinite(r0) || !std::isfinite(r1) || !std::isfinite(r2))
    {
        std::cout << "poseEstimation: rejected PnP - non-finite rvec" << std::endl;
        return;
    }
    cv::Rodrigues(rvec, R_cv);
    cv::Mat R_cv_d;
    R_cv.convertTo(R_cv_d, CV_64F);
    // Quick sanity: all elements finite
    bool R_finite = true;
    for (int r = 0; r < R_cv_d.rows && R_finite; ++r)
        for (int c = 0; c < R_cv_d.cols; ++c)
            if (!std::isfinite(R_cv_d.at<double>(r, c))) { R_finite = false; break; }
    if (!R_finite)
    {
        std::cout << "poseEstimation: rejected PnP - rotation matrix contains non-finite" << std::endl;
        return;
    }
    Eigen::Matrix3d R_eig;
    cv::cv2eigen(R_cv_d, R_eig);
    // check rotation determinant is close to 1
    double detR = R_eig.determinant();
    if (!std::isfinite(detR) || std::abs(detR - 1.0) > 1e-2)
    {
        std::cout << "poseEstimation: rejected PnP - invalid rotation det=" << detR << std::endl;
        return;
    }

    Eigen::Vector3d t_eig;
    t_eig << readMatDouble(tvec, 0), readMatDouble(tvec, 1), readMatDouble(tvec, 2);
    // Basic sanity checks on tvec
    if (!std::isfinite(t_eig(0)) || !std::isfinite(t_eig(1)) || !std::isfinite(t_eig(2)))
    {
        std::cout << "poseEstimation: rejected PnP - non-finite translation" << std::endl;
        return;
    }
    if (t_eig.norm() > 1e5)
    {
        std::cout << "poseEstimation: rejected PnP - translation magnitude too large: " << t_eig.norm() << std::endl;
        return;
    }

    // Camera in world (optical frame): X_cam = R * X_world + t  => T_cam_w = [R|t]
    // World-to-camera (optical) inverse: T_w_cam_opt = [R^T | -R^T t]
    Eigen::Matrix3d R_w_cam_opt = R_eig.transpose();
    Eigen::Vector3d t_w_cam_opt = -R_w_cam_opt * t_eig;

    // ROS->Optical used in Triangulation: T_ros_opt (maps ROS camera frame -> OpenCV optical)
    Eigen::Matrix3d R_ros_opt;
    R_ros_opt << 1, 0, 0,
                 0, 0, 1,
                 0, -1, 0;
    Eigen::Matrix3d R_opt_ros = R_ros_opt.inverse();

    // Compose T_w_cam_ros = T_w_cam_opt * T_opt_ros
    Eigen::Matrix3d R_w_cam_ros = R_w_cam_opt * R_opt_ros;
    Eigen::Vector3d t_w_cam_ros = R_w_cam_opt * Eigen::Vector3d::Zero() + t_w_cam_opt; // no translation in T_opt_ros
    // Note: T_opt_ros is pure rotation, so translation unchanged

    Eigen::Vector3d new_position = t_w_cam_ros;
    // Guard against non-finite or absurd positions before publishing
    if (!std::isfinite(new_position(0)) || !std::isfinite(new_position(1)) || !std::isfinite(new_position(2)) || new_position.norm() > 1e5)
    {
        std::cout << "poseEstimation: rejected PnP - resulting world position invalid or out-of-range: " << new_position.transpose() << std::endl;
        return;
    }

    Eigen::Vector3f position_measurement = new_position.cast<float>();
    // store for tests and publish to pose estimator
    last_visual_position = new_position;
    pose.sendVisualT(position_measurement);
    std::cout << "poseEstimation: accepted PnP pose. inliers=" << inlier_count << " median_reproj=" << median_err << std::endl;
}
void Orb_Basic::Triangulation(const std::vector<cv::Point2f> &pts1,
                              const std::vector<cv::Point2f> &pts2)
{
    // points_3d.clear();

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
    // Allow smaller baselines during initialization to bootstrap landmarks, but avoid degenerate cases.
    double min_baseline = 0.02; // reduced threshold to allow initialization in small motions
    if (baseline < min_baseline)
    {
        RCLCPP_DEBUG(consumer_->get_logger(), "Baseline too small: %.3f m. Skipping triangulation.", baseline);
        return;
    }

    RCLCPP_INFO(consumer_->get_logger(), "Baseline: %.3f m - triangulating", baseline);

    // Pose 1: camera-to-world (ROS Frame)
    Eigen::Matrix4d T_w_c1_ros = Eigen::Matrix4d::Identity();
    T_w_c1_ros.block<3, 3>(0, 0) = prev_orientation.toRotationMatrix();
    T_w_c1_ros.block<3, 1>(0, 3) = prev_position;

    // Pose 2: camera-to-world (ROS Frame)
    Eigen::Matrix4d T_w_c2_ros = Eigen::Matrix4d::Identity();
    T_w_c2_ros.block<3, 3>(0, 0) = curr_orientation.toRotationMatrix();
    T_w_c2_ros.block<3, 1>(0, 3) = curr_position;

    // Transform from ROS camera frame (Y-forward, X-right, Z-up)
    // to OpenCV optical frame (Z-forward, X-right, Y-down)
    // -90 deg x-rotation
    Eigen::Matrix4d T_ros_opt = Eigen::Matrix4d::Identity();
    T_ros_opt.block<3, 3>(0, 0) << 1, 0, 0,
        0, 0, 1,
        0, -1, 0;

    Eigen::Matrix4d T_w_c1 = T_w_c1_ros * T_ros_opt;
    Eigen::Matrix4d T_w_c2 = T_w_c2_ros * T_ros_opt;

    // Invert to get world-to-camera
    Eigen::Matrix4d T_c1_w = T_w_c1.inverse();
    Eigen::Matrix4d T_c2_w = T_w_c2.inverse();

    // Extract [R|t] for projection matrices
    Eigen::Matrix3d R1 = T_c1_w.block<3, 3>(0, 0);
    Eigen::Vector3d t1 = T_c1_w.block<3, 1>(0, 3);

    Eigen::Matrix3d R2 = T_c2_w.block<3, 3>(0, 0);
    Eigen::Vector3d t2 = T_c2_w.block<3, 1>(0, 3);

    // Convert K to Eigen
    Eigen::Matrix3d K_eigen;
    K_eigen << K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
        K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
        K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

    // Build projection matrices P = K[R|t]
    Eigen::Matrix<double, 3, 4> P1;
    P1.block<3, 3>(0, 0) = K_eigen * R1;
    P1.block<3, 1>(0, 3) = K_eigen * t1;

    Eigen::Matrix<double, 3, 4> P2;
    P2.block<3, 3>(0, 0) = K_eigen * R2;
    P2.block<3, 1>(0, 3) = K_eigen * t2;

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

        cv::Point3f world_pt(static_cast<float>(x3D(0)), static_cast<float>(x3D(1)), static_cast<float>(x3D(2)));
        points_3d.push_back(world_pt);
        points_3d_world.push_back(world_pt);
        // Store landmark keyed to the current-frame keypoint index (trainIdx),
        // which will become prev_keypoints on the next iteration.
        if (i < match_curr_idx.size())
        {
            int key = match_curr_idx[i];
            std::lock_guard<std::mutex> lk(landmarks_mutex);
            landmarks[key] = world_pt;
        }
    }

    // KEYFRAME UPDATE: Only update reference frame after successful triangulation
    prev_keypoints = curr_keypoints;
    prev_descriptors = curr_descriptors.clone();
    prev_position = curr_position;
    prev_orientation = curr_orientation;
}