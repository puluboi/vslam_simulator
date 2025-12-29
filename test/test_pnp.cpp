#include <iostream>
#include <random>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "orb.hpp"

int main()
{
    // Synthetic camera intrinsics
    double fx = 400.0, fy = 400.0, cx = 320.0, cy = 240.0;
    cv::Mat K = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    // Create Orb_Basic for a headless unit test (no ROS loop)
    Orb_Basic orb(500, false);
    orb.setIntrinsics(K);

    // Ground truth camera pose in ROS frame (rotation and translation)
    Eigen::Matrix3d R_cam = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Vector3d t_cam(0.5, -0.2, 1.2); // camera position in world (ROS frame)

    // ROS->Optical rotation used by Triangulation
    Eigen::Matrix3d R_ros_opt;
    R_ros_opt << 1, 0, 0,
                 0, 0, 1,
                 0, -1, 0;

    // T_w_c_ros (camera-to-world in ROS frame): we store as T_w_c_ros with R_cam and t_cam
    Eigen::Matrix4d T_w_c_ros = Eigen::Matrix4d::Identity();
    T_w_c_ros.block<3,3>(0,0) = R_cam;
    T_w_c_ros.block<3,1>(0,3) = t_cam;

    // Convert to optical frame: T_w_c_opt = T_w_c_ros * T_ros_opt
    Eigen::Matrix4d T_ros_opt = Eigen::Matrix4d::Identity();
    T_ros_opt.block<3,3>(0,0) = R_ros_opt;
    Eigen::Matrix4d T_w_c_opt = T_w_c_ros * T_ros_opt;
    Eigen::Matrix4d T_c_w_opt = T_w_c_opt.inverse();
    Eigen::Matrix3d Rcw = T_c_w_opt.block<3,3>(0,0);
    Eigen::Vector3d tcw = T_c_w_opt.block<3,1>(0,3);

    // Generate synthetic 3D points in world frame and project to image
    std::vector<cv::Point3f> world_pts;
    std::vector<cv::Point2f> image_pts;
    std::mt19937 rng(12345);
    std::normal_distribution<double> noise(0.0, 1.0); // 1 pixel noise

    for (int i = 0; i < 200; ++i)
    {
        // sample points in front of camera
        double X = ((double)rng() / rng.max() - 0.5) * 2.0; // -1..1
        double Y = ((double)rng() / rng.max() - 0.5) * 1.0; // -0.5..0.5
        double Z = 2.0 + ((double)rng() / rng.max()) * 8.0; // 2..10m
        Eigen::Vector3d Pw(X, Y, Z);

        // Project to image using camera extrinsics and optical transform
        Eigen::Vector3d Pc = Rcw * Pw + tcw; // point in camera optical frame
        if (Pc.z() <= 0) continue;
        double u = (fx * Pc.x() / Pc.z()) + cx + noise(rng);
        double v = (fy * Pc.y() / Pc.z()) + cy + noise(rng);
        world_pts.emplace_back((float)Pw.x(), (float)Pw.y(), (float)Pw.z());
        image_pts.emplace_back((float)u, (float)v);
    }

    if (world_pts.size() < 10) {
        std::cerr << "Not enough synthetic points" << std::endl;
        return 2;
    }

    // Inject landmarks and matches for the test
    orb.injectLandmarks(world_pts, image_pts);
    // Call PnP pose estimation
    std::vector<cv::Point2f> dummy_prev; // not used
    orb.poseEstimation(dummy_prev, image_pts);

    // Compare orb.last_visual_position to t_cam (ground truth)
    Eigen::Vector3d recovered = orb.last_visual_position;
    double err = (recovered - t_cam).norm();
    std::cout << "GT t_cam: " << t_cam.transpose() << " recovered: " << recovered.transpose() << " error=" << err << std::endl;

    if (err < 0.2)
    {
        std::cout << "TEST PASSED" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "TEST FAILED: error too large" << std::endl;
        return 1;
    }
}
