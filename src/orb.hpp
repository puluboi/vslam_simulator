#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "rosConsumer.hpp"
#include <Eigen/Geometry>

class Orb_Basic
{
public:
    Orb_Basic(const int feature_count);

private:
    std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> featureExtraction();
    void poseEstimation();
    void Triangulation(const std::vector<cv::Point2f> &pts1,
                       const std::vector<cv::Point2f> &pts2);

    cv::Mat image;
    std::string frame_id;

    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::BFMatcher> bf;

    cv::Mat curr_descriptors;
    cv::Mat prev_descriptors;
    std::vector<cv::DMatch> matches;

    std::vector<cv::KeyPoint> curr_keypoints;
    std::vector<cv::KeyPoint> prev_keypoints;

    std::vector<cv::Point3f> points_3d;
    std::vector<cv::Point3f> points_3d_world;
    Eigen::Vector3d prev_position;
    Eigen::Quaterniond prev_orientation;
    Eigen::Vector3d curr_position;
    Eigen::Quaterniond curr_orientation;
    cv::Mat K; // Camera Intrinsic params
    std::shared_ptr<rosConsumer> consumer_;
};