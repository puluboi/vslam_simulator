#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <memory>
#include <Eigen/Geometry>
#include <pose.hpp>
#include <unordered_map>
#include <mutex>

// Forward declare rosConsumer to avoid heavy ROS headers in unit tests
class rosConsumer;

class Orb_Basic
{
public:
    // If start_loop==false skip the ROS spin/display loop (unit tests)
    Orb_Basic(const int feature_count, bool start_loop = true);
    void setIntrinsics(const cv::Mat &K_in);
    // Inject landmarks and matching indices (for unit tests)
    void injectLandmarks(const std::vector<cv::Point3f> &world_pts, const std::vector<cv::Point2f> &image_pts);
    // Exposed for unit tests: run PnP-based pose estimation using current landmarks
    void poseEstimation(const std::vector<cv::Point2f> &pts1,
                       const std::vector<cv::Point2f> &pts2);
    // Last visual position computed by PnP (world frame). Useful for unit tests.
    Eigen::Vector3d last_visual_position;

private:
    std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> featureExtraction();
    void preProcess();
    void Triangulation(const std::vector<cv::Point2f> &pts1,
                       const std::vector<cv::Point2f> &pts2);
    
    Pose pose;
    cv::Mat image;
    std::string frame_id;
    // Last processed IMU timestamp (seconds)
    double last_imu_time = 0.0;

    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::BFMatcher> bf;
    cv::Ptr<cv::CLAHE> clahe;

    cv::Mat curr_descriptors;
    cv::Mat prev_descriptors;
    std::vector<cv::DMatch> matches;

    // Map from prev_keypoint index -> 3D world landmark
    std::unordered_map<int, cv::Point3f> landmarks;
    std::mutex landmarks_mutex; // protect landmarks map
    // For each retained match, index of the previous keypoint (queryIdx)
    std::vector<int> match_prev_idx;
    // For each retained match, index of the current keypoint (trainIdx)
    std::vector<int> match_curr_idx;

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
    // PnP tuning parameters
    int pnp_min_inliers = 10;
    int pnp_min_points = 6;
    double pnp_reproj_thresh = 4.0; // pixels
    int pnp_ransac_iters = 2000;
    double pnp_confidence = 0.999;
    // Visual tracking fallbacks
    int visual_lost_count = 0;
    int visual_lost_threshold = 3; // number of consecutive bad frames before declaring visual lost
    int feature_min_good_matches = 8; // lower threshold to recover from low-feature scenes
};