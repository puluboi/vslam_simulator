#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
/// Pose estimator running alongside the visual estimator.
/// When a visual position arrives it is used to correct the state.
class Pose
{
public:
    /// Construct with initial position, velocity, and orientation.
    Pose(const Eigen::Vector3f &initialPosition,
         const Eigen::Vector3f &initialVelocity,
         const Eigen::Quaternionf &initialOrientation);

    void sendVisualT(const Eigen::Vector3f &visual_t_);

    void update(const Eigen::Vector3f &linAcc, const Eigen::Vector3f &angVel, float dt_);

    // Accessors for external inspection (ground-truth comparison)
    Eigen::Vector3f getPosition() const;
    Eigen::Quaternionf getOrientation() const;

private:
    Eigen::Vector3f visual_t;
    Eigen::Vector3f direction_expected;

    Eigen::Matrix<float, 13, 3> K; // Kalman Gain (state_dim x meas_dim)
    Eigen::Matrix<float, 3, 13> H; // Measurement Model (meas_dim x state_dim)
    Eigen::Matrix<float, 13, 13> F; // State Transition Jacobian;
    Eigen::Matrix<float, 13, 13> Q; // Process Noise Jacobian;
        Eigen::Matrix3f R; // measurement noise (set in constructor)

    Eigen::Matrix<float, 13, 1> x_;     // state
    Eigen::Matrix<float, 13, 1> predx_; // predicted state
        // Tuned initial uncertainties (variance): increase position/velocity uncertainty so visual corrections can pull state
        // and give the filter flexibility during initialization/low-visibility periods.
        float sigma_p2 = 1.0f, sigma_q2 = 0.01f, sigma_v2 = 1.0f, sigma_ba2 = 0.0001f, sigma_vision2 = 0.25f; // pos, ori, vel, bias, vision variance
    Eigen::Matrix<float, 13, 13> P_;
    Eigen::Matrix<float, 13, 13> predP_;
    float dt;
    float prevTime;

    Eigen::Vector3f a_meas;
    Eigen::Vector3f w_meas;
    
    bool newVisualT; // when the orb class has processed a new pose estimate, we can correct our imu integration based on it.

    void Predict();
    void updateF();
    void updateQ();
    void updateH();
    void Correct();
};