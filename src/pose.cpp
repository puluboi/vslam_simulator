#include "pose.hpp"
#include <iostream>

Pose::Pose(const Eigen::Vector3f &initialPosition, const Eigen::Vector3f &initialVelocity, const Eigen::Quaternionf &initialOrientation)
{
    // Initialize covariance diagonal
    P_.setZero();
    P_.block<3,3>(0,0) = Eigen::Matrix3f::Identity() * sigma_p2;
    P_.block<4,4>(3,3) = Eigen::Matrix4f::Identity() * sigma_q2;
    P_.block<3,3>(7,7) = Eigen::Matrix3f::Identity() * sigma_v2;
    P_.block<3,3>(10,10) = Eigen::Matrix3f::Identity() * sigma_ba2;

    // Initialize state: position, quaternion (w,x,y,z), velocity, biases
    x_.setZero();
    x_.segment<3>(0) = initialPosition;
    x_.segment<4>(3) = Eigen::Vector4f(initialOrientation.w(), initialOrientation.x(), initialOrientation.y(), initialOrientation.z());
    x_.segment<3>(7) = initialVelocity;
    x_.segment<3>(10).setZero();

    // Measurement noise
    R = Eigen::Matrix3f::Identity() * sigma_vision2;
    newVisualT = false;
    // initialize expected measurement to current position
    direction_expected = x_.segment<3>(0);
}

void Pose::sendVisualT(const Eigen::Vector3f &visual_t_)
{
    visual_t = visual_t_;
    newVisualT = true;
}

void Pose::update(const Eigen::Vector3f &linAcc, const Eigen::Vector3f &angVel, float dt_)
{
    dt = dt_;
    a_meas = linAcc;
    w_meas = angVel;
    // Refresh jacobians and process noise, then predict; apply visual correction if available.
    updateF();
    updateQ();

    Predict();
    if (newVisualT)
    {
        newVisualT = false;
        updateH();
        Correct();
    }
}

void Pose::Predict()
{
    Eigen::Vector3f p = x_.segment<3>(0);
    Eigen::Vector4f qv = x_.segment<4>(3);
    Eigen::Quaternionf q(qv(0), qv(1), qv(2), qv(3));
    Eigen::Vector3f v = x_.segment<3>(7);
    Eigen::Vector3f ba = x_.segment<3>(10);

    // If dt is invalid, keep predicted==current
    if (dt <= 0.0f || dt > 1.0f) {
        // Keep predicted == current when dt unreasonable
        predx_ = x_;
        predP_ = P_;
        return;
    }

    // Rotate measured acceleration (body -> world). Accel is assumed linear (gravity removed upstream).
    Eigen::Vector3f acc_world = q * a_meas;

    // Kinematic propagation using world-frame acceleration
    Eigen::Vector3f p_new = p + v * dt + 0.5f * acc_world * dt * dt;
    Eigen::Vector3f v_new = v + acc_world * dt;

    float angle = w_meas.norm() * dt;
    Eigen::Quaternionf q_new = q;
    if (angle > 1e-6f) {
        Eigen::Vector3f axis = w_meas.normalized();
        Eigen::Quaternionf dq(Eigen::AngleAxisf(angle, axis));
        q_new = q * dq;
    }
    q_new.normalize();

    predx_.setZero();
    predx_.segment<3>(0) = p_new;
    predx_.segment<4>(3) = Eigen::Vector4f(q_new.w(), q_new.x(), q_new.y(), q_new.z());
    predx_.segment<3>(7) = v_new;
    predx_.segment<3>(10) = ba;

    // P_predicted = F * P * F^T + Q
    predP_ = F * P_ * F.transpose() + Q;
}
void Pose::Correct()
{
    // expected measurement (position) from predicted state
    direction_expected = predx_.segment<3>(0);

    // basic sanity checks for incoming visual measurement
    if (!visual_t.allFinite()) {
        std::cout << "Correct(): visual_t is not finite, skipping correction" << std::endl;
        return;
    }

    Eigen::Vector3f innovation = visual_t - direction_expected;

    if (!innovation.allFinite()) {
        std::cout << "Correct(): innovation contains non-finite values, skipping correction" << std::endl;
        return;
    }
    // reject absurdly large innovations (likely numerical error)
    const float maxInnovation = 1e8f;
    if (innovation.norm() > maxInnovation) {
        std::cout << "Correct(): innovation too large (" << innovation.norm() << ") skipping correction" << std::endl;
        return;
    }
    // Compute Kalman gain and apply correction
    Eigen::Matrix3f S = H * predP_ * H.transpose() + R;

    Eigen::Matrix<float, 13, 3> Kloc = predP_ * H.transpose() * S.inverse();
    std::cout << "predx_:\n" << predx_.transpose() << std::endl;
    std::cout << "Kloc:\n" << Kloc << std::endl;
    std::cout << "innovation:\n" << innovation.transpose() << std::endl;
    x_ = predx_ + Kloc * innovation;

    // Normalize quaternion in state
    Eigen::Vector4f qv = x_.segment<4>(3);
    float qn = qv.norm();
    if (qn > 1e-8f) qv /= qn;
    x_.segment<4>(3) = qv;

    // Update covariance
    P_ = (Eigen::Matrix<float, 13, 13>::Identity() - Kloc * H) * predP_;
}

Eigen::Vector3f Pose::getPosition() const {
    return x_.segment<3>(0);
}

Eigen::Quaternionf Pose::getOrientation() const {
    Eigen::Vector4f qv = x_.segment<4>(3);
    return Eigen::Quaternionf(qv(0), qv(1), qv(2), qv(3));
}
//-------------------- HELPERS --------------------
void Pose::updateF()
{
    F = Eigen::Matrix<float, 13, 13>::Identity();

    // F[0:3, 10:13] = -0.5 * I₃ * dt²  // ∂p/∂ba
    F.block<3, 3>(0, 10) = -0.5f * Eigen::Matrix3f::Identity() * dt * dt;
    F.block<3, 3>(7, 10) = -Eigen::Matrix3f::Identity() * dt;

    // Quaternion dynamics Jacobian
    Eigen::Matrix4f Omega;
    float wx = w_meas.x(), wy = w_meas.y(), wz = w_meas.z();
    Omega << 0.f, -wx, -wy, -wz,
             wx,  0.f,  wz, -wy,
             wy, -wz,  0.f,  wx,
             wz,  wy, -wx,  0.f;
    F.block<4, 4>(3, 3) = Eigen::Matrix4f::Identity() + 0.5f * Omega * dt;
}

void Pose::updateQ()
{
    Q.setZero();

    // Position noise
    Q.block<3, 3>(0, 0) = sigma_p2 * dt * dt * Eigen::Matrix3f::Identity();
    // Orientation noise (quaternion)
    Q.block<4, 4>(3, 3) = sigma_q2 * dt * Eigen::Matrix4f::Identity();
    // Velocity noise
    Q.block<3, 3>(7, 7) = sigma_v2 * dt * Eigen::Matrix3f::Identity();
    // Bias noise
    Q.block<3, 3>(10, 10) = sigma_ba2 * dt * Eigen::Matrix3f::Identity();
}

void Pose::updateH()
{
    H.setZero();
    // Measure position directly (first 3 states)
    H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
}
