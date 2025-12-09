#include "player.hpp"
#include <math.h>
#include <iostream>

Player::Player(Vector3 startPos)
{
    position = startPos;
    velocity = {0.0f, 0.0f, 0.0f};
    prevVelocity = {0.0f, 0.0f, 0.0f};
    acceleration = {0.0f, 0.0f, 0.0f};
    
    camera.position = position;
    camera.target = {position.x, position.y, position.z - 1.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;
}

const Camera3D Player::getCamera()
{
    return camera;
}

void Player::update()
{
    float dt = GetFrameTime();
    
    handleInput();
    applyPhysics(dt);
    updateCamera();
}

void Player::handleInput()
{
    // Reset acceleration each frame
    acceleration = {0.0f, 0.0f, 0.0f};
    
    // Calculate forward and right vectors based on yaw
    Vector3 forward = {
        sinf(yaw),
        0.0f,
        cosf(yaw)
    };
    
    Vector3 right = {
        cosf(yaw),
        0.0f,
        -sinf(yaw)
    };
    
    // Apply acceleration based on input (only affects acceleration, not velocity directly)
    float forwardInput = (IsKeyDown(KEY_S) - IsKeyDown(KEY_W)) * MOVE_ACCEL;
    float rightInput = (IsKeyDown(KEY_A) - IsKeyDown(KEY_D));
    
    acceleration.x = forwardInput * forward.x - velocity.x*resistance;
    acceleration.z = forwardInput * forward.z - velocity.z*resistance;

    yawVelocity = rightInput*MAX_ANGULAR_SPEED;

}

void Player::applyPhysics(float dt)
{
    // Apply acceleration to velocity: v = v + a * dt
    velocity.x += acceleration.x * dt;
    velocity.y += acceleration.y * dt;
    velocity.z += acceleration.z * dt;
    
    //std::cout<<acceleration.x<<" "<<acceleration.z<< std::endl;
  
    

    
    // Apply velocity to position: p = p + v * dt
    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
    position.z += velocity.z * dt;
    yaw += yawVelocity;
}

void Player::updateCamera()
{
    camera.position = position;
    
    // Calculate target based on yaw and pitch
    camera.target = {
        position.x - sinf(yaw) * cosf(pitch),
        position.y + sinf(pitch),
        position.z - cosf(yaw) * cosf(pitch)
    };

}
Vector3 Player::imuLinearAcceleration(){
    // Get current acceleration and add Gaussian noise to simulate IMU
    float noiseScale = 0.0f;  // Adjust noise magnitude as needed
    
    Vector3 noisyAccel = {
        acceleration.x + (((float)rand() / RAND_MAX) - 0.5f) * 2.0f * noiseScale,
        acceleration.y + (((float)rand() / RAND_MAX) - 0.5f) * 2.0f * noiseScale,
        acceleration.z + (((float)rand() / RAND_MAX) - 0.5f) * 2.0f * noiseScale
    };
    if(noisyAccel.x >5 || noisyAccel.x < -5){
        //std::cout << noisyAccel.x<<std::endl;
    }
    
    return noisyAccel;
}

Vector3 Player::imuGyroAcceleration(){
    // Get current angular velocity and add Gaussian noise to simulate IMU gyroscope
    float noiseScale = 0.0f;  // Adjust noise magnitude as needed
    
    Vector3 noisyGyro = {
        0.0f + (((float)rand() / RAND_MAX) - 0.5f) * 2.0f * noiseScale,  // roll rate (not implemented)
        yawVelocity + (((float)rand() / RAND_MAX) - 0.5f) * 2.0f * noiseScale,  // yaw rate
        0.0f + (((float)rand() / RAND_MAX) - 0.5f) * 2.0f * noiseScale   // pitch rate (not implemented)
    };
    
    return noisyGyro;
}

Vector3 Player::getPosition() const {
    return position;
}

Vector3 Player::getVelocity() const {
    return velocity;
}

geometry_msgs::msg::Pose Player::getPose() const {
    geometry_msgs::msg::Pose pose;
    
    // Convert Raylib coordinates to ROS coordinates
    // Raylib: X=right, Y=up, Z=forward
    // ROS:    X=forward, Y=left, Z=up
    pose.position.x = position.x;   // Raylib Z → ROS X (forward)
    pose.position.y = -position.z;  // Raylib X → ROS Y (left, negated)
    pose.position.z = position.y;   // Raylib Y → ROS Z (up)
    
    // Convert yaw/pitch to quaternion with ROS coordinate system
    // In Raylib: yaw rotates around Y axis (up)
    // In ROS: yaw rotates around Z axis (up)
    // We need to adjust the qufaternion conversion
    
    // For ROS: rotation around Z (yaw), then Y (pitch), then X (roll=0)
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    
    // Quaternion for ROS coordinate system (Z-up, yaw around Z)
    pose.orientation.w = cy * cp;
    pose.orientation.x = -sy * sp;
    pose.orientation.y = cy * sp;
    pose.orientation.z = sy * cp;
    
    return pose;
}

geometry_msgs::msg::Twist Player::getTwist() const {
    geometry_msgs::msg::Twist twist;
    
    // Convert velocity to ROS coordinates
    // Raylib: X=right, Y=up, Z=forward
    // ROS:    X=forward, Y=left, Z=up
    twist.linear.x = velocity.z;   // Raylib Z → ROS X (forward)
    twist.linear.y = -velocity.x;  // Raylib X → ROS Y (left, negated)
    twist.linear.z = velocity.y;   // Raylib Y → ROS Z (up)
    
    // Convert angular velocity to ROS coordinates
    // In ROS, yaw rate is around Z axis (up)
    twist.angular.x = 0.0f;        // roll rate
    twist.angular.y = 0.0f;        // pitch rate  
    twist.angular.z = yawVelocity; // yaw rate around Z (up) in ROS
    
    return twist;
}

