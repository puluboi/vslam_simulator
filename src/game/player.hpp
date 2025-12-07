#pragma once
#include "raylib.h"
#include "raymath.h"
#include "rcamera.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Player
{
public:
    Player(Vector3 position);

    void update();
    const Camera3D getCamera();
    Vector3 imuLinearAcceleration();
    Vector3 imuGyroAcceleration();
    
    // ROS 2 compatible getters
    geometry_msgs::msg::Pose getPose() const;
    geometry_msgs::msg::Twist getTwist() const;
    Vector3 getPosition() const;
    Vector3 getVelocity() const;

private:
    void handleInput();
    void applyPhysics(float dt);
    void updateCamera();

    Camera camera = {0};
    int cameraMode = CAMERA_FIRST_PERSON;

    // Movement state
    Vector3 position = {0};
    Vector3 velocity = {0};
    
    Vector3 prevVelocity = {0}; // For IMU
    Vector3 acceleration = {0};

    // Look direction (yaw/pitch in radians)
    float yaw = 0.0f;
    float yawVelocity = 0.0f;
    float pitch = 0.0f;
    float pitchVelocity = 0.0f;


    // Physics constants
    static constexpr float resistance = 1.0f;
    static constexpr float MOVE_ACCEL = 10.0f;  // Acceleration from input
    static constexpr float MAX_SPEED = 500.0f;  // Max velocity
    static constexpr float FRICTION = 0.80f;    // Deceleration when no input
    static constexpr float MOUSE_SENS = 0.003f; // Mouse sensitivity
    static constexpr float MAX_ANGULAR_SPEED = 0.012f;
    static constexpr float ANGULAR_FRICTION = 10.0f;
};