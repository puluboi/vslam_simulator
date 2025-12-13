# VSLAM Simulator

This project implements a Visual SLAM system split across two branches:
*   **`main`**: Contains the ORB-SLAM algorithm (VSLAM backend).
*   **`master`**: Contains a Raylib-based physics simulator.

The two components communicate via **ROS2**.

## Goal
The simulation will provide realistic IMU and camera data, which the ORB-SLAM system will use for pose estimation & point cloud triangulation.
# VSLAM Simulator

This project implements a Visual SLAM system split across two branches:
*   **`main`**: Contains the ORB-SLAM algorithm (VSLAM backend).
*   **`master`**: Contains a Raylib-based physics simulator.

The two components communicate via **ROS2**.

## Goal
The simulation will provide realistic IMU and camera data, which the ORB-SLAM system will use for pose estimation & point cloud triangulation.



## Timelapse:
The video is from an earlier version: the point cloud is quite noisy compared to the "Full Scan" video.

https://github.com/user-attachments/assets/4c43edb4-3150-4ff5-9d46-410478112b16
## Full Scan
https://github.com/user-attachments/assets/b53b01e1-7a70-497d-b8d4-b224f227b976
