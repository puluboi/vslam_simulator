cmake_minimum_required(VERSION 3.10)
project(vslam_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Build raylib only if not already built
if(NOT TARGET raylib)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    add_subdirectory(libs/raylib EXCLUDE_FROM_ALL)
endif()

# Collect source files
file(GLOB_RECURSE SOURCES
    src/main.cpp
    src/*/*.cpp
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/game
    ${CMAKE_SOURCE_DIR}/src/ros
)

# Link ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    tf2_ros
)

# Link raylib
target_link_libraries(${PROJECT_NAME}
    raylib
    m pthread dl rt X11
)

