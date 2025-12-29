cmake_minimum_required(VERSION 3.10.0)
project(orb_basic VERSION 0.1.0 LANGUAGES C CXX)

# Find packages first
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)

# Collect source files
file(GLOB CORE_SRCS src/*.cpp)

# Build a library from source files so unit tests can link against implementation
add_library(orb_core STATIC ${CORE_SRCS})
target_include_directories(orb_core PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(orb_core
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    tf2_ros
    cv_bridge
)
target_link_libraries(orb_core ${OpenCV_LIBS})

# Main executable links against orb_core
add_executable(${PROJECT_NAME} main.cpp)

# ROS 2 dependencies for main executable
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    tf2_ros
    cv_bridge
)
target_link_libraries(${PROJECT_NAME} orb_core ${OpenCV_LIBS})

include(CTest)
enable_testing()

# Unit test: PnP synthetic
add_executable(test_pnp test/test_pnp.cpp)
target_include_directories(test_pnp PRIVATE ${CMAKE_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(test_pnp
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    tf2_ros
    cv_bridge
)
target_link_libraries(test_pnp orb_core ${OpenCV_LIBS})
add_test(NAME test_pnp COMMAND test_pnp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
