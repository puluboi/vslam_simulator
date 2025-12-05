cmake_minimum_required(VERSION 3.16)
project(vslam_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find raylib using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(RAYLIB REQUIRED raylib)


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
    ${RAYLIB_INCLUDE_DIRS}
)

# Link raylib
target_link_libraries(${PROJECT_NAME}
    ${RAYLIB_LIBRARIES}
    GL m pthread dl rt X11
)