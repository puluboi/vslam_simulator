# VSLAM Simulator

A visual SLAM (Simultaneous Localization and Mapping) simulator.

## Dependencies

### raylib

This project requires [raylib](https://github.com/raysan5/raylib) for graphics rendering.

#### Installation on Linux (Ubuntu/Debian)

1. Install build dependencies:

```bash
sudo apt install build-essential git libasound2-dev libx11-dev libxrandr-dev libxi-dev libgl1-mesa-dev libglu1-mesa-dev libxcursor-dev libxinerama-dev libwayland-dev libxkbcommon-dev
```

2. Clone and build raylib:
cd ~/ros2_kilted && rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```bash
git clone --depth 1 https://github.com/raysan5/raylib.git
cd raylib/src
make PLATFORM=PLATFORM_DESKTOP
sudo make install
sudo ldconfig
```

3. Verify installation:

```bash
pkg-config --libs raylib
```

## Building

TODO: Add build instructions for vslam_simulator

## Usage

TODO: Add usage instructions

