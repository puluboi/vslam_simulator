#!/bin/bash

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Build 
cmake --build "$SCRIPT_DIR/out/build/VSLAM_SIMULATOR"

# Run
"$SCRIPT_DIR/out/build/VSLAM_SIMULATOR/vslam_simulator"