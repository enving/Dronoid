#!/bin/bash
# Dronoid - Setup Dependencies
# This script downloads and builds large dependencies

set -e

echo "================================================"
echo "  Dronoid - Dependency Setup"
echo "================================================"
echo ""

# 1. PX4 Autopilot
if [ ! -d "PX4-Autopilot" ]; then
    echo "📦 Cloning PX4-Autopilot v1.15.4..."
    git clone --depth 1 --branch v1.15.4 https://github.com/PX4/PX4-Autopilot.git
    cd PX4-Autopilot
    git fetch --unshallow
    git checkout v1.15.4
    echo "✓ PX4 cloned"
    cd ..
else
    echo "✓ PX4-Autopilot already exists"
fi

# 2. Micro-XRCE-DDS-Agent
if [ ! -d "Micro-XRCE-DDS-Agent" ]; then
    echo "📦 Cloning Micro-XRCE-DDS-Agent..."
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    echo "✓ DDS Agent built"
    cd ../..
else
    echo "✓ Micro-XRCE-DDS-Agent already exists"
fi

# 3. px4_msgs (ROS2)
if [ ! -d "px4_msgs" ]; then
    echo "📦 Cloning px4_msgs..."
    git clone https://github.com/PX4/px4_msgs.git
    echo "✓ px4_msgs cloned"
else
    echo "✓ px4_msgs already exists"
fi

# 4. Build dronecore ROS2 package
echo "📦 Building dronecore package..."
cd dronecore
colcon build
echo "✓ dronecore built"
cd ..

echo ""
echo "================================================"
echo "  ✓ All dependencies ready!"
echo "================================================"
echo ""
echo "Next steps:"
echo "  1. Install system dependencies (see README.md)"
echo "  2. Run: ./start_drone.sh"
echo "  3. Run: python3 nl_drone.py"
echo ""
