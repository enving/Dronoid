#!/bin/bash
# Quick Test - Drohne direkt steuern

cd /home/tristan/Repos/Humanoid/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=$PWD/Tools/simulation/gz/models:$PWD/Tools/simulation/gz/worlds

echo "Starting PX4 SITL..."
echo "Nach dem Start: 'commander takeoff' zum Abheben"
make px4_sitl gz_x500
