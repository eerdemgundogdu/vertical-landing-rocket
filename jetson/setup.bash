#!/bin/bash
# Jetson Nano environment setup script

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace (if built)
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# Set environment variables
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "ROS2 Humble environment configured"
echo "  ROS_DISTRO: $ROS_DISTRO"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
