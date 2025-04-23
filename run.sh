#!/bin/bash

# Navigate to the ROS 2 workspace
cd ~/ros2_ws

# Source the ROS 2 setup script
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build

# Source the workspace setup script
source install/setup.bash

# Run the nodes
ros2 launch predprey esp32_location.py