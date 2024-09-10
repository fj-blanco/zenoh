#!/bin/bash

# build_rmw_zenoh.sh

set -e

# Create workspace
mkdir -p ~/ws_rmw_zenoh/src && cd ~/ws_rmw_zenoh/src

# Clone rmw_zenoh repository
if [ ! -d "rmw_zenoh" ]; then
    git clone https://github.com/ros2/rmw_zenoh.git
else
    echo "rmw_zenoh directory already exists, skipping clone"
fi

cd ~/ws_rmw_zenoh

# Install dependencies
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# Source ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Build rmw_zenoh
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "rmw_zenoh has been built."
echo "To use rmw_zenoh, please run the following commands in each new terminal:"
echo "source ~/ws_rmw_zenoh/install/setup.bash"
echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp"
echo ""
echo "To start the Zenoh router, run:"
echo "ros2 run rmw_zenoh_cpp rmw_zenohd"
echo ""
echo "Before running ROS 2 nodes, you may need to terminate the ROS 2 daemon:"
echo "pkill -9 -f ros && ros2 daemon stop"