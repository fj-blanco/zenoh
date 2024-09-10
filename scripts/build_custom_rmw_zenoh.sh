#!/bin/bash

set -e

ZENOH_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ZENOH_C_DIR=$HOME/ws_zenoh_c/zenoh-c
RMW_ZENOH_DIR=$HOME/ws_rmw_zenoh

# Create rmw_zenoh workspace
mkdir -p $RMW_ZENOH_DIR/src
cd $RMW_ZENOH_DIR/src

# Clone rmw_zenoh repository
if [ ! -d "rmw_zenoh" ]; then
    git clone https://github.com/ros2/rmw_zenoh.git
fi

# Modify zenoh_c_vendor CMakeLists.txt
cat << EOF > rmw_zenoh/zenoh_c_vendor/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(zenoh_c_vendor)

find_package(ament_cmake REQUIRED)

set(CUSTOM_ZENOH_C_DIR $ZENOH_C_DIR)
set(CMAKE_PREFIX_PATH \${CUSTOM_ZENOH_C_DIR}:\${CMAKE_PREFIX_PATH})

find_package(zenohc REQUIRED PATHS \${CUSTOM_ZENOH_C_DIR})

ament_export_libraries(zenohc::lib)
ament_export_include_directories(\${ZENOHC_INCLUDE_DIR})

ament_package()
EOF

cd $RMW_ZENOH_DIR

# Install dependencies
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# Source ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Build rmw_zenoh
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$ZENOH_C_DIR

echo "Custom rmw_zenoh has been built at $RMW_ZENOH_DIR"