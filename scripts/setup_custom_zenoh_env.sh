#!/bin/bash

ZENOH_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ZENOH_C_DIR=$HOME/ws_zenoh_c/zenoh_c
RMW_ZENOH_DIR=$HOME/ws_rmw_zenoh

source /opt/ros/jazzy/setup.bash
source $RMW_ZENOH_DIR/install/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export LD_LIBRARY_PATH=$ZENOH_C_DIR/target/release:$LD_LIBRARY_PATH

echo "Custom Zenoh environment set up successfully"