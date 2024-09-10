#!/bin/bash

set -e

ZENOH_C_DIR=$HOME/ws_zenoh_c/zenoh-c

# Create zenoh-c workspace
mkdir -p $HOME/ws_zenoh_c
cd $HOME/ws_zenoh_c

# Clone zenoh-c and checkout the specific commit
if [ ! -d "zenoh-c" ]; then
    git clone https://github.com/eclipse-zenoh/zenoh-c.git
fi
cd zenoh-c
git fetch --all
git checkout 134dbfa06ca212def5fb51dd8e816734dfd8dff6

# Function to update a specific package
update_package() {
    local package=$1
    sed -i "/^$package = {/,/}/{
        s|git = \"https://github.com/eclipse-zenoh/zenoh.git\"|git = \"https://github.com/fj-blanco/zenoh.git\"|g
        s|branch = \"main\"|branch = \"pq\"|g
    }" Cargo.toml
}

# Set Cargo flags
export ZENOHC_CARGO_FLAGS="--no-default-features --features=zenoh/shared-memory,zenoh/transport_compression,zenoh/transport_tcp,zenoh/transport_tls"

# Build zenoh-c
cargo build --release -j 4

# Create CMake config file
cat << EOF > zenohcConfig.cmake
set(ZENOHC_INCLUDE_DIR "${ZENOH_C_DIR}/include")
set(ZENOHC_LIBRARY "${ZENOH_C_DIR}/target/release/libzenohc.so")

add_library(zenohc::lib SHARED IMPORTED)
set_target_properties(zenohc::lib PROPERTIES
  IMPORTED_LOCATION "\${ZENOHC_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "\${ZENOHC_INCLUDE_DIR}"
)
EOF

echo "Custom zenoh-c built successfully at $ZENOH_C_DIR"