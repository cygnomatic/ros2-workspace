#!/bin/bash

set -e

# Ensure the ROS 2 environment is sourced, replace this path with your actual ROS 2 setup script path if different
source /opt/ros/humble/install/setup.bash || source /opt/ros/humble/setup.bash

if [ $# -ne 1 ]; then
    echo "Usage: ./install_ros_packages.bash <path_to_ros_packages.txt>"
    echo ""
    echo "This script installs ROS 2 packages listed in a text file."
    echo "The text file should contain one package name per line."
    exit 1
fi

PACKAGE_FILE="$1"
if [ ! -f "$PACKAGE_FILE" ]; then
    echo "Package file not found: $PACKAGE_FILE"
    exit 1
fi

install_from_apt() {
    all_packages=""
    while IFS= read -r package || [[ -n "$package" ]]; do
        # Replace underscores with dashes for Debian-based package naming convention
        formatted_package=${package//_/-}
        # Append the formatted package name prefixed with ros-humble- to the all_packages variable
        all_packages+=" ros-humble-$formatted_package"
    done < "$PACKAGE_FILE"

    if [ -z "$all_packages" ]; then
        echo "No packages to install."
        exit 1
    fi
    echo "Installing packages from apt: $all_packages"

    sudo apt-get update
    sudo apt-get install -y $all_packages
}

install_from_source() {
    all_packages=""
    while IFS= read -r package; do
        all_packages+="$package "
    done < "$PACKAGE_FILE"

    if [ -z "$all_packages" ]; then
        echo "No packages to install."
        exit 1
    fi
    echo "Installing packages from source: $all_packages"

    cd /opt/ros/humble || exit 1
    mkdir -p src/cygnomatic_deps

    # Install the python-rosinstall-generator
    sudo apt-get update
    sudo apt-get install python3-rosinstall-generator

    # Fetch the ROS 2 package sources
    export ROS_PACKAGE_PATH=${AMENT_PREFIX_PATH}
    rosinstall_generator --deps --exclude RPP --rosdistro humble $all_packages > ros2.humble.cygnomatic_deps.rosinstall
    
    # Skip if all packages are already installed (if ros2.humble.cygnomatic_deps.rosinstall is empty)
    if [ ! -s ros2.humble.cygnomatic_deps.rosinstall ]; then
        echo "All packages are already installed."
        return
    fi

    vcs import src/cygnomatic_deps < ros2.humble.cygnomatic_deps.rosinstall

    # Install the dependencies
    rosdep update
    rosdep install -y --ignore-src --from-paths src/cygnomatic_deps --rosdistro humble \
    --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" # Skip OpenCV dependencies as it is already included in l4t base image

    # Build and install the packages
    colcon build --merge-install --base-paths src/cygnomatic_deps

    # Clean up
    sudo rm -rf src logs build
}

# Determine the Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
if [ "$UBUNTU_VERSION" == "22.04" ]; then
    install_from_apt
elif [ "$UBUNTU_VERSION" == "20.04" ]; then
    install_from_source # humble packages are not available in the official Ubuntu 20.04 repositories
else
    echo "Unsupported Ubuntu version: $UBUNTU_VERSION"
    exit 1
fi

echo "Done. All ROS packages installed."

# Clean up
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*