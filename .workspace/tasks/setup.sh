#!/bin/bash
set -e

git submodule init
git submodule update

sudo apt-get update

rosdep update --rosdistro=$ROS_DISTRO

if [ "${IS_JETSON_CONTAINER}" = "1" ]; then
    echo "OpenCV is already installed on Jetson containers. Skipping installation of OpenCV dependencies."
    rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO \
    --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" 
else
    rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO
fi

# Add upstream remote if not exist
git remote get-url upstream &>/dev/null
if [ $? -ne 0 ]; then
    git remote add upstream git@codeup.aliyun.com:cygnomatic/dev_ws/vscode_ros2_workspace.git
fi
git fetch upstream

# Check if there is any update from upstream
BEHIND_COUNT=$(git rev-list --count main..upstream/main)
if [ $BEHIND_COUNT -gt 0 ]; then
    echo -e "\e[1mYour main branch is behind upstream/main by $BEHIND_COUNT commits.\e[0m"
    echo -e "\e[1mPlease merge the changes by running task \e[38;5;208m\u001b[4msync upstream\e[0m\e[1m.\e[0m"
else
    echo "The main branch is up to date with upstream/main."
fi