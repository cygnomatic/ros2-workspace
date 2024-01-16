#!/bin/bash
set -e

git submodule init
git submodule update
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y
