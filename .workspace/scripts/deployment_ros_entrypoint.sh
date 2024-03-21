#!/bin/bash
#set -e

if [ -f ${ROS_ROOT}/install/setup.bash ]; then source ${ROS_ROOT}/install/setup.bash; fi
if [ -f ${ROS_ROOT}/setup.bash ]; then source ${ROS_ROOT}/setup.bash; fi
if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi

exec "$@"