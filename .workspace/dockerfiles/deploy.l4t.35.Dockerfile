FROM --platform=linux/arm64 dustynv/ros:humble-ros-base-l4t-r35.4.1 as base

# Reset RMW_IMPLEMENTATION to the default FastRTPS
ENV RMW_IMPLEMENTATION=

###########################################
# Install requirements & dependencies
###########################################

# Install ROS packages
COPY .workspace/requirements/ros_packages.txt /tmp/ros_packages.txt
COPY .workspace/scripts/install_ros_packages.bash /tmp/install_ros_packages.bash
RUN bash /tmp/install_ros_packages.bash /tmp/ros_packages.txt && \
    rm /tmp/ros_packages.txt /tmp/install_ros_packages.bash

# Install apt packages
COPY .workspace/requirements/apt_packages.txt /tmp/apt_packages.txt
RUN apt-get update && \
    xargs -a /tmp/apt_packages.txt apt-get install -y --no-install-recommends && \
    rm /tmp/apt_packages.txt && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY .workspace/requirements/pip_requirements.txt /tmp/pip_requirements.txt
RUN pip install --no-cache-dir -r /tmp/pip_requirements.txt && \
    rm /tmp/pip_requirements.txt

###########################################
# Deployment
###########################################

##### CACHE VALID UNTIL HERE #####

WORKDIR /workspace
COPY src /workspace/src

# # Install ROS dependencies
# RUN source ${ROS_ROOT}/install/setup.bash && \
#     apt-get update && \
#     rosdep update --rosdistro=$ROS_DISTRO && \
#     rosdep install -y \
#         --from-paths src \
#         --ignore-src \
#         --rosdistro=$ROS_DISTRO \
#         --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"

RUN source ${ROS_ROOT}/setup.bash && \
    colcon build --merge-install --base-paths src

COPY .workspace/scripts/deployment_ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Change CMD to your ros2 bringup command
CMD ["bash"]