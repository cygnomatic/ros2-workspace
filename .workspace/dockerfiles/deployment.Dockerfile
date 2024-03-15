FROM amd64/ros:humble

# Setup auto-completion for ros2
RUN apt-get update && apt-get install -y git-core bash-completion && \
    echo -e "\nif [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /ros_entrypoint.sh && \
    echo -e "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /ros_entrypoint.sh && \
    echo -e "\nsource /ros_entrypoint.sh" >> /root/.bashrc && \
    rm -rf /var/lib/apt/lists/*

# Setup byobu
RUN apt-get update && apt-get install -y byobu && \
    rm -rf /var/lib/apt/lists/*
CMD byobu new -n main

# Remove entrypoint to prevent duplicate sourcing
ENTRYPOINT []

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
RUN pip3 install --no-cache-dir -r /tmp/pip_requirements.txt && \
    rm /tmp/pip_requirements.txt

###########################################
# Deployment
###########################################

##### CACHE VALID UNTIL HERE #####

WORKDIR /workspace
COPY . /workspace

# # Install ROS dependencies
# RUN source ${ROS_ROOT}/install/setup.bash && \
#     apt-get update && \
#     rosdep update --rosdistro=$ROS_DISTRO && \
#     rosdep install -y \
#         --from-paths src \
#         --ignore-src \
#         --rosdistro=$ROS_DISTRO \
#         --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"

RUN source ${ROS_ROOT}/install/setup.bash && \
    colcon build --merge-install --base-paths src