FROM cygnomatic/ros2ws_devel_base:amd64

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