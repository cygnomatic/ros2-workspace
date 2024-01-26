FROM --platform=linux/arm64 registry.cn-beijing.aliyuncs.com/cygnomatic/jetson_base:fb32fdd8

# Install Python dependencies
COPY .devcontainer/python_requirements.txt /tmp/python_requirements.txt
RUN pip install --no-cache-dir -r /tmp/python_requirements.txt && \
    rm /tmp/python_requirements.txt

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