FROM althack/ros2:humble-full

RUN apt-get update && apt-get install -y --no-install-recommends \
  clangd \
  && rm -rf /var/lib/apt/lists/*