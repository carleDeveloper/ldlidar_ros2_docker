# ROS 2 Jazzy (Ubuntu 24.04) container for the LDROBOT LD19 LiDAR driver.
# Builds the maintained ROS 2 package: ldrobotSensorTeam/ldlidar_stl_ros2
FROM ros:jazzy-ros-base

ARG LDLIDAR_REPO=https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
# Pinned to the latest upstream release tag for reproducible builds.
ARG LDLIDAR_BRANCH=v3.0.3
ARG WORKSPACE=/ros2_ws

SHELL ["/bin/bash", "-c"]

# Build tooling + rviz2 (needed by the viewer_*.launch.py files)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Clone the ROS 2 driver into the workspace src/
WORKDIR ${WORKSPACE}/src
RUN git clone --branch ${LDLIDAR_BRANCH} --depth 1 ${LDLIDAR_REPO}

# Patch: the vendored log_module.cpp uses pthread_mutex_* without including
# <pthread.h>, which fails to compile on Ubuntu 24.04 / GCC 13 (Jazzy).
RUN f=ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp && \
    grep -q '#include <pthread.h>' "$f" || sed -i '1i #include <pthread.h>' "$f"

# Resolve and install package dependencies via rosdep
WORKDIR ${WORKSPACE}
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Auto-source ROS + workspace for interactive shells
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> /root/.bashrc

# Entrypoint sources the overlay, then execs the given command
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
