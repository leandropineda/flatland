# Flatland simulator image, parameterized by ROS 2 distro:
#   docker build -f docker/Dockerfile --build-arg ROS_DISTRO=jazzy -t flatland:jazzy .
# Builds the core packages only (no rviz/GUI); dependencies come from rosdep,
# so the same file works for every supported distro.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

COPY flatland_box2d/ /ws/src/flatland/flatland_box2d/
COPY flatland_msgs/ /ws/src/flatland/flatland_msgs/
COPY flatland_server/ /ws/src/flatland/flatland_server/
COPY flatland_plugins/ /ws/src/flatland/flatland_plugins/

RUN apt-get update && \
    rosdep update --rosdistro "$ROS_DISTRO" && \
    rosdep install --from-paths /ws/src --ignore-src -y && \
    apt-get install -y --no-install-recommends "ros-$ROS_DISTRO-rmw-cyclonedds-cpp" && \
    rm -rf /var/lib/apt/lists/*

RUN cd /ws && . "/opt/ros/$ROS_DISTRO/setup.sh" && \
    colcon build --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace for any command run in the container.
RUN echo ". /opt/ros/$ROS_DISTRO/setup.sh && . /ws/install/setup.sh" > /etc/flatland_env.sh
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
