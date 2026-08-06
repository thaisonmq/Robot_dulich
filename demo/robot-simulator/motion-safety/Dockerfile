FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-nav2-velocity-smoother \
        ros-humble-twist-mux \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src/rovera_motion_safety
COPY package.xml setup.py setup.cfg safety_core.py safety_node.py ./
COPY resource ./resource
WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select rovera_motion_safety

COPY config /opt/rovera/config
COPY launch /opt/rovera/launch
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "/opt/rovera/launch/motion_safety.launch.py"]
