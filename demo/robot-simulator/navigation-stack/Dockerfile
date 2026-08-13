FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pil \
        python3-yaml \
        ros-humble-nav2-amcl \
        ros-humble-nav2-behaviors \
        ros-humble-nav2-bt-navigator \
        ros-humble-nav2-controller \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-nav2-map-server \
        ros-humble-nav2-planner \
        ros-humble-nav2-regulated-pure-pursuit-controller \
        ros-humble-nav2-smac-planner \
        ros-humble-nav2-waypoint-follower \
        ros-humble-robot-localization \
        ros-humble-robot-state-publisher \
        ros-humble-slam-toolbox \
        ros-humble-std-srvs \
    && rm -rf /var/lib/apt/lists/*

# Keep these small controller/planner additions in their own layer so existing
# robot images reuse the much larger ROS/Nav2 dependency layer above.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-nav2-rotation-shim-controller \
        ros-humble-nav2-theta-star-planner \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src/rovera_navigation_adapter
COPY package.xml setup.py setup.cfg adapter_node.py navigation_core.py sensor_normalizer.py speed_profiles.py ./
COPY resource ./resource
WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select rovera_navigation_adapter

COPY config /opt/rovera/config
COPY launch /opt/rovera/launch
COPY scripts /opt/rovera/scripts
RUN chmod +x /opt/rovera/scripts/*.sh
ENTRYPOINT ["/opt/rovera/scripts/entrypoint.sh"]
CMD ["ros2", "launch", "/opt/rovera/launch/navigation_stack.launch.py"]
