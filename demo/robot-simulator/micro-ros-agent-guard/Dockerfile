ARG ROS_RUNTIME_IMAGE=rovera-robot-edge-ros-control-bridge

# Use the upstream image only as the bootstrap source for micro_ros_setup.
FROM microros/micro-ros-agent:humble AS micro_ros_upstream

FROM ${ROS_RUNTIME_IMAGE}

USER root

ENV DEBIAN_FRONTEND=noninteractive \
    MICROROS_DISABLE_SHM=0

COPY --from=micro_ros_upstream /uros_ws /uros_ws
COPY --from=micro_ros_upstream /micro-ros_entrypoint.sh /micro-ros_entrypoint.sh

# Build the Agent against the exact ROS runtime used by the other Pi
# containers. Copying an older prebuilt executable is not sufficient because
# its graph-manager type support was compiled against older rmw_dds_common
# headers and aborts while decoding ParticipantEntitiesInfo.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && (rosdep init 2>/dev/null || true) \
    && rosdep update --rosdistro=humble \
    && cd /uros_ws \
    && . /opt/ros/humble/setup.sh \
    && . /uros_ws/install/setup.sh \
    && ros2 run micro_ros_setup create_agent_ws.sh \
    && ros2 run micro_ros_setup build_agent.sh \
    && rm -rf /var/lib/apt/lists/* /uros_ws/build /uros_ws/log /uros_ws/src

COPY guarded-entrypoint.sh /opt/rovera/guarded-entrypoint.sh

RUN chmod 0555 /micro-ros_entrypoint.sh /opt/rovera/guarded-entrypoint.sh

ENTRYPOINT ["/opt/rovera/guarded-entrypoint.sh"]
CMD ["--help"]
