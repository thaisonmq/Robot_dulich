FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    HOME=/tmp/rovera \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/opt/rovera \
    ROS_LOG_DIR=/tmp/rovera/ros-logs

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-sensor-msgs \
        ros-humble-twist-mux \
    && rm -rf /var/lib/apt/lists/*

COPY simulator/__init__.py /opt/rovera/simulator/__init__.py
COPY simulator/control_protocol.py /opt/rovera/simulator/control_protocol.py
COPY ros_bridge /opt/rovera/ros_bridge
RUN chmod 0755 /opt/rovera/ros_bridge/entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/opt/rovera/ros_bridge/entrypoint.sh"]
