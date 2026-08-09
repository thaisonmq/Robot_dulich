#!/usr/bin/env bash
set -Eeo pipefail

script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
script_dir="$(cd -- "$(dirname -- "$script_path")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"

if docker ps -q --filter label=rovera.runtime=legacy-yahboom | grep -q .; then
  echo "Guarded Yahboom legacy runtime is already running."
  exit 0
fi

# Never start beside an unlabelled vendor joystick. This avoids duplicate
# /cmd_vel publishers when the original desktop launcher is still active.
while IFS= read -r container_id; do
  [ -n "$container_id" ] || continue
  if docker top "$container_id" -eo args 2>/dev/null | grep -q 'yahboomcar_joy_launch.py'; then
    echo >&2 "Refusing duplicate Yahboom runtime: vendor container $container_id is already active"
    exit 73
  fi
done < <(docker ps -q --filter ancestor=yahboomtechnology/ros-humble:4.1.2)

export DISPLAY="${DISPLAY:-:0}"
xhost + >/dev/null

docker run -d \
  --label rovera.runtime=legacy-yahboom \
  --privileged=true \
  --net=host \
  --ipc=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="ROS_DOMAIN_ID=20" \
  --env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
  --env="FASTRTPS_DEFAULT_PROFILES_FILE=/etc/rovera/micro_ros_fastdds.xml" \
  --env="YAHBOOM_PROCESS_VMEM_KB=0" \
  --env="YAHBOOM_CONTAINER_RSS_KB=921600" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --security-opt apparmor:unconfined \
  -v /dev/input:/dev/input \
  -v /dev/video0:/dev/video0 \
  -v /home/pi/ros2_ws:/root/ros2_ws \
  -v "$project_dir/micro_ros_fastdds.xml:/etc/rovera/micro_ros_fastdds.xml:ro" \
  -v "$project_dir/ros_bridge/yahboom_legacy_guarded_entrypoint.sh:/opt/rovera/yahboom_legacy_guarded_entrypoint.sh:ro" \
  yahboomtechnology/ros-humble:4.1.2 \
  /bin/bash /opt/rovera/yahboom_legacy_guarded_entrypoint.sh
