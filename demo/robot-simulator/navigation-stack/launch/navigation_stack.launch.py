import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mode = os.getenv("NAVIGATION_MODE", "NAVIGATION").upper()
    use_vendor_base_runtime = os.getenv(
        "ROVERA_USE_VENDOR_BASE_RUNTIME", "0"
    ).lower() in {"1", "true", "yes"}
    robot_description = Path("/opt/rovera/config/robot.urdf").read_text()
    owned_base_runtime = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            output="screen",
        ),
        Node(
            package="rovera_navigation_adapter",
            executable="sensor_normalizer",
            name="rovera_sensor_normalizer",
            output="screen",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=["/opt/rovera/config/ekf.yaml"],
            output="screen",
        ),
        Node(
            package="rovera_navigation_adapter",
            executable="adapter_node",
            name="rovera_navigation_adapter",
            output="screen",
        ),
    ]
    # The Yahboom compatibility container can preserve its existing EKF,
    # robot_state_publisher and static transforms. Reuse them instead of
    # creating duplicate TF authorities. The Rovera adapter itself is always
    # isolated and remains the only process that owns its Unix socket.
    if use_vendor_base_runtime:
        common = [
            Node(
                package="rovera_navigation_adapter",
                executable="adapter_node",
                name="rovera_navigation_adapter",
                output="screen",
            ),
            # Yahboom publishes the base and IMU transforms, but its standalone
            # bringup omits the LiDAR transform that its own mapping launch
            # normally supplies. Reuse the calibrated vendor offset without
            # starting a second robot_state_publisher or EKF authority.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="rovera_base_to_laser",
                arguments=[
                    "--x", "-0.0046412",
                    "--y", "0",
                    "--z", "0.094079",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "base_link",
                    "--child-frame-id", "laser_frame",
                ],
                output="screen",
            ),
        ]
    else:
        common = owned_base_runtime
    if mode == "MAPPING":
        # Keep SLAM Toolbox as a separately respawnable child. The navigation
        # adapter restarts only this child between map sessions, which clears
        # the pose graph without restarting the Agent, motion safety or the
        # whole Docker stack.
        runtime = Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[
                "/opt/rovera/config/slam_toolbox.yaml",
                {"use_sim_time": False},
            ],
            output="screen",
            respawn=True,
            respawn_delay=1.0,
        )
    else:
        parameters = [
            "/opt/rovera/config/nav2_params.yaml",
            {"use_sim_time": False},
        ]
        cmd_vel_remaps = [
            ("cmd_vel", "/cmd_vel_nav"),
            ("cmd_vel_smoothed", "/cmd_vel_nav"),
        ]
        runtime = [
            Node(package="nav2_map_server", executable="map_server", name="map_server", parameters=[*parameters, {"yaml_filename": os.getenv("NAV2_MAP_YAML", "/opt/rovera/config/bootstrap-map.yaml")}], output="screen"),
            Node(package="nav2_amcl", executable="amcl", name="amcl", parameters=parameters, output="screen"),
            Node(package="nav2_planner", executable="planner_server", name="planner_server", parameters=parameters, output="screen"),
            Node(package="nav2_controller", executable="controller_server", name="controller_server", parameters=parameters, remappings=cmd_vel_remaps, output="screen"),
            Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", parameters=parameters, remappings=cmd_vel_remaps, output="screen"),
            Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", parameters=parameters, output="screen"),
            Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower", parameters=parameters, output="screen"),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[{
                    "autostart": True,
                    "node_names": [
                        "map_server", "amcl", "planner_server", "controller_server",
                        "behavior_server", "bt_navigator", "waypoint_follower",
                    ],
                }],
                output="screen",
            ),
        ]
    return LaunchDescription([*common, *runtime] if isinstance(runtime, list) else [*common, runtime])
