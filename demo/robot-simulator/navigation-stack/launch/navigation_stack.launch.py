import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from speed_profiles import AutoNavigationProfiles


def generate_launch_description() -> LaunchDescription:
    mode = os.getenv("NAVIGATION_MODE", "NAVIGATION").upper()
    speed_profiles = AutoNavigationProfiles.load(
        os.getenv(
            "AUTO_NAVIGATION_SPEED_PROFILES_PATH",
            "/opt/rovera/config/auto_navigation_speed_profiles.yaml",
        )
    )
    behavior_trees = speed_profiles.write_behavior_trees(
        os.getenv(
            "AUTO_NAVIGATION_BT_DIRECTORY",
            "/var/lib/rovera/navigation/behavior_trees",
        )
    )
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
            parameters=["/opt/rovera/config/sensor_time.yaml"],
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
            parameters=["/opt/rovera/config/nav2_params.yaml"],
            output="screen",
        ),
    ]
    # The deployed Yahboom runtime can disappear independently (for example
    # when its non-restarting compatibility container exits). It still leaves
    # /scan, /imu and /odom_raw available through micro-ROS, so Navigation must
    # own the complete normalized odometry and fixed base TF chain instead of
    # depending on that optional process. This also keeps the adapter as the
    # only process that owns its Unix socket.
    if use_vendor_base_runtime:
        common = [
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
                parameters=["/opt/rovera/config/sensor_time.yaml"],
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
                parameters=["/opt/rovera/config/nav2_params.yaml"],
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
            {
                "use_sim_time": False,
                "default_nav_to_pose_bt_xml": str(
                    behavior_trees[speed_profiles.default_mode]
                ),
            },
        ]
        cmd_vel_remaps = [
            # The adapter applies only the selected Auto profile before this
            # reaches twist_mux. Manual Web/joystick sources never pass through
            # this limiter and therefore retain their existing Fast behavior.
            ("cmd_vel", "/cmd_vel_nav_raw"),
            ("cmd_vel_smoothed", "/cmd_vel_nav_raw"),
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
