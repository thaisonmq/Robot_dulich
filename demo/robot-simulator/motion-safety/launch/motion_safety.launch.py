from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                parameters=["/opt/rovera/config/twist_mux.yaml"],
                remappings=[("cmd_vel_out", "/cmd_vel_muxed")],
                output="screen",
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                parameters=["/opt/rovera/config/velocity_smoother.yaml"],
                remappings=[("cmd_vel", "/cmd_vel_muxed"), ("cmd_vel_smoothed", "/cmd_vel_smoothed")],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_safety",
                parameters=[{"autostart": True, "node_names": ["velocity_smoother"]}],
                output="screen",
            ),
            Node(
                package="rovera_motion_safety",
                executable="safety_node",
                name="rovera_motion_safety",
                parameters=["/opt/rovera/config/safety.yaml"],
                output="screen",
            ),
        ]
    )
