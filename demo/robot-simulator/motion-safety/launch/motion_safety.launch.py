from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
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
            # Fast DDS can occasionally time out the lifecycle manager's
            # configure response while the smoother has already reached the
            # inactive state. The manager then never sends activate. Run one
            # delayed, bounded reconciliation so the final command pipeline
            # cannot remain silently disconnected after container startup.
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "/opt/rovera/scripts/ensure_velocity_smoother_active.py"
                        ],
                        output="screen",
                    )
                ],
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
