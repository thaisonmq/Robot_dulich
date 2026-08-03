from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="yahboomcar_ctrl",
                executable="yahboom_joy",
                name="joy_ctrl",
                remappings=[("cmd_vel", "/cmd_vel_joy")],
            ),
            Node(package="joy", executable="joy_node", name="joy_node"),
        ]
    )
