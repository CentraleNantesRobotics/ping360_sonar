from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    pingNode = Node(
        package = "ping360_sonar",
        executable = "ping360_node",
        parameters = [
            {"range_max": 1},
            {"angle_sector": 120}
        ],
        output = "screen"
    )

    ld.add_action(pingNode)

    return ld