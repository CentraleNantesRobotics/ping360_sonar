from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    pingNode = Node(
        package = "ping360_sonar",
        executable = "ping360_node",
        parameters = [
            {"range_max": 5},
            {"angle_sector": 20},
            {"publish_distance": True},
            {"publish_echo": True}
        ],
        output = "screen"
    )

    ld.add_action(pingNode)

    return ld