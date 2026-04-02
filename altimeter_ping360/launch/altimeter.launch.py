from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    altimeterNode = Node(
        package='altimeter_ping360',
        executable='altimeter_ping360',
        name='altimeter_ping360_node',   # must match the logger name
        output='screen',
        # prefix="xterm -fa 'Monospace' -fs 10 -e gdb start --args",  # uncomment to start debugger
        arguments=[
            "--ros-args",
            "--log-level", 
            "altimeter_ping360_node:=info"  # per-node level
        ]
    )

    ld = LaunchDescription()
    ld.add_action(altimeterNode)
    return ld