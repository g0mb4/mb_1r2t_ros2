from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package="mb_1r2t",
        executable="mb_1r2t_node",
        parameters= [
            {"port": "/dev/ttyUSB0"},
            {"frame_id": "lidar"}
        ]
    ))

    return ld