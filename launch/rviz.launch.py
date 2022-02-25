from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/start.launch.py'])
        )
    )

    ld.add_action(Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', [ThisLaunchFileDir(), '/../rviz/mb_1r2t.rviz']],
        )
    )

    return ld