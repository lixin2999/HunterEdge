from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='remote_agent',
            executable='remote_agent',
            name='remote_agent',
            output='screen',
            parameters=[],
        )
    ])