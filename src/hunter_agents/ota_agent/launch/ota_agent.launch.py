from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ota_agent',
            executable='ota_agent_node',
            name='ota_agent',
            output='screen',
            parameters=[],
        )
    ])