from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system',
            executable='hub',
            name='hub_node',
            output='screen'
        ),
        Node(
            package='system',
            executable='sensor',
            name='sensor',  
            output='screen'
        )

    ])
