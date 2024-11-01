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
            output='screen',
            parameters=[
                {
                "ecg_HighRisk0": [0.0, 70.0],
                "ecg_MidRisk0": [70.0, 85.0],
                "ecg_LowRisk": [85.0, 97.0],
                "ecg_MidRisk1": [97.0, 115.0],
                "ecg_HighRisk1": [115.0, 300.0],
                
                "trm_HighRisk0": [0.0, 31.99],
                "trm_MidRisk0": [32.0, 35.99],
                "trm_LowRisk": [36.0, 37.99],
                "trm_MidRisk1": [38.0, 40.99],
                "trm_HighRisk1": [41.0, 50.0]
                }
            ]
        )

    ])
