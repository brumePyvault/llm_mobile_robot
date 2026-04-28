from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='llm_mobile_robot',
            executable='llm_control',
            name='llm_control'
        )
    ])
