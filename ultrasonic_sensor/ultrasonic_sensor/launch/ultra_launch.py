from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ultrasonic_sensor',
            executable='ultra_node',
            output='screen'
        )
    ])