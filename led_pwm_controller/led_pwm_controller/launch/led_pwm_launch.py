from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='led_pwm_controller',
            executable='led_pwm_controller',
            output='screen'
        )
    ])

