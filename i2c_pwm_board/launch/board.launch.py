from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='i2c_pwm_board',
            executable='controller',
            name="i2c_pwm_controller",
            output='screen',
            prefix='sudo -E'
        )
    ])
