from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='i2cpwm_board',
            executable='i2cpwm_executable',
            name="i2cpwm_controller",
            output='screen',
            prefix='sudo -E'
        )
    ])