from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_following',
            executable='wall_follow',
            output='screen',
            emulate_tty=True,
            parameters = [
                {'kp': 0.5},
                {'ki': 0.2},
                {'kd': 0.0},
                {'history': 40},
                {'beam_angle': 20},
            ]
        ),
    ])
