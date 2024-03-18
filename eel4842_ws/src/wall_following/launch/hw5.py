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
                {'Kp': 0.4},
                {'Ki': 0.01},
                {'Kd': 0.1},
                {'Speed': 70.0},
                {'Angle_Limit', 45.0},
            ]
        ),
    ])
