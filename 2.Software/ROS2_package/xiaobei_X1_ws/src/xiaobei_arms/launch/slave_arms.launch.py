from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xiaobei_arms',
            executable='slave_arm_subscriber',
            name='slave_arms',
            output='screen'
        )
    ])
