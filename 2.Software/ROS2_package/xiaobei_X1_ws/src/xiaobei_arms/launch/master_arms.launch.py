from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xiaobei_arms',
            executable='master_arm_publisher',
            name='master_arms',
            output='screen'
        )
    ])
