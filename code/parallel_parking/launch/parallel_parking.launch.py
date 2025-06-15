from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Parallel Parking Node
        Node(
            package='parallel_parking',
            executable='parallel_parking_node',
            name='parallel_parking',
            output='screen',
        )
    ])
