from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_lite',
            namespace='rtabmap_lite',
            executable='rtabmap_lite_node',
            name='rtabmap_lite_node'
        ),
        Node(
            package='rtabmap_odom_lite',
            namespace='rtabmap_odom_lite',
            executable='rtabmap_odom_lite_node',
            name='rtabmap_odom_lite_node'
        )
    ])