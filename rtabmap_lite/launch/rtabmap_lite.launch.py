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
            namespace='rtabmap_lite',
            executable='rtabmap_odom_lite_node',
            name='rtabmap_odom_lite_node'
            #remappings=[
                #("rgb/image", "/camera/color/image_raw"),
                #("depth/image", "/camera/aligned_depth_to_color/image_raw"),
                #("rgb/camera_info", "/camera/color/camera_info")
                #("rgbd_image", LaunchConfiguration('rgbd_topic_relay')),
                #("odom", LaunchConfiguration('odom_topic')),
                #("imu", LaunchConfiguration('imu_topic'))],
        )
    ])