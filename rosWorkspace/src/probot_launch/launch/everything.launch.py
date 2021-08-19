import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('probot_launch'),
        'config',
        'params.yaml'
    )
    print(f"config: {config}")
    return LaunchDescription([
        Node(
            package='vision_localization',
            executable='vision_localization',
            name='vision_localization',
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            parameters=[config],
            arguments=['--ros-args', '--log-level', 'warn'],
            namespace="tracking_camera"
        ),
    ])
