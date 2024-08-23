import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('corsano_ros'),
        'config',
        'params.yaml'
        )

    corsano_node=Node(
        package = 'corsano_ros',
        name = 'corsano_ros',
        executable = 'corsano_wrapper.py',
        parameters = [config],
    )
    ld.add_action(corsano_node)

    return ld
