from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(get_package_share_directory('project-name'),'launch', 'example.yaml')

    example_node = Node(
            package='project-name',
            executable='project-name',
            name = 'umrt_example_node',
            parameters=[config]
            )

    return LaunchDescription([
        example_node
    ])

