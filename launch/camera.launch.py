"""
UMRT Camera Launch File
"""

# Imports 
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

"""
Launch File for Camera and Encoders 
"""
def generate_launch_description():

    """
    Parameters
    """
    
    # We can declare some arguments that can be passed on but it's not used here
    rover0_arg = DeclareLaunchArgument('rover0', default_value=os.getenv('CAM1'), description='Device path for rover camera 0')
    arm1_arg = DeclareLaunchArgument('arm0', default_value=os.getenv('ARMCAM0'), description='Device path for arm camera 0')
    arm2_arg = DeclareLaunchArgument('arm1', default_value=os.getenv('ARMCAM1'), description='Device path for arm camera 1')
 
    rover0 = LaunchConfiguration('rover0')
    arm0 = LaunchConfiguration('arm0')
    arm1 = LaunchConfiguration('arm1')

    # Config for all camera parameters
    config_path = os.path.join(
        get_package_share_directory('umrt-serial-cam-ros'),
        'config',
        'cam_params.yaml'
        )

    """
    Nodes
    """
    # Camera 0 - Rover Serial Camera
    rover0_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='rover_cam0',
        namespace = 'rover',
        parameters=[config_path, {'video_device': rover0}
        #     parameters=[config_path, {'video_device': cam1}], <- Use this if you plan to use launch arguments
        #     {
        #     'video_device': '/dev/video8',
        #     'pixel_format': 'YUYV',
        #     'frame_rate': 30.0, # Equivalent to 'frame': 30.0,
        #     'image_size': (640, 480),
        #     '.image_raw.ffmpeg.encoding': 'raw'
        # }
        ]
    )

    # Camera 1 - Arm Camera 0
    arm1_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='arm_cam0',
        namespace='arm0',
        parameters=[config_path, {'video_device': arm0}]
    )

    # Camera 2 - Arm Camera 1
    arm2_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='arm_cam1',
        namespace='arm1',
        parameters=[config_path, {'video_device': arm1}]
    )

    """
    Launch
    """
    cameras = [
        rover0_arg,
        arm1_arg,
        arm2_arg,
        rover0_node,
        arm1_node,
        arm2_node
    ]

    launch_entities = cameras

    return LaunchDescription(launch_entities)