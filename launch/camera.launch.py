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
    rover0 = LaunchConfiguration('rover0')
    arm0 = LaunchConfiguration('arm0')
    arm1 = LaunchConfiguration('arm1')

    # Config for all camera parameters
    config_path = os.path.join(
        get_package_share_directory('umrt-camera-serial-ros'),
        'config',
        'cam_params.yaml'
        )

    return LaunchDescription([

        # We can declare some arguments that can be passed on but it's not used here
        DeclareLaunchArgument('rover0', default_value=os.getenv('CAM1', 'CAM2'), description='Device path for rover camera 0'),
        DeclareLaunchArgument('arm0', default_value=os.getenv('ARMCAM0'), description='Device path for arm camera 0'),
        DeclareLaunchArgument('arm1', default_value=os.getenv('ARMCAM1'), description='Device path for arm camera 1'),

        # Camera 0 - Rover Serial Camera
        Node(
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
            # remappings=[
            #     ('/image_raw', '/camera_0/image_raw'),
            #     ('/camera_info', '/camera_0/camera_info')
            # ]
        ),

        # Camera 1 - Arm Camera 0
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='arm_cam0',
            namespace='arm0',
            parameters=[config_path, {'video_device': arm0}]
        ),

        # Camera 2 - Arm Camera 1
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='arm_cam1',
            namespace='arm1',
            parameters=[config_path, {'video_device': arm1}]
        ),

        # # USB Camera 1
        # Node(
        #     package='usb_cam',
        #     executable='usb_cam_node_exe',
        #     namespace="usb_cam1",
        #     name='usb_camera_1',
        #     parameters=[config_path, {'video_device': cam1}],
        #     remappings=[
        #         ('/image_raw', '/camera_1/image_raw'),
        #         ('/camera_info', '/camera_1/camera_info')
        #     ]
        # ),
        
    ])