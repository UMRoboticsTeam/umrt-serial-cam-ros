"""
UMRT Robot Launch File

TO-DO:
- Work on QoS Profile for all cameras
- Define Launch Arguments for Cameras to keep a persistent way of getting /dev/video
"""

"""
Imports
"""
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy 
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

"""
Generate Launch Description
"""
def generate_launch_description():

    # Declare Launch Arguments - these maybe unnecessary as camera.launch.py can handle this

    # rover cam 0
    # Problem with default value for rover cam - as even empty environment variables still pass through 
    rover0_arg = DeclareLaunchArgument(
        'rover0', default_value=os.getenv('CAM1') or os.getenv('CAM2'), 
        description='Device path for rover camera 0'
    )
    # arm cam 0
    arm0_arg = DeclareLaunchArgument(
        'arm0', default_value=os.getenv('ARMCAM0'),
        description='Device path for arm camera 0'
    )
    # arm cam 1
    arm1_arg = DeclareLaunchArgument(
        'arm1', default_value=os.getenv('ARMCAM1'),
        description='Device path for arm camera 1'
    )

    # Path to this package's launch dir
    launch_dir = os.path.join(
        get_package_share_directory('umrt-serial-cam-ros'), 'launch')
    
    #WIP    
    qos = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT, 
                depth=1, 
                history = HistoryPolicy.KEEP_LAST,
                durability = DurabilityPolicy.VOLATILE)

    return LaunchDescription([

        rover0_arg,
        arm0_arg,
        arm1_arg,

        # Launch all Cameras
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'camera.launch.py')
            ), 
            launch_arguments = {
                'rover0': LaunchConfiguration('rover0'),
                'arm0': LaunchConfiguration('arm0'),
                'arm1': LaunchConfiguration('arm1')
            }.items()
        ),

        # Launch the Encoder Manager Node
        Node(
            package='umrt-serial-cam-ros', 
            executable='encoder_manager_node',
            name='encoder_manager',
            output='screen',
            emulate_tty=True, # Recommended for seeing node output
        )
    
    ])