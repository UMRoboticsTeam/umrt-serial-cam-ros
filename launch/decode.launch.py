"""
UMRT Camera Decoder Launch File 
"""

"""
Imports
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

"""
Generate Launch Description
"""
def generate_launch_description():

    # Declare arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description = 'Namespace for camera.'
    )
    in_ffmpeg_arg = DeclareLaunchArgument(
        'in_ffmpeg',
        default_value='/camera/ffmpeg',
        description = 'Input FFmpeg-encoded topic.'
    )
    out_raw_arg = DeclareLaunchArgument(
        'out_raw',
        default_value='/camera/decoded',
        description = 'Output raw image topic.'
    )

    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, # Matches your C++ publisher
        history=HistoryPolicy.KEEP_LAST,
        depth=1, # Depth 1 as per your C++ publisher
        durability=DurabilityPolicy.VOLATILE # Matches your C++ publisher
    )

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    in_ffmpeg = LaunchConfiguration('in_ffmpeg')
    out_raw = LaunchConfiguration('out_raw')

    # Image transport republish node for decoding
    ffmpeg_decoder_node = Node(
        name='image_ffmpeg_to_raw',
        namespace=namespace,
        package='image_transport',
        executable='republish',
        remappings=[
            ('in/ffmpeg', in_ffmpeg),
            ('out', out_raw),
        ],
        arguments=['ffmpeg', 'raw']
    )

    return LaunchDescription([
        namespace_arg,
        in_ffmpeg_arg,
        out_raw_arg,
        ffmpeg_decoder_node
    ])

    # ld = LaunchDescription()
    # ld.add_action(namespace_arg)
    # ld.add_action(in_ffmpeg_arg)
    # ld.add_action(out_raw_arg)
    # ld.add_action(ffmpeg_transport_node)
    # return ld