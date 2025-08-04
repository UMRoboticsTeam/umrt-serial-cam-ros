"""
UMRT Camera Encoder Launch File 
"""

"""
Imports
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


"""
Generate Launch Description
"""
def generate_launch_description():
    
    # Declare arguments
    in_raw_topic = DeclareLaunchArgument(
        'in_raw',
        default_value='/camera_0/image_raw',
        description='Input raw image topic.'
    )
    out_ffmpeg_topic = DeclareLaunchArgument(
        'out_ffmpeg',
        default_value='/camera_0/ffmpeg',
        description='Output FFmpeg-encoded topic.'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/cam',
        description='Namespace describer for cameras.'
    )
    encoding_arg = DeclareLaunchArgument(
        'encoding',
        default_value='h264_nvenc', # Explicitly setting h264_nvenc here
        description='FFmpeg encoding codec.'
    )
    qmax_arg = DeclareLaunchArgument(
        'qmax',
        default_value='31',
        description='Max quantization rate for FFmpeg.'
    )
    preset_arg = DeclareLaunchArgument(
        'preset',
        default_value='p1',
        description='Preset for FFmpeg encoder (e.g., p1, p7 for nvenc).'
    )
    tune_arg = DeclareLaunchArgument(
        'tune',
        default_value='ll',
        description='Tune for FFmpeg encoder (e.g., ll for low latency).'
    )
    bit_rate_arg = DeclareLaunchArgument(
        'bit_rate',
        default_value='2000000',
        description='Bit rate for FFmpeg encoder.'
    )
    gop_size_arg = DeclareLaunchArgument(
        'gop_size',
        default_value='15',
        description='Group of Pictures size (keyframes interval).'
    )
    
    # Launch configurations
    in_raw = LaunchConfiguration('in_raw')
    out_ffmpeg = LaunchConfiguration('out_ffmpeg')
    namespace = LaunchConfiguration('namespace')
    encoding = LaunchConfiguration('encoding')
    qmax = LaunchConfiguration('qmax')
    preset = LaunchConfiguration('preset')
    tune = LaunchConfiguration('tune')
    bit_rate = LaunchConfiguration('bit_rate')
    gop_size = LaunchConfiguration('gop_size')

    # Image transport republish node for encoding
    ffmpeg_encoder_node = Node(
        name='image_raw_to_ffmpeg',
        namespace=namespace,
        package='image_transport',
        executable='republish',
        remappings=[
            ('in', in_raw),
            ('out/ffmpeg', out_ffmpeg),
        ],
        arguments=['raw', 'ffmpeg'], # Input is raw, output is ffmpeg
        parameters=[{
            '.out.ffmpeg.encoding': encoding,
            '.out.ffmpeg.qmax': qmax,
            '.out.ffmpeg.preset': preset,
            '.out.ffmpeg.tune': tune,
            '.out.ffmpeg.bit_rate': bit_rate,
            '.out.ffmpeg.gop_size': gop_size,
        }]
    )

    return LaunchDescription([
        in_raw_topic,
        out_ffmpeg_topic,
        namespace_arg,
        encoding_arg,
        qmax_arg,
        preset_arg,
        tune_arg,
        bit_rate_arg,
        gop_size_arg,
        ffmpeg_encoder_node
    ])