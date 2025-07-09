"""
UMRT Encoder Manager Node Service

This Encoder Manager Node will start seperate services for each camera.
Currently we have 3 Serial Cameras, each named for their respective section,
Rover Cameras
Arm Cameras

TO-DO
- Convert this file to C++
- Launch Arguments in a seperate file to be pulled into this document
"""

"""
Import
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
import subprocess
import os
import signal
import time
# from umrt_ros_msgs.srv import EncoderControl 
from functools import partial

"""
Encoder Manager Node Class
"""
class EncoderManager(Node):

    def __init__(self):
        super().__init__('encoder_manager')
        self.get_logger().info('Encoder Manager Node Started')

        # Store the Encoder Processes
        self.encoder_processes = {
            'rover_cam0': None,
            'arm_cam0': None,
            'arm_cam1': None
        }

        # Launch Arguments for different camera namespace
        self.encoder_launch_args = {
            'rover_cam0': {            
                'namespace': 'rover',
                'in_raw': '/rover/image_raw',
                'out_ffmpeg': '/rover/ffmpeg',
                'encoding': 'h264_nvenc',
                'qmax': '31',
                'preset': 'p1',
                'tune': 'll',
                'bit_rate': '2000000',
                'gop_size': '15'
            },

            'arm_cam0': {
                'namespace': 'arm0',
                'in_raw': '/arm0/image_raw',
                'out_ffmpeg': '/arm0/ffmpeg',
                'encoding': 'h264_nvenc',
                'qmax': '31',
                'preset': 'p1',
                'tune': 'll',
                'bit_rate': '2000000',
                'gop_size': '15'
            },

            'arm_cam1': {
                'namespace': 'arm1',
                'in_raw': '/arm1/image_raw',
                'out_ffmpeg': '/arm1/ffmpeg',
                'encoding': 'h264_nvenc',
                'qmax': '31',
                'preset': 'p1',
                'tune': 'll',
                'bit_rate': '2000000',
                'gop_size': '15'
            }
        }

        # self.srv = self.create_service(EncoderControl, 'encoder_control', self.encoder_control_callback)

        # Create Service for each camera
        #self.services = {}
        for cam in self.encoder_processes.keys():
            service_name = f'/{cam}/bool'

            # Create service
            self.create_service(
                SetBool,
                service_name,
                partial(self.encoder_control_callback, camera_namespace=cam)
            )
            
            self.get_logger().info(f"Created service: {service_name} for {cam}")

    def encoder_control_callback(self, request, response, camera_namespace: str):
        # camera_namespace = request.camera_namespace
        # command = request.command

        command_bool = request.data # Either True - start or False - stop
        command = "start" if command_bool else "stop"

        # # Check first if the Camera can be encoded
        # if camera_namespace not in self.encoder_processes:
        #     response.success = False
        #     response.message = f"Invalid camera name space: {camera_namespace}"
        #     self.get_logger().error(response.message)
        #     return response

        # Successful Camera namespace
        self.get_logger().info(f'Received command: {command} for {camera_namespace}')

        # Check if command is valid 
        # Start Command - To start the encoder
        if command == "start":
            # Check if encoder for that namespace is already running 
            # If it is running
            if self.encoder_processes[camera_namespace] is not None and self.encoder_processes[camera_namespace].poll is None:
                response.success = False
                response.message = f'Encoder {camera_namespace} is already running.'
                self.get_logger().warn(response.message)
            # Else if its not running 
            else:
                success, msg = self.start_encoder(camera_namespace)
                response.success = success
                response.message = msg
        # Stop Command - To stop the encoder
        elif command == "stop":
            # Check if encoder for that namespace has stopped 
            # If it is stopped
            if self.encoder_processes[camera_namespace] is None or self.encoder_processes[camera_namespace].poll() is not None:
                response.success = False
                response.message = f'Encoder {camera_namespace} is already stopped.'
                self.get_logger().warn(response.message)
            # Else if it not stopped
            else:
                success, msg = self.stop_encoder(camera_namespace)
                response.success = success
                response.message = msg
        # Invalid Command
        else:
            response.success = False
            response.message = f"Unknown command: {command}. Use 'start' or 'stop'"
            self.get_logger().error(response.message)

        return response
    
    """
    Start Encoder Function - It will start the encoder for a given camera namespace
    """
    def start_encoder(self, camera_namespace):
        # Try to start Encoder
        try:
            args = self.encoder_launch_args[camera_namespace]

            # Create the commmand with Launch Arguments
            cmd = ['ros2', 'launch', 'umrt-cam-launch-pkg', 'encode.launch.py']
            for key, value in args.items():
                cmd.append(f'{key}:={value}')

            self.get_logger().info(f"Starting encoder {camera_namespace}, with command: {' '.join(cmd)}")

            process = subprocess.Popen(cmd, preexec_fn=os.setsid)
            self.encoder_processes[camera_namespace] = process
            
            return True, f'Encoder {camera_namespace} started (PID: {process.pid})'
        
        # Failed to start Encoder
        except Exception as e:
            self.get_logger().error(f'Failed to start encoder {camera_namespace}: {e}')
            return False, f'Failed to start encoder {camera_namespace}: {e}'

    """
    Stop Encoder Function - It will stop the encoder for a given camera namespace
    """
    def stop_encoder(self, camera_namespace: str):
        # Get encoder process
        process = self.encoder_processes[camera_namespace]

        # Try if it is running
        if process:
            
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                process.wait(timeout=5)

                # Check if its still running even after the timeout
                if process.poll is None:
                    # Try SIGTERM
                    self.get_logger().warn(f'Encoder {camera_namespace} did not kill, sending SIGTERM')
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.wait(timeout=2)
                    if process.poll is None:
                        # Try SIGKILL
                        self.get_logger().warn(f'Encoder {camera_namespace} did not kill, sending SIGKILL')
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                        process.wait(timeout=2)

                self.encoder_processes[camera_namespace] = None
                return True, f'Encoder {camera_namespace} stopped.'
            # Error that it already stopped or doesnt exist
            except ProcessLookupError:
                self.get_logger().warn(f'Encoder {camera_namespace} process not found')
                self.encoder_processes[camera_namespace] = None
                return True, f'Encoder {camera_namespace} already stopped or process not found'
            # Error with stopping the encoder
            except Exception as e:
                self.get_logger().error(f'Error stopping encoder: {camera_namespace}: {e}')
                return False, f'Error stopping encoder {camera_namespace}: {e}'
        # Else it is already stopped
        return False, f'Encoder {camera_namespace} is not running'
        
"""
Main
"""
def main(args=None):
    rclpy.init(args=args)
    encoder_manager = EncoderManager()

    executor = MultiThreadedExecutor()
    executor.add_node(encoder_manager)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    finally:
        # All subprocess are terminated
        for ns, process in encoder_manager.encoder_processes.items():
            if process and process.poll() is None:
                encoder_manager.get_logger().info(f'Terminating the remaining encoders for {ns}')
                # Terminate
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.wait(timeout=5)
                except Exception as e:
                    encoder_manager.get_logger().error(f'Error during shutdown of {ns} encoder: {e}')
        
        # Kill Encoder Manager Node
        encoder_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
