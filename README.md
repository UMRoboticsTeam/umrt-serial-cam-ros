# UMRT Serial Camera ROS Package
Serial Camera ROS project repository for the University of Manitoba Robotics Team.

### Requirements

This package requires consistent symlink for cameras, the following is an example of UDEV rules to ensure consistent symlinks:
```
# Name camera by serial number or product
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", ATTRS{idVendor}=="idVendor", ATTRS{idProduct}=="idProduct", ATTR{name}=="name", ATTR{index}=="index", MODE="0664", GROUP="video", SYMLINK+="cameras/cam1"
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", ATTRS{idVendor}=="idVendor", ATTRS{idProduct}=="idProduct", ATTR{name}=="name", ATTR{index}=="index" ATTRS{devpath}=="devpath", MODE="0664", GROUP="video", SYMLINK+="cameras/cam2", RUN+="/usr/local/bin/set_cam_fps.sh /dev/cameras/cam2 (fps)"
```

The first symlink is just a simple renaming for a device, based off attributes. 

The second symlink includes an additional attributes, devpath, as some serial cameras you use could be exactly the same and the only way to differentiate is through the devpath. The other thing included in the second symlink is running a script to change the Frames Per Second (fps) for that specific camera.

Note that you must reload these rules after they have been written:
```
    sudo udevadm control --reload
    sudo udevadm trigger 
```

To access these attributes for any given serial camera use the following:
```
   udevadm info --name=/dev/video* --attribute-walk

   [*] Is used to indicate which video input to check for 
```

The follow lists the pixel format, resolution, and FPS for that specific video device. This is important when trying to set parameters for the cameras. 
```
   v4l2-ctl --device=/dev/video4 --list-formats-ext
```

To access the symlinks for cameras into a Docker container the following is required in the Docker run command, but is recomended to be placed in a bash script:

```
CAM1_SYMLINK="/dev/cameras/cam1" # Is the name of the symlink from udev rules  
CAM1=""

# Check and resolve camera symlinks if they exist
if [ -e "$CAM1_SYMLINK" ]; then
  CAM1=$(readlink -f "$CAM1_SYMLINK")
  CAM1_ARG="--device=$CAM1"
else
  echo "Warning CAM1: Webcam C170 not found. Continuing without it."
  CAM1_ARG

docker run \ 
   $CAM1_ARG \ # Will be the actual device
   -e CAM1="$CAM1" \ # Will include it a environment variable 
```

For the encoders to encode using a NVIDIA GPU, it is assumed you have a NVIDIA GPU, the device and Docker container must have "h264_nvenc" avaialble:
First, have to check if libnvidia-encode.so is available on host device:
```
   ls -l /usr/lib/x86_64-linux-gnu/libnvidia-encode.so.1

   You can then check if GPU supports h264_nvenc

   ffmpeg -encoders | grep nvenc
```

The Docker container must have the following to use the GPU:
```
  --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
```

---
# umrt-serial-cam-ros

This library/executable/umrt-serial-cam-ros implements serial camera video streaming for the University of Manitoba Robotics Team's rover and robotic arm.

[See the documentation](https://umroboticsteam.github.io/umrt-serial-cam-ros/)