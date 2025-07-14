import os
from glob import glob
from setuptools import setup

# Seperate the package name and python name 
package_name = 'umrt-serial-cam-ros'
python_name = 'umrt_serial_cam_ros'

# Read version from a file
with open(os.path.join(os.path.dirname(__file__), 'version')) as f:
    version = f.read().strip()

setup(
    name=package_name,
    version=version,
    packages=[python_name],
    data_files=[
        # Install launch files
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/config', glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edcel',
    maintainer_email='158010272+edcela@users.noreply.github.com',
    description='UMRT ROS2 Package for Serial Cameras',
    license='MPL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_manager_node = umrt_serial_cam_ros.encoder_manager_node:main',
        ]
    },
)