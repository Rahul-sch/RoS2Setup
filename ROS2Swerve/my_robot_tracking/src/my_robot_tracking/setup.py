from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_tracking'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_name + '.scripts'],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 package for robot tracking system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = my_robot_tracking.camera_node:main',
            'tracking_node = my_robot_tracking.tracking_node:main',
            'hardware_node = my_robot_tracking.hardware_node:main',
            'teleop_node = my_robot_tracking.teleop_node:main',
            'object_selector = my_robot_tracking.scripts.object_selector:main',
            'lidar_guard = my_robot_tracking.lidar_guard_node:main',
        ],
    },
)

