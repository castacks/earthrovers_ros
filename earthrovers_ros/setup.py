import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'earthrovers_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        ('share/' + package_name, glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name, glob(os.path.join('config/camera_calibration', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nlitz88',
    maintainer_email='nlitz88@gmail.com',
    description='ROS Wrapper for the FrodoBots Earth Rover SDK',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base = earthrovers_ros.base:main',
            'camera = earthrovers_ros.camera:main',
            'nav = earthrovers_ros.nav:main',
            'mock_gps = earthrovers_ros.mock_gps:main',
            'mission_control = earthrovers_ros.mission_control:main',
            'waypoint_receiver = earthrovers_ros.waypoint_receiver:main',
        ],
    },
)
