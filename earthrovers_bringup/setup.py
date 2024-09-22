from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'earthrovers_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                # Include all launch files.
        ('share/' + package_name, glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nlitz88',
    maintainer_email='nlitz88@gmail.com',
    description='Package containing launch files to bringup the earthrovers ROS stack',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
