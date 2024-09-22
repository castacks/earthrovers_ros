from setuptools import find_packages, setup

package_name = 'earthrovers_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nlitz88',
    maintainer_email='nlitz88@gmail.com',
    description='Nodes and utilities for high level navigation and mission control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav = earthrovers_navigation.nav:main',
            'mission_controller = earthrovers_navigation.mission_controller:main',
            'waypoint_manager = earthrovers_navigation.waypoint_manager:main',
            'waypoint_receiver = earthrovers_navigation.waypoint_receiver:main',
        ],
    },
)
