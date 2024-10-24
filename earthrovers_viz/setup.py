from setuptools import find_packages, setup

package_name = 'earthrovers_viz'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['rviz/rover_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nlitz88',
    maintainer_email='nlitz88@gmail.com',
    description='Rviz, rqt, and any other configuration files/templates for visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
