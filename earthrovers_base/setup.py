from setuptools import find_packages, setup

package_name = 'earthrovers_base'

setup(
    name=package_name,
    version='0.0.0',
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
    description='Core nodes for controlling and interacting with the FrodoBots Earth Rovers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base = earthrovers_base.base:main'
        ],
    },
)