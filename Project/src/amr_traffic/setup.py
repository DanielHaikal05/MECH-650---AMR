import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_traffic'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models', 'traffic_robot_red'),
            glob('models/traffic_robot_red/*')),
        (os.path.join('share', package_name, 'models', 'traffic_robot_yellow'),
            glob('models/traffic_robot_yellow/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anthony Boulos',
    maintainer_email='anthony.boulos@example.invalid',
    description='Traffic robots (dynamic obstacles) for the AMR town simulation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'random_walker_node = amr_traffic.random_walker_node:main',
        ],
    },
)
