import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suhaib',
    maintainer_email='suhaib.aburaidah@gmail.com',
    description='Autonomous exploration package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exploration_node = amr_exploration.exploration_node:main',
            'exploration_node_filtered = amr_exploration.exploration_node_filtered:main',
        ],
    },
)
