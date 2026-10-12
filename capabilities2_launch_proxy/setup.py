from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'capabilities2_launch_proxy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'test_nodes'), glob(os.path.join('test_nodes', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalana',
    maintainer_email='kalanaratnayake95@gmail.com',
    description='ROS 2 action server for starting, supervising, and stopping launch files with runtime resource feedback.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capabilities2_launch_proxy = capabilities2_launch_proxy.launch_node:main',
        ],
    },
)
