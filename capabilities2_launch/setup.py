from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'capabilities2_launch'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch_server'), glob(os.path.join('launch_server', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalana',
    maintainer_email='kalanaratnayake95@gmail.com',
    description='A ROS 2 package to manage starting and stopping ROS 2 launch files through services.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capabilities2_launch = capabilities2_launch.capabilities2_launch:main',
        ],
    },
)
