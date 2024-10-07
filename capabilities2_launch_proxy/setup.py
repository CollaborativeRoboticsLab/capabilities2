from setuptools import find_packages, setup

package_name = 'capabilities2_launch_proxy'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anon',
    maintainer_email='anon',
    description='ros2 launch proxy to support launching capabilities similar to ros1 capabilities',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capabilities_launch_proxy = capabilities2_launch_proxy.capabilities_launch_proxy:main',
        ],
    },
)
