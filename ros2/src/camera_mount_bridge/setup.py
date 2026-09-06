from setuptools import find_packages, setup

package_name = 'camera_mount_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/servos.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Desmond',
    maintainer_email='desmond@eastworlds.io',
    description='ROS2 bridge node for the 2-DOF (yaw/pitch) Dynamixel camera mount.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = camera_mount_bridge.bridge_node:main',
        ],
    },
)
