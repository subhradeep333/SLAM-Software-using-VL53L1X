import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'vl53l0x_lidar_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'pyserial', 'numpy'],
    zip_safe=True,
    maintainer='SLAM Developer',
    maintainer_email='user@todo.todo',
    description='ROS 2 driver and SLAM package for ESP32 + VL53L0X LiDAR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_driver = vl53l0x_lidar_slam.serial_driver_node:main',
            'mock_driver = vl53l0x_lidar_slam.mock_serial_node:main',
        ],
    },
)
