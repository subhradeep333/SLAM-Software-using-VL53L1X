from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 VL53L0X LiDAR (e.g. /dev/ttyUSB0 or /dev/ttyACM0)'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    driver_node = Node(
        package='vl53l0x_lidar_slam',
        executable='serial_driver',
        name='vl53l0x_serial_driver',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': 'laser_frame',
            'base_frame': 'base_link',
            'publish_tf': True,
            'laser_x': 0.1,
            'laser_y': 0.0,
            'laser_z': 0.1,
            'angle_min_deg': -90.0,
            'angle_max_deg': 90.0,
            'range_min': 0.03,
            'range_max': 2.0
        }]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        driver_node
    ])

