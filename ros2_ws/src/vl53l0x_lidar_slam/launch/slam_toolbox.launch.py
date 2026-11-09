import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('vl53l0x_lidar_slam')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'vl53l0x_rviz.rviz')

    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Set to true to use synthetic simulated scan publisher instead of physical serial hardware.'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='ESP32 serial port'
    )

    # Real driver
    real_driver_node = Node(
        package='vl53l0x_lidar_slam',
        executable='serial_driver',
        name='vl53l0x_serial_driver',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_mock')),
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': 115200,
            'frame_id': 'laser_frame',
            'base_frame': 'base_link',
            'publish_tf': True,
        }]
    )

    # Mock driver
    mock_driver_node = Node(
        package='vl53l0x_lidar_slam',
        executable='mock_driver',
        name='mock_vl53l0x_lidar',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mock')),
    )

    # SLAM Toolbox Node (Async)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
        remappings=[('/scan', '/scan')]
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        use_mock_arg,
        port_arg,
        real_driver_node,
        mock_driver_node,
        slam_toolbox_node,
        rviz_node
    ])
