import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # LiDAR Node (from your driver)
    lidar_node = Node(
        package='sllidar_ros2',  # Replace with your package name
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'standard'  # Adjust based on your LiDAR model
        }],
        output='screen'
    )

    # Nav2 Bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(
                get_package_share_directory('ebot_control'),
                'config',
                'nav2_params.yaml'
            )
        }.items()
    )

    # Optional: RViz
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        lidar_node,
        nav2_launch,
        rviz_node
    ])