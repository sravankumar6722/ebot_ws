#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node

def get_xacro_path(package_name, filename):
    """Helper function to locate XACRO file in both source and install spaces"""
    share_dir = get_package_share_directory(package_name)
    # Check install space first
    install_path = os.path.join(share_dir, 'urdf', filename)
    if os.path.exists(install_path):
        return install_path
    # Fallback to source space
    source_path = os.path.join(os.path.dirname(share_dir), '../../src', package_name, 'urdf', filename)
    if os.path.exists(source_path):
        return source_path
    raise FileNotFoundError(f"Could not find {filename} in package {package_name}")

def generate_launch_description():
    # Common launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # LiDAR configuration parameters
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')  # Changed to match URDF
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    # Get path to XACRO file
    xacro_path = get_xacro_path('ebot_control', 'diff_bot.urdf.xacro')
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                FindExecutable(name='xacro'),
                ' ',
                xacro_path
            ])
        }]
    )
    
    # Controller Node
    controller_node = Node(
        package='ebot_control',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # LiDAR Node
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,  # Now matches URDF's laser_frame
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode,
            'use_sim_time': use_sim_time  # Added for time synchronization
        }],
        output='screen'
    )

    # Static TF Publisher for LiDAR
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', frame_id],
        name='static_tf_publisher',
        output='screen'
    )

    return LaunchDescription([
        # Common arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # LiDAR arguments
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        # Nodes
        robot_state_publisher_node,
        controller_node,
        lidar_node,
        static_tf_node
    ])