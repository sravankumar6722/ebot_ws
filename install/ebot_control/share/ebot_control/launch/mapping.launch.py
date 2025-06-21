from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # LiDAR Node (SLLidar)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'angle_compensate': True
            }]
        ),
        # SLAM Toolbox Node
    Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',  # Correct executable name
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'params_file': get_package_share_directory('ebot_control') + '/config/slam_params.yaml'
        }]
    ),
        # RViz (Optional)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', get_package_share_directory('nav2_bringup') + '/rviz/nav2_default_view.rviz']
        )
    ])