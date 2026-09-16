from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        # ===== ARG =====
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        # ===== SLAM TOOLBOX =====
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,

                # 🔥 FRAME
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',

                # 🔥 USE FILTERED SCAN
                'scan_topic': '/scan',
                # 'scan_topic': '/scan',

                # 🔥 BASIC TUNING
                'map_update_interval': 2.0,
                'max_laser_range': 12.0,

                # 👉 giúp ổn hơn cho lidar 180°
                'minimum_travel_distance': 0.05,
                'minimum_travel_heading': 0.05,
            }]
        ),
    ])