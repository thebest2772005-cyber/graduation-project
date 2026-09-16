from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB1'
        ),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200'   # A1 = 115200
        ),

        DeclareLaunchArgument(
            'frame_id',
            default_value='laser'
        ),

        # 🚀 RPLidar node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': False,
                'angle_compensate': True,
            }]
        ),

        # 🧭 RViz2 node
        #Node(
         #    package='rviz2',
          #   executable='rviz2',
          #   name='rviz2',
          #  output='screen'
        #)
    ])