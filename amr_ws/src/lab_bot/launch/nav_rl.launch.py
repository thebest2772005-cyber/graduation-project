import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_pkg = get_package_share_directory('lab_bot')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(
        robot_pkg,
        'config',
        'lab_params.yaml'
    )

    rviz_config_file = os.path.join(
        robot_pkg,
        'rviz',
        'robot.rviz'
    )

    map_file = os.path.join(
        robot_pkg,
        'maps',
        'simulation_map_save.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_pkg,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'map': map_file,
            'autostart': 'true',
            'use_composition': 'False'
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        nav2,
        rviz
    ])