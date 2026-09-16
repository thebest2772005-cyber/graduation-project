#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("lab_bot")

    bringup = os.path.join(
        pkg_share,
        "launch",
        "bringup.launch.py"
    )

    slam_yaml = os.path.join(
        pkg_share,
        "config",
        "slam_rl_params.yaml"
    )

    return LaunchDescription([

        ##################################################
        ## Bringup Robot
        ##################################################

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                bringup
            )
        ),

        ##################################################
        ## SLAM Toolbox
        ##################################################

        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_yaml],
            remappings=[
                ("scan", "/scan")
            ]
        )

    ])