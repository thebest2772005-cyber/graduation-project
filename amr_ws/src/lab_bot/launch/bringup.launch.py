#!/usr/bin/env python3

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import (
    LaunchConfiguration,
    Command
)

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_share = get_package_share_directory("lab_bot")

    ##########################################################

    urdf_file = os.path.join(
        pkg_share,
        "urdf",
        "lab_robot.urdf.xacro"
    )

    ##########################################################

    lidar_yaml = os.path.join(
        pkg_share,
        "config",
        "rplidar.yaml"
    )

    ##########################################################

    return LaunchDescription([

        ######################################################
        ##
        ## Robot State Publisher
        ##
        ######################################################

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "robot_description": Command([
                        "xacro ",
                        urdf_file
                    ])
                }
            ]
        ),

        ######################################################
        ##
        ## Odom Receiver
        ##
        ######################################################

        Node(

            package="lab_bot",

            executable="odom_receiver.py",

            output="screen",

            parameters=[

                {

                    # "port": "/dev/mcu",
                    "port": "/dev/stm32_odom",
                    "baudrate": 115200,

                    # MCU odometry increments are 0.1 m per metre.
                    "position_scale": 10.0,

                    "frame_id": "odom",

                    "child_frame_id": "base_footprint"

                }

            ]

        ),

        ######################################################
        ##
        ## CMD Sender
        ##
        ######################################################

        Node(

            package="lab_bot",

            executable="cmd_vel_sender.py",

            output="screen"

        ),

        ######################################################
        ##
        ## RPLidar
        ##
        ######################################################

        Node(

            package="rplidar_ros",

            executable="rplidar_composition",

            name="rplidar_node",

            output="screen",

            parameters=[lidar_yaml]

        ),


        Node(
            package="lab_bot",
            executable="scan_filter.py",
            output="screen"
        )

        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="base_to_lidar_tf",
        #     output="screen",
        #     arguments=["0.0", "0.0", "0.1", "0.0", "0.0", "0.0", "base_link", "lidar_link"]
        # ),


        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="footprint_to_base_tf",
        #     output="screen",
        #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "base_footprint", "base_link"]
        # )

    ])
