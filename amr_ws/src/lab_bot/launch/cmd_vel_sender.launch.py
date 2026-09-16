from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory("lab_bot")

    script = os.path.join(
        pkg,
        "scripts",
        "cmd_vel_sender.py"
    )

    return LaunchDescription([

        ExecuteProcess(
            cmd=["python3", script],
            output="screen"
        )

    ])