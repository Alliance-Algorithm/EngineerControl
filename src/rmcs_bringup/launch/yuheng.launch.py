from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    visual_exchange = Node(package="visual_exchange", executable="visual_exchange")

    yuheng_arm = ExecuteProcess(
        cmd=[
            get_package_share_directory("manager")
            + "/YuHengArm/YuHengArm/YuHengArm.x86_64"
        ],
        output="screen"
    )

    return LaunchDescription([visual_exchange, yuheng_arm])
