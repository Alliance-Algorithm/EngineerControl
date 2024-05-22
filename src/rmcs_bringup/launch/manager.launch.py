from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    manager = Node(package="manager", executable="manager",
            respawn=True,
            respawn_delay=4.0,)


    return LaunchDescription([ manager])
