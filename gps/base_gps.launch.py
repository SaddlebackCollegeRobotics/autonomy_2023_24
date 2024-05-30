import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from launch_ros.actions import Node


def generate_launch_description():

    gps_rtcm_publisher = Node(
        package="gps_driver",
        executable="rtcm_publisher",
    )

    return LaunchDescription(
        [
            gps_rtcm_publisher,
        ]
    )
