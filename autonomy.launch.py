import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# import config_cool.launch_arguments


def generate_launch_description():

    # LIDAR Configuration
    aruco_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ["aruco_detection", "launch", "aruco_detection_launch.py"]
        ),
        # config_cool.launch_arguments.lidar_launch_args.items(),
    )

    return LaunchDescription(
        [
            # aruco_detection,
            Node(package="led_indicator", executable="listener", name="led_indication"),
        ]
    )
