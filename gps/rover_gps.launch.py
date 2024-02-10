import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from launch_ros.actions import Node

def generate_launch_description():

    moving_base = GroupAction(
        actions=[

            IncludeLaunchDescription(

                PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ublox_dgnss'), 'launch',
                'moving_base.launch.py')]),
            )
            
        ]
    )

    moving_rover = GroupAction(
        actions=[

            IncludeLaunchDescription(

                PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ublox_dgnss'), 'launch',
                'moving_rover.launch.py')]),
            )
            
        ]
    )

    heading_republisher = Node(
        package='gps_driver',
        executable='heading_republisher',
    )

    return LaunchDescription([
      moving_base,
      moving_rover,
      heading_republisher,
   ])

