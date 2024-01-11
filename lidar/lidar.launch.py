import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import SetRemap

def generate_launch_description():

    lidar = GroupAction(
        actions=[

            IncludeLaunchDescription(

                PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('sllidar_ros2'), 'launch',
                'sllidar_a2m8_launch.py')]),

                launch_arguments = {
                    'scan_mode' : 'Standard',
                    'frame_id' : 'lidar_link',
                }.items(),
            )
            
        ]
    )

    laser_filters = GroupAction(
        actions=[

            IncludeLaunchDescription(

                PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('laser_filters'), 'examples',
                'angular_filter_example.launch.py')]),
            )
            
        ]
    )

    slam = GroupAction(
        actions=[

            SetRemap(src='/scan',dst='/scan_filtered'),

            IncludeLaunchDescription(

                PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch',
                'online_async_launch.py')])
            )
            
        ]
    )

    return LaunchDescription([
      laser_filters,
      lidar,
      slam,
   ])

