import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# import config_cool.launch_arguments

def generate_launch_description():

    # LIDAR Configuration
    lidar_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'), 'launch'),
            '/sllidar_a2m8_launch.py']),

            # config_cool.launch_arguments.lidar_launch_args.items(),
    )

    return LaunchDescription([
      lidar_nodes,
   ])
