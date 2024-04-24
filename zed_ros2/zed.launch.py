import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import Node
from launch_ros.actions import SetRemap

def generate_launch_description():

    zed_wrapper = GroupAction(
        actions=[

            IncludeLaunchDescription(

                PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('zed_wrapper'), 'launch',
                'zed_camera.launch.py')]),

                launch_arguments = {
                    'camera_model' : 'zed2i',
                    'publish_tf' : 'false',
                    'publish_map_tf' : 'false',
                    'publish_imu_tf' : 'false',
                    'config_path' : 'config/zed_config.yaml',

                }.items(),
            )
            
        ]
    )

    zed_odom_republisher = Node(
        package='zed_driver',
        executable='republisher',
        name='odom_republisher',
        output='screen',
    )
    

    return LaunchDescription([
      zed_wrapper,
      zed_odom_republisher,
   ])

