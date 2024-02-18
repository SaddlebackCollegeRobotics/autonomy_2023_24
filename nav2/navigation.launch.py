import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import Node

def generate_launch_description():

    robot_localization = IncludeLaunchDescription(

        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sam_bot_description'), 'launch',
        'display.launch.py')]),
    )

    navigation_server = GroupAction(
        actions=[

            IncludeLaunchDescription(

                PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch',
                'navigation_launch.py')]),

                launch_arguments = {
                    'params_file' : 'config/nav2_config4.yaml',
                }.items(),
            )
        ]
    )

    drive_kinematics = Node(
        package='drive_kinematics',
        executable='kinematics_converter',
        name='kinematics_converter',
        output='screen',
    )

    return LaunchDescription([
      robot_localization,
      navigation_server,
      drive_kinematics,
    ])

