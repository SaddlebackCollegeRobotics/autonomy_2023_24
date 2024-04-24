import launch
import launch_ros.actions


gps_fix = launch.actions.ExecuteProcess(
    cmd=['ros2', 'bag', 'play', '-l', 'rosbag_gps_fix'],
    output='screen')

gps_fix_republisher = launch_ros.actions.Node(
    package='republisher',
    executable='gps_fix_republisher',
    name='gps_fix_republisher',
    output='screen'
)

gps_heading = launch.actions.ExecuteProcess(
    cmd=['ros2', 'bag', 'play', '-l', 'rosbag_gps_heading'],
    output='screen')

lidar = launch.actions.ExecuteProcess(
    cmd=['ros2', 'bag', 'play', '-l', 'rosbag_laser_filtered'],
    output='screen')

zed_odometry = launch.actions.ExecuteProcess(
    cmd=['ros2', 'bag', 'play', '-l', 'rosbag_zed_odom'],
    output='screen')

zed_odom_republisher = launch_ros.actions.Node(
    package='republisher',
    executable='zed_odom_republisher',
    name='zed_odom_republisher',
    output='screen'
)


def generate_launch_description():
    return launch.LaunchDescription([

        gps_fix,
        gps_fix_republisher,
        gps_heading,
        lidar,
        zed_odometry,
        zed_odom_republisher,
    ])