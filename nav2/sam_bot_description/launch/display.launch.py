import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', [os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')]]
    )
    ekf_filter_local_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_local_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf3.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
       remappings=[('/odometry/filtered', '/odometry/filtered_local')]
    )
    ekf_filter_global_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_global_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf3.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
       remappings=[('/odometry/filtered', '/odometry/filtered_global')]

    )
    navsat_transform_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf3.yaml')],
        remappings=[('/gps/fix', '/base/fix_qos_sensor'),
                    ('/odometry/filtered', '/odometry/filtered_global')]
    )

    wgs84_transformer_node = launch_ros.actions.Node(
              package="swri_transform_util",
              executable="initialize_origin.py",
              name="initialize_origin",
              parameters=[
                  {"local_xy_frame": "map"},
                  {"local_xy_origin": "auto"},
                  {"local_xy_navsatfix_topic": "/base/fix_qos_reliable"},
                  {'local_xy_gpsfix_topic': '/local_xy_gpsfix_topic'},
                #   {
                #       "local_xy_origins": [
                #           33.55343555,
                #           -117.663964169,
                #           0.0,
                #           0.0,
                #       ]
                #   },
                  {"use_sim_time": False},
              ],
          )
    
    mapviz = launch_ros.actions.Node(
              package="mapviz",
              executable="mapviz",
              name="mapviz_node",
          )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),

        # joint_state_publisher_node,
        
        wgs84_transformer_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        ekf_filter_global_node,
        ekf_filter_local_node,
        navsat_transform_node,
        mapviz,
        rviz_node
    ])
