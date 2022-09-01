import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import subprocess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('navigation_bringup')
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    params_file = LaunchConfiguration('params_file')
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    autostart = LaunchConfiguration('autostart')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_config.rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}])

    # Launch RViz
    start_rviz_cmd = launch_ros.actions.Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])    

    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments = {'namespace': namespace,
                            'use_namespace': use_namespace,
                            'slam': slam,
                            'map': "",
                            'use_sim_time': use_sim_time,
                            'params_file': params_file,
                            'default_bt_xml_filename': default_bt_xml_filename,
                            'autostart': autostart}.items())
    ld = launch.LaunchDescription([
        DeclareLaunchArgument(
            name='default_bt_xml_filename',
            default_value=behavior_tree_xml_path,
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='False',
            description='Whether to start RVIZ'),

        DeclareLaunchArgument(
            name='rviz_config_file',
            default_value=default_rviz_config_path,
            description='Full path to the RVIZ config file to use'),

        DeclareLaunchArgument(
            name='slam',
            default_value='False',
            description='Whether to run SLAM'),

        DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            name='use_namespace',
            default_value='False',
            description='Whether to apply a namespace to the navigation stack'),

        DeclareLaunchArgument(
            name='params_file',
            default_value=nav2_params_path,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            name='autostart', 
            default_value='False',
            description='Automatically startup the nav2 stack'),

        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False'
        ),
        start_ros2_navigation_cmd,
        start_rviz_cmd
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
