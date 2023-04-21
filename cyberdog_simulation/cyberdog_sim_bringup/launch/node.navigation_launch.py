# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('cyberdog_sim_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Get the navigation launch directory
    nav2_bringup_dir = get_package_share_directory('navigation_bringup')
    nav2_launch_dir =  os.path.join(nav2_bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    # lifecycle_nodes = ['controller_server_ab',
    #                    'planner_server_ab',
    #                    'map_server',
    #                    'recoveries_server',
    #                    'bt_navigator_ab']

    lifecycle_nodes = ['planner_server_ab',
                       'map_server',
                       'recoveries_server',
                       'bt_navigator_ab']


    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'navigation_pose_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # map_server
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        namespace=namespace)

    # bt_navigators
    bt_navigator_cmd = Node(
        package='bt_navigators',
        executable='bt_navigator_pose',
        name='bt_navigator_ab',
        output='screen',
        parameters=[configured_params],
        namespace=namespace)

    # controller_server
    # controller_cmd = Node(
    #     package='nav2_controller',
    #     executable='controller_server',
    #     name='controller_server_ab',
    #     output='screen',
    #     parameters=[configured_params],
    #     namespace=namespace,
    #     remappings=remappings)
    
    # planner_server
    planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server_ab',
        output='screen',
        parameters=[configured_params],
        namespace=namespace)

    # recoveries_server
    recovery_cmd = Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            namespace=namespace,
            parameters=[configured_params])

    # Auto start lifecycle manager
    lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_laser_slam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add the actions to launch all of the navigation nodes 
    ld.add_action(map_server_cmd)
    ld.add_action(bt_navigator_cmd)
    # ld.add_action(controller_cmd)
    ld.add_action(planner_cmd)
    ld.add_action(recovery_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld
