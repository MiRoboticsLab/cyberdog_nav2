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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_control_demo')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator']

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'navigate_to_pose_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        Node(
            package='nav2_control_demo',
            executable='nav2_control_demo',
            name='nav2_control_demo',
            namespace=namespace,
            output='screen',
            parameters=[configured_params]),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params]),
        
         Node(
            package='nav2_map_server',
            executable='map_server',
            namespace=namespace,
            name='map_server',
            output='screen',
            parameters=[configured_params]),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            parameters=[configured_params]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
    ])
