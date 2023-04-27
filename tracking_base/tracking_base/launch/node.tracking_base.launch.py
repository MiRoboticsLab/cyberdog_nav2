#!/usr/bin/python3
#
# Copyright (c) 2021 Xiaomi Corporation
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
import sys

import launch
import subprocess
import launch_ros.actions
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import subprocess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

######################## shared ##############################
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    namespace_declare = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace'
        )
    package_dir = get_package_share_directory('tracking_base')
    remappings = []
    map_subscribe_transient_local = LaunchConfiguration(
        'map_subscribe_transient_local',
        default='true'
        )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_use_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    follow_params_file = LaunchConfiguration(
        'follow_params_file',
        default=os.path.join(
            os.path.join(package_dir, 'params'),
            'tracking_base.yaml')
        )
    param_substitutions_tracking = {
        'map_subscribe_transient_local': map_subscribe_transient_local
        }
    configured_params_f = RewrittenYaml(
        source_file=follow_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions_tracking,
        convert_types=True
        )

    lifecycle_nodes = ['tracking_base']

    tracking_base_cmd = Node(
        package='tracking_base',
        executable='tracking_base',
        name='tracking_base',
        namespace=namespace,
        output='screen',
        parameters=[{configured_params_f}],
        remappings=remappings
        )
    tracking_pose_pub_cmd = Node(
        package='tracking_pose_pub',
        executable='subscriber_member_function',
        name='tracking_pose_pub',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        # parameters=[{'use_sim_time': use_sim_time}]
        )

    # lifecycle_manager
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    ld = launch.LaunchDescription([
        namespace_declare,
        declare_use_sim_time_cmd,
        # declare_use_autostart_cmd,
        tracking_base_cmd,
        tracking_pose_pub_cmd
        # lifecycle_manager_cmd
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
