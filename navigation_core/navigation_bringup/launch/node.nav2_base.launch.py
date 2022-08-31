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

    namespace = LaunchConfiguration('namespace', default='')
    namespace_declare = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace'
        )
    package_dir = get_package_share_directory('mcr_bringup')
    # param_dir = os.path.join(package_dir, 'params')
    # nav_param_file = 'nav2_params.yaml'
    # follow_param_file = 'follow_params.yaml'
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    params_file = LaunchConfiguration('params_file')
    params_file_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            os.path.join(package_dir, 'params'),
            'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use'
        )
    follow_params_file = LaunchConfiguration('follow_params_file')
    follow_params_file_declare = DeclareLaunchArgument(
        'follow_params_file',
        default_value=os.path.join(
            os.path.join(package_dir, 'params'),
            'follow_params.yaml'),
        description='Full path to the ROS2 parameters file to use'
        )
    # bt_dir = os.path.join(package_dir, 'behavior_trees')
    # bt_file = 'target_tracking.xml'
    default_target_tracking_bt_xml = LaunchConfiguration('default_target_tracking_bt_xml')
    default_target_tracking_bt_xml_declare = DeclareLaunchArgument(
        'default_target_tracking_bt_xml',
        default_value=os.path.join(
            os.path.join(package_dir, 'behavior_trees'),
            'target_tracking.xml'),
        description='Full path to the behavior tree xml file to use'
        )
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    map_subscribe_transient_local_declare = DeclareLaunchArgument(
        'map_subscribe_transient_local', 
        default_value='true',
        description='Whether to set the map subscriber QoS to transient local'
        )
    param_substitutions = {
        'default_target_tracking_bt_xml': default_target_tracking_bt_xml,
        'map_subscribe_transient_local': map_subscribe_transient_local
        }

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True
            )
    configured_params_f = RewrittenYaml(
            source_file=follow_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True
            )
    controller_cmd = Node(
            package='mcr_controller',
            executable='controller_server',
            output='screen',
            parameters=[{configured_params},{configured_params_f}],
            remappings=remappings
            )
    ld = launch.LaunchDescription([
        namespace_declare,
        params_file_declare,
        follow_params_file_declare,
        default_target_tracking_bt_xml_declare,
        map_subscribe_transient_local_declare,
        controller_cmd
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
