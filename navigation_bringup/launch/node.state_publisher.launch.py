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

def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='')
    namespace_declare = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    urdf_file = 'static_tf.urdf'
    urdf_map_file = 'static_tf_map.urdf'
    urdf = os.path.join(
        get_package_share_directory('navigation_bringup'),
        'urdf',
        urdf_file) 
    urdf_map = os.path.join(
        get_package_share_directory('navigation_bringup'),
        'urdf',
        urdf_map_file)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    with open(urdf_map, 'r') as infp_map:
      robot_desc_map = infp_map.read()
    rsp_params = {'robot_description': robot_desc}
    rsp_params_map = {'robot_description': robot_desc_map}

    base_link_urdf = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[rsp_params],
            )

    map_urdf = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[rsp_params_map],
            )

    ld = launch.LaunchDescription([
        namespace_declare,
        base_link_urdf,
        map_urdf,
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()