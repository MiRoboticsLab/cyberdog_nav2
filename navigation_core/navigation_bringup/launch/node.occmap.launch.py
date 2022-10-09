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

    occmap_dir = FindPackageShare(package='cyberdog_occmap').find('cyberdog_occmap') 
    occmap_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(occmap_dir, 'launch/create_gridmap_node.launch.py')),
            launch_arguments={'namespace': namespace}.items()
        )   

    ld = launch.LaunchDescription([
        namespace_declare,
        occmap_cmd,
    ])

    return ld

if __name__ == '__main__':
    generate_launch_description()