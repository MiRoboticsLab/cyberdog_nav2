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
import launch_ros.actions
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import subprocess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
sys.path.append(os.path.join(get_package_share_directory('cyberdog_bringup'), 'bringup'))
from manual import get_namespace

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default=get_namespace())
    namespace_declare = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')
    nav2_dir = FindPackageShare(package='navigation_bringup').find('navigation_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    node_lists = [
        'static_tf',
        'vision_manager',
        'camera_server',
        'tracking',
        'realsense',
        'mcr_uwb',
        'velocity_adaptor',
        'nav2_base',
        'laser_mapping',
        'laser_localization',
        'lifecycle_mgr_localization',
        'lifecycle_mgr_nav',
        'lifecycle_mgr_mapping',
        'lifecycle_mgr_mcr_uwb'
        ]
    lds = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'node.' + node + '.launch.py')),
        launch_arguments = {'namespace': namespace}.items()) for node in node_lists]
    return launch.LaunchDescription(lds + [namespace_declare])

if __name__ == '__main__':
    generate_launch_description()