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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    param_path = os.path.join(get_package_share_directory('cyberdog_tracking'), 'config')
    namespace = LaunchConfiguration('namespace', default='')
    namespace_declare = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')
    tracking_cmd = Node(
            package='cyberdog_tracking',
            executable='cyberdog_tracking',
            namespace=namespace,
            name='tracking',
            parameters=[{'logger_level': 0,
                         'ai_intrinsic': param_path}],
            remappings=None,
            arguments=None,
            output='screen',
        )
    ld = LaunchDescription([
        namespace_declare,
        tracking_cmd
    ])    
    return ld
    
if __name__ == '__main__':
    generate_launch_description()
