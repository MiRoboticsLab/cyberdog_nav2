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

from cgi import parse_multipart
import os
import sys
from tkinter.tix import Tree

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
from launch.actions import GroupAction 
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    
    namespace = LaunchConfiguration('namespace')
    namespace_declare = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    odom_out_cmd = Node(
            package='motion_bridge',
            executable='odom_out_publisher',
            name='odom_out_publisher',
            namespace=namespace,
            parameters=[{'odom_frame': LaunchConfiguration('odom_frame', default='odom'),
                        'base_frame': LaunchConfiguration('base_frame', default='base_link_leg'),
                        'map_frame': LaunchConfiguration('map_frame', default='map'),
                        'tf_pub': LaunchConfiguration('tf_pub', default=True)}]
            )
    motion_manager_cmd = Node(
            package='motion_manager',
            executable='motion_manager',
            name='motion_manager',
            namespace=namespace,
            )
    sensor_manager_cmd = Node(
            package='sensor_manager',
            executable='sensor_manager',
            name='sensor_manager',
            namespace=namespace,
            parameters=[
            {
                # 'simulator': LaunchConfiguration('simulator', default='[lidar, tof, gps, ultrasonic]'),
                'simulator': LaunchConfiguration('simulator', default='[lidar, gps, ultrasonic]'),
            },],
            )

    device_manager_cmd = Node(
            package='device_manager',
            executable='device_manager',
            name='device_manager',
            namespace=namespace,
            output={'stdout': 'log',
                    'stderr': 'log'},
        #     parameters=[
        #     {
        #         'simulator': LaunchConfiguration('simulator', default='[uwb, touch]'),
        #     },],
            )

    head_tof_pc_cmd = Node(
            package='motion_utils',
            executable='head_tof_pcl_publisher.py',
            name='head_tof_pcl_publisher',
            namespace=namespace,
            )

    ld = launch.LaunchDescription([
        namespace_declare,
        odom_out_cmd,
        motion_manager_cmd,
        sensor_manager_cmd,
        device_manager_cmd,
        head_tof_pc_cmd
    ])

    return ld

if __name__ == '__main__':
    generate_launch_description()
