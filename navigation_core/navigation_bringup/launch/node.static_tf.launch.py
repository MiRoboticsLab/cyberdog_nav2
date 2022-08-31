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

    tf2_node_base_to_lidar = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_lidar',
                                arguments=['0.179', '0', '0.0837','0', '0', '0', '1','base_link','laser_frame'],
                                )
    tf2_node_base_to_camera = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_depth_optical',
                                arguments=['0.275309', '0.025', '0.114282','-0.545621', '0.545621', '-0.4497752', '0.4497752','base_link','camera_depth_optical_frame'],
                                )
    tf2_node_base_to_tof_left_head = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_tof_left_head',
                                arguments = ['0.259', '0.03', '0.102', '0', '-0.266', '0.296', 'base_link', 'left_head']
                                )
    tf2_node_base_to_tof_right_head = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_tof_right_head',
                                arguments = ['0.259', '-0.03', '0.102', '0', '-0.266', '-0.296', 'base_link','right_head']
                                )
    tf2_node_base_to_tof_left_rear = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_tof_left_rear',
                                arguments = ['-0.021', '0.042', '-0.051', '0', '0', '0.296', 'base_link', 'left_rear']
                                )
    tf2_node_base_to_tof_right_rear = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_tof_right_rear',
                                arguments = ['-0.021', '-0.042', '-0.051', '0', '0', '-0.296', 'base_link', 'right_rear']
                                )
    # TODO：腿式里程计的位置
    tf2_node_map_to_odom = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_map_to_odom',
                                arguments=['0', '0', '0','0', '0', '0', '1','map','odom'],
                                )
    # lds
    ld = launch.LaunchDescription([
        tf2_node_base_to_lidar,
        tf2_node_base_to_camera,
        tf2_node_base_to_tof_left_head,
        tf2_node_base_to_tof_right_head,
        tf2_node_base_to_tof_left_rear,
        tf2_node_base_to_tof_right_rear,
        # tf2_node_map_to_odom,
    ])
    return ld
    
if __name__ == '__main__':
    generate_launch_description()
