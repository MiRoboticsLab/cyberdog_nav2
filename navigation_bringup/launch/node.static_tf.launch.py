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
    tf2_node_base_to_lidar = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_lidar',
                                namespace=namespace,
                                arguments=['0.179', '0', '0.0837','0', '0', '0', '1','base_link','laser_frame'],
                                )
    tf2_node_base_to_camera = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_depth_optical',
                                namespace=namespace,
                                arguments=['0.275309', '0.025', '0.114282','-0.545621', '0.545621', '-0.4497752', '0.4497752','base_link','camera_link'],
                                )
    tf2_node_base_to_tof_left_head = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_tof_left_head',
                                namespace=namespace,
                                arguments = ['0.259', '0.03', '0.102', '0', '-0.266', '0.296', 'base_link', 'left_head']
                                )
    tf2_node_base_to_tof_right_head = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_tof_right_head',
                                namespace=namespace,
                                arguments = ['0.259', '-0.03', '0.102', '0', '-0.266', '-0.296', 'base_link','right_head']
                                )
    tf2_node_base_to_tof_left_rear = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_tof_left_rear',
                                namespace=namespace,
                                arguments = ['-0.021', '0.042', '-0.051', '0', '0', '0.296', 'base_link', 'left_rear']
                                )
    tf2_node_base_to_tof_right_rear = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_tof_right_rear',
                                namespace=namespace,
                                arguments = ['-0.021', '-0.042', '-0.051', '0', '0', '-0.296', 'base_link', 'right_rear']
                                )
    # TODO：腿式里程计的位置
    tf2_node_map_to_odom = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_map_to_odom',
                                namespace=namespace,
                                arguments=['0', '0', '0','0', '0', '0', '1','map','odom'],
                                )
    # TODO：腿式里程计的位置
    tf2_node_map_to_vodom = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_map_to_vodom',
                                namespace=namespace,
                                arguments=['0', '0', '0','0', '0', '0', '1','map','vodom'],
                                )

    tf2_node_base_to_uwb = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_base_to_uwb',
                                namespace=namespace,
                                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'uwb'],
                                )        
    tf2_node_uwb_to_head_tof = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_uwb_to_head_uwb',
                                namespace=namespace,
                                arguments=['0.2185', '0', '-0.00495', '0', '0', '0', 'uwb', 'head_tof']
                                )

    tf2_node_uwb_to_head_uwb = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_uwb_to_head_tof',
                                namespace=namespace,
                                arguments=['0.17', '0', '0.164', '3.14159', '0', '0', 'uwb', 'head_uwb']
                                )

    tf2_node_uwb_to_rear_uwb = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_uwb_to_rear_uwb',
                                namespace=namespace,
                                arguments=['-0.023', '0.0845', '-0.00325', '1.5708', '0', '0', 'uwb', 'rear_uwb']
                                )

    tf2_node_uwb_to_rear_tof = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_uwb_to_rear_tof',
                                namespace=namespace,
                                arguments=['-0.0235', '-0.0845', '-0.00325', '-1.5708', '0', '0', 'uwb', 'rear_tof']
                                )

    # lds
    ld = launch.LaunchDescription([
        namespace_declare,
        tf2_node_base_to_lidar,
        tf2_node_base_to_camera,
        tf2_node_base_to_tof_left_head,
        tf2_node_base_to_tof_right_head,
        tf2_node_base_to_tof_left_rear,
        tf2_node_base_to_tof_right_rear,
        tf2_node_map_to_odom,
        tf2_node_map_to_vodom,
        tf2_node_base_to_uwb,
        tf2_node_uwb_to_head_uwb,
        tf2_node_uwb_to_head_tof,
        tf2_node_uwb_to_rear_uwb,
        tf2_node_uwb_to_rear_tof
    ])
    return ld
    
if __name__ == '__main__':
    generate_launch_description()
