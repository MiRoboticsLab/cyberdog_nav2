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
    namespace_declare = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace'
        )
    package_dir = get_package_share_directory('mcr_bringup')
    remappings = []
    recoveries_params_file = LaunchConfiguration('params_file',
        default=os.path.join(package_dir, 'params', 
        'recoveries_params.yaml')
        )
    map_subscribe_transient_local = LaunchConfiguration(
        'map_subscribe_transient_local',
        default='true'
        )
    recoveries_params = RewrittenYaml(
        source_file=recoveries_params_file,
        root_key=namespace,
        convert_types=True
        )
    recoveries_cmd = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[recoveries_params],
        namespace=namespace,
        remappings=remappings
        ),
########################  shared  ##############################

######################## tracking ##############################
    follow_params_file = LaunchConfiguration(
        'follow_params_file',
        default=os.path.join(
            os.path.join(package_dir, 'params'),
            'follow_params.yaml')
        )
    default_target_tracking_bt_xml = LaunchConfiguration(
        'default_target_tracking_bt_xml',
        default=os.path.join(
            os.path.join(package_dir, 'behavior_trees'),
            'target_tracking.xml')
        )
    param_substitutions_tracking = {
        'default_target_tracking_bt_xml': default_target_tracking_bt_xml,
        'map_subscribe_transient_local': map_subscribe_transient_local
        }
    configured_params_f = RewrittenYaml(
        source_file=follow_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions_tracking,
        convert_types=True
        )
    controller_tracking_cmd = Node(
        package='mcr_controller',
        executable='controller_server',
        name='controller_server_tracking',
        namespace=namespace,
        output='screen',
        parameters=[{configured_params_f}],
        remappings=remappings
        )
    planner_tracking_cmd = Node(
        package='mcr_planner',
        executable='mcr_planner_server',
        name='planner_server_tracking',
        namespace=namespace,
        output='screen',
        parameters=[{configured_params_f}],
        remappings=remappings
        )
    bt_navigator_tracking_cmd = Node(
        package='bt_navigators',
        executable='bt_navigator_tracking',
        name='bt_navigator_tracking',
        namespace=namespace,
        output='screen',
        parameters=[{configured_params_f}],
        remappings=remappings
        )
######################## tracking ##############################

######################## docking  ##############################
    auto_docking_file = LaunchConfiguration(
        'auto_charing_file',
        default=os.path.join(
            os.path.join(package_dir, 'params'),
            'auto_docking.yaml')
        )
    default_auto_docking_bt_xml = LaunchConfiguration(
        'default_auto_docking_bt_xml',
        default=os.path.join(
            os.path.join(package_dir, 'behavior_trees'),
            'auto_docking.xml')
        )
    param_substitutions_docking = {
        'default_auto_docking_bt_xml': default_auto_docking_bt_xml,
        'map_subscribe_transient_local': map_subscribe_transient_local
        }
    configured_params_a = RewrittenYaml(
        source_file=auto_docking_file,
        root_key=namespace,
        param_rewrites=param_substitutions_docking,
        convert_types=True)
    controller_docking_cmd = Node(
        package='mcr_controller',
        executable='controller_server',
        name='controller_server_docking',
        namespace=namespace,
        output='screen',
        parameters=[{configured_params_a}],
        remappings=remappings
        )
    planner_docking_cmd = Node(
        package='mcr_planner',
        executable='mcr_planner_server',
        name='planner_server_docking',
        namespace=namespace,
        output='screen',
        parameters=[{configured_params_a}],
        remappings=remappings
        )
    bt_navigator_docking_cmd = Node(
        package='bt_navigators',
        executable='bt_navigator_docking',
        name='bt_navigator_docking',
        namespace=namespace,
        output='screen',
        parameters=[{configured_params_a}],
        remappings=remappings
        )
######################## docking ##############################

######################## nav_ab  ##############################
    ab_params_file = LaunchConfiguration('ab_params_file',
        default=os.path.join(package_dir, 'params', 
        'navigate_to_pose_params.yaml')
        )
    configured_params_ab = RewrittenYaml(
        source_file=ab_params_file,
        root_key=namespace,
        convert_types=True
        )
    # SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    bt_navigator_ab_cmd = Node(
        package='bt_navigators',
        executable='bt_navigator_pose',
        name='bt_navigator_ab',
        output='screen',
        parameters=[configured_params_ab],
        namespace=namespace,
        remappings=remappings
        ),
    controller_ab_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server_ab',
        output='screen',
        parameters=[configured_params_ab],
        namespace=namespace,
        remappings=remappings
        ),
    planner_ab_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server_ab',
        output='screen',
        parameters=[configured_params_ab],
        namespace=namespace,
        remappings=remappings
        ),
######################## nav_ab ##############################

    ld = launch.LaunchDescription([
        namespace_declare,
        controller_tracking_cmd,
        planner_tracking_cmd,
        bt_navigator_tracking_cmd,
        controller_docking_cmd,
        planner_docking_cmd,
        bt_navigator_docking_cmd,
        controller_ab_cmd,
        planner_ab_cmd,
        bt_navigator_ab_cmd,
        recoveries_cmd,
        # map_server_cmd
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
