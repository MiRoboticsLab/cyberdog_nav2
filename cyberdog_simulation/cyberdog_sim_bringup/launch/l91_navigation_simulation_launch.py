# Copyright (c) 2018 Intel Corporation
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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the motion launch directory
    bringup_dir = get_package_share_directory('cyberdog_sim_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
  
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # motion
    motion_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'node.motion_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time}.items())

    # Get the state_publisher launch directory
    urdf_bringup_dir = get_package_share_directory('navigation_bringup')
    urdf_launch_dir = os.path.join(urdf_bringup_dir, 'launch')

    # state_publisher
    state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(urdf_launch_dir, 'node.state_publisher.launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time}.items())

    # navigation
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'node.navigation_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time}.items())
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to launch all of the fnavigation nodes
    ld.add_action(motion_cmd)
    ld.add_action(state_publisher_cmd)
    ld.add_action(navigation_cmd)

    return ld
