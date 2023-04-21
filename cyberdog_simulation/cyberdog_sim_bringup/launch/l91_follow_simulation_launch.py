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
    # Get the launch directory
    package_dir = get_package_share_directory('mcr_bringup')
    param_dir = os.path.join(package_dir, 'params')
    bt_dir = os.path.join(package_dir, 'behavior_trees')

    follow_param_file = 'follow_params.yaml'
    bt_file = 'target_tracking.xml'

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    follow_params_file = LaunchConfiguration('follow_params_file')
    default_target_tracking_bt_xml = LaunchConfiguration('default_target_tracking_bt_xml')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = ['controller_server_tracking',
                       'planner_server_tracking',
                       'recoveries_server',
                       'bt_navigator_tracking']
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_target_tracking_bt_xml': default_target_tracking_bt_xml,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    configured_params_f = RewrittenYaml(
            source_file=follow_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

     # Set env var to print messages to stdout immediately
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_use_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'follow_params_file',
        default_value=os.path.join(param_dir, follow_param_file),
        description='Full path to the ROS2 parameters file to use')

    declare_default_target_tracking_bt_xml_cmd = DeclareLaunchArgument(
        'default_target_tracking_bt_xml',
        default_value=os.path.join(bt_dir, bt_file),
        description='Full path to the behavior tree xml file to use')

    declare_map_subscribe_transient_local_cmd = DeclareLaunchArgument(
        'map_subscribe_transient_local', default_value='true',
        description='Whether to set the map subscriber QoS to transient local')

    # Get the motion launch directory
    bringup_dir = get_package_share_directory('cyberdog_sim_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # motion
    motion_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'node.motion_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time}.items())
    # mcr_controller
    mcr_controller_cmd = Node(
        package='mcr_controller',
        executable='controller_server',
        name='controller_server_tracking',
        output='screen',
        # prefix=['xterm -e gdb  --args'],
        parameters=[{configured_params_f}],
        remappings=remappings)

    # mcr_planner
    mcr_planner_cmd = Node(
        package='mcr_planner',
        executable='mcr_planner_server',
        name='planner_server_tracking',
        output='screen',
        # prefix=['xterm -e gdb  --args'],
        parameters=[{configured_params_f}],
        remappings=remappings)

    # nav2_recoveries
    recoveries_cmd = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[{configured_params_f}],
        remappings=remappings)

    # bt_navigators
    bt_navigators_cmd = Node(
        package='bt_navigators',
        executable='bt_navigator_tracking',
        name='bt_navigator_tracking',
        output='screen',
        parameters=[{configured_params_f}],
        remappings=remappings)

    # lifecycle_manager
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_autostart_cmd) 
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_default_target_tracking_bt_xml_cmd)
    ld.add_action(declare_map_subscribe_transient_local_cmd)

    # Add the actions to launch all of the fnavigation nodes
    ld.add_action(motion_cmd)
    ld.add_action(mcr_controller_cmd)
    ld.add_action(mcr_planner_cmd)
    ld.add_action(recoveries_cmd)
    ld.add_action(bt_navigators_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld
