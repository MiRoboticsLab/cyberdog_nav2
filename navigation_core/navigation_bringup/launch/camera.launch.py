#   lidar
#   odom
#   realsense
#   camera

#sudo chmod a+rw /dev/ttyTHS0
#sudo usermod -aG dialout mi
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
    # Launch the ROS 2 Navigation Stack

def generate_launch_description():
    sudoPassword = '123'
    command = 'chmod a+rw /dev/ttyTHS0'
    command1 = 'usermod -aG dialout mi'
    command_route = 'route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0'

    os.system('echo %s|sudo -S %s' % (sudoPassword, command))
    os.system('echo %s|sudo -S %s' % (sudoPassword, command1))
    os.system('echo %s|sudo -S %s' % (sudoPassword, command_route))

    #odom
    start_odom_cmd = launch_ros.actions.Node(
        package='odom_publisher',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        # prefix=['xterm -e gdb  --args'],
        namespace='/')

    start_motion_cmd = launch_ros.actions.Node(
        package='motion_sender',
        executable='sendlcm',
        name='motion_sender',
        output='screen',
        namespace='/')

    start_get_pose_cmd = launch_ros.actions.Node(
        package='positionchecker',
        executable='pose_server',
        name='pose_server',
        output='screen',
        namespace='/')

    realsense_dir = FindPackageShare(package='realsense2_camera').find('realsense2_camera') 
    nav2_realsense_dir = os.path.join(realsense_dir, 'launch')
   
    #realsense 
    start_realsense_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(nav2_realsense_dir, 'on_dog.py')))   

    #camera for mapping
    camera_params_file = LaunchConfiguration('camera_params_file')
    camera_share_dir = get_package_share_directory('laser_slam')

    #lidar for localization
    camera_localization_params_file = LaunchConfiguration('camera_localization_params_file')

    #lidar
    #ydlidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    #yilidar_params_file = LaunchConfiguration('yilidar_params_file')


    #elevation_mapping
    # elevation_mapping_share_dir = get_package_share_directory('elevation_mapping')
    # elevation_mapping_params_file = LaunchConfiguration('elevation_mapping_params_file')


    #lifecycle nodes
    lifecycle_data_nodes = ['lidar_controller', 'realsense_controller', 'odom_controller']
    lifecycle_maping_nodes = ['map_builder']
    #lifecycle_elevation_nodes = ['lifecycle_manager_elevation']

    #navigations
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    pkg_share = get_package_share_directory('navigation_bringup')
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    camera_node = LifecycleNode(
                                package='laser_slam',
                                executable='mapping',
                                name='map_builder',
                                output='screen',
                                emulate_tty=True,
                                parameters=[camera_params_file],
                                namespace='/',
                                )

    camera_node_relocation = LifecycleNode(
                                package='laser_slam',
                                executable='demo',
                                name='demo_node',
                                output='screen',
                                emulate_tty=True,
                                #prefix=['xterm -e gdb  --args'],
                                parameters=[camera_localization_params_file],
                                namespace='/',
                                )

    # ydlidar_node = LifecycleNode(
    #                             package='ydlidar_ros2_driver',
    #                             executable='ydlidar_ros2_driver_node',
    #                             name='ydlidar_ros2_driver_node',
    #                             output='screen',
    #                             emulate_tty=True,
    #                             parameters=[yilidar_params_file],
    #                             namespace='/',
    #                             )

    tf2_node = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_laser',
                                arguments=['0.179', '0', '0.0837','1', '0', '0', '0','base_link','lidar_link'],
                                )

    tf2_node_depth_base = Node(package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_laser',
                                arguments=['0.275309', '0.025', '0.114282','-0.545621', '0.545621', '-0.4497752', '0.4497752','base_link','camera_depth_optical_frame'],
                                )

    # tf2_node_elevation_mapping = Node(
    #                            package='elevation_mapping',
    #                            executable='elevation_mapping',
    #                            name='elevation_mapping',
    #                            output='screen',
    #                            emulate_tty=True,
    #                            parameters=[elevation_mapping_params_file],
    #                            namespace='/',
    #                            )
    tf2_node_vodom_map = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_laser',
                                arguments=['0', '0', '0','0', '0', '0', '1','map','vodom'],
                                )
    tf2_node_odom_map = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                name='static_tf_pub_laser',
                                arguments=['0', '0', '0','0', '0', '0', '1','map','odom_out'],
                                )
    lidar_controller = LifecycleNode(
                                package='lidar_controller',
                                executable='lidar_controller',
                                name='lidar_controller',
                                output='screen',
                                namespace='/',
                                emulate_tty=True)       
    realsense_controller = LifecycleNode(
                                package='realsense_controller',
                                executable='realsense_controller',
                                name='realsense_controller',
                                output='screen',
                                namespace='/',
                                emulate_tty=True)
    odom_controller = LifecycleNode(
                                package='odom_controller',
                                executable='odom_controller',
                                name='odom_controller',
                                output='screen',
                                namespace='/',
                                emulate_tty=True)   
    lifecycle_data = Node(
                                package='nav2_lifecycle_manager',
                                executable='lifecycle_manager',
                                name='lifecycle_manager_data',
                                output='screen',
                                parameters=[{'use_sim_time': use_sim_time},
                                            {'autostart': autostart},
                                            {'node_names': lifecycle_data_nodes}])
    lifecycle_mapping = Node(
                                package='nav2_lifecycle_manager',
                                executable='lifecycle_manager',
                                name='lifecycle_manager_mapping',
                                output='screen',
                                parameters=[{'use_sim_time': use_sim_time},
                                            {'autostart': autostart},
                                            {'node_names': lifecycle_maping_nodes}])
    # lifecycle_elevation = Node(
    #                             package='nav2_lifecycle_manager',
    #                             executable='lifecycle_manager',
    #                             name='lifecycle_manager_elevation',
    #                             output='screen',
    #                             parameters=[{'use_sim_time': use_sim_time},
    #                                         {'autostart': autostart},
    #                                         {'node_names': lifecycle_elevation_nodes}])


    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments = {'namespace': namespace,
                            'use_namespace': use_namespace,
                            'slam': "False",
                            'map': "/home/mi/mapping/map.yaml",
                            'use_sim_time': use_sim_time,
                            'params_file': params_file,
                            'default_bt_xml_filename': default_bt_xml_filename,
                            'autostart': autostart}.items())
    # lds
    ld = launch.LaunchDescription([
        # DeclareLaunchArgument(
        #     name='yilidar_params_file',
        #     default_value=os.path.join(
        #     ydlidar_share_dir, 'params', 'ydlidar.yaml'),
        #     description='FPath to the ROS2 parameters file to use.'),

        DeclareLaunchArgument(
            name='camera_params_file',
            default_value=os.path.join(
            camera_share_dir, 'param', 'mapping_node.yaml'),
            description='FPath to the ROS2 parameters file to use.'),

        DeclareLaunchArgument(
            name='camera_localization_params_file',
            default_value=os.path.join(
            camera_share_dir, 'param', 'localization.yaml'),
            description='FPath to the ROS2 parameters file to use.'),

        # DeclareLaunchArgument(
        #    name='elevation_mapping_params_file',
        #    default_value=os.path.join(
        #    elevation_mapping_share_dir, 'launch', 'mi_elevation_mapping_param_l91_test.yaml'),
        #    description='FPath to the elevation_maping parameters file to use.'),

        DeclareLaunchArgument(
            name='default_bt_xml_filename',
            default_value=behavior_tree_xml_path,
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'autostart', default_value='false',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='Top-level namespace'),
    
        DeclareLaunchArgument(
            name='use_namespace',
            default_value='False',
            description='Whether to apply a namespace to the navigation stack'),

        DeclareLaunchArgument(
            name='autostart', 
            default_value='False',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False'
        ),

        DeclareLaunchArgument(
            name='params_file',
            default_value=nav2_params_path,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        #start_odom_cmd,
        odom_controller,
        lidar_controller,
        realsense_controller,
        start_get_pose_cmd,
        tf2_node,
        tf2_node_depth_base,
        tf2_node_vodom_map,
        tf2_node_odom_map,
        start_realsense_cmd,
        # ydlidar_node,
        camera_node,
        #camera_node_relocation,
        #start_motion_cmd,
        #tf2_node_elevation_mapping,
        #lifecycle_elevation,
        lifecycle_data,
        lifecycle_mapping,
    ])
    return ld
if __name__ == '__main__':
    generate_launch_description()
