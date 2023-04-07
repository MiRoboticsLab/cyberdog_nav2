import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('cyberdog_rosbag_recorder')
    configured_params = os.path.join(bringup_dir, 'param', 'multiple_topics.yaml')

    namespace = LaunchConfiguration('namespace', default='')
    namespace_declare_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    rosbag_recorder_cmd = Node(
        package='cyberdog_rosbag_recorder',
        executable='rosbag_recorder',
        name='cyberdog_rosbag_recorder',
        namespace=namespace,
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(namespace_declare_cmd)
    ld.add_action(rosbag_recorder_cmd)
    return ld