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

    rosbag_recorder_cmd = Node(
        package='cyberdog_rosbag_recorder',
        executable='rosbag_recorder',
        name='cyberdog_rosbag_recorder',
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(rosbag_recorder_cmd)
    return ld