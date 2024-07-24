import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    monitor = Node(
        package="wrg_core",
        executable="monitoring_node.py",
        name="monitoring_node",
        # output = "screen",
    )
    
    ld.add_action(monitor)

    return ld