import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Define the path to the launch file directory
    wrg_launch_file_dir = os.path.join(get_package_share_directory('wrg_core'), 'launch')
    wrg_gazebo_launch_file_dir = os.path.join(get_package_share_directory('wrg_turtlebot3_gazebo'), 'launch')

    # Include navigate.launch.py
    navigate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wrg_launch_file_dir, 'navigate.launch.py')
        )
    )
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wrg_gazebo_launch_file_dir, 'wrg_world.launch.py')
        )
    )

    # Add actions to the launch description
    ld.add_action(navigate_launch)
    ld.add_action(gazebo_launch)

    return ld

if __name__ == '__main__':
    generate_launch_description()
