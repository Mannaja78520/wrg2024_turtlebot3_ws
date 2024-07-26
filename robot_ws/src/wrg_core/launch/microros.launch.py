from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    node_microros_1 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        # output="screen",
        arguments=["serial", "--dev", "/dev/ttyACM0"],
        on_exit=[
            ExecuteProcess(
                cmd=['killall', 'micro_ros_agent'],
                output='screen'
            )
        ]
    )
    
    ld.add_action(node_microros_1)

    return ld