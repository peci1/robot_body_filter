import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("body_filter_node"),
                    "examples",
                    "full_example.urdf",
                ]
            ),
            " sim_mode:=",
            LaunchConfiguration("sim_mode"),
        ]
    )
    config = os.path.join(
        get_package_share_directory('ros2_tutorials'),
        'config',
        'params.yaml'
        )

    node = Node(
        package="ros2_tutorials",
        name="your_amazing_node",
        executable="test_yaml_params",
        parameters=[config],
    )
    robot_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
        ],
    )
    ld.add_action(robot_description)
    ld.add_action(node)
    return ld
