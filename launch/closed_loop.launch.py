from launch import LaunchDescription, launch_description
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), 
            ' ',
            PathJoinSubstitution([FindPackageShare('closed_loop'), 'model', 'closed_loop.xacro']),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    gazebo_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-entity', 'closed_loop', '-topic', 'robot_description']
        )

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros,"launch","gazebo.launch.py"))
    )

    nodes = [
        robot_state_pub_node,
        gazebo_spawner,
    ]

    return LaunchDescription(nodes + [gazebo_launch])