import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  use_sim_time = True
  is_gravity_param_name = "is_gravity"
  is_gravity = LaunchConfiguration(is_gravity_param_name)

  launch_arguments = [
    DeclareLaunchArgument(
      is_gravity_param_name,
      default_value="true",
      description="Toggle the gravity acting on the robot on and off."
    )
  ]
  
  xacro_file = PathJoinSubstitution([FindPackageShare("closed_loop"), "urdf", "closed_loop.urdf.xacro"])
  robot_description = Command(
    [
      PathJoinSubstitution([FindExecutable(name="xacro")]), 
      " ",
      xacro_file,
      " ",
      "is_gravity:=", is_gravity
    ]
  )

  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[
      {"use_sim_time": use_sim_time}, 
      {"robot_description": robot_description}
    ]
  )

  gazebo_path = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
  gazebo_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(gazebo_path, "launch", "gazebo.launch.py")),
    launch_arguments={
      "use_sim_time": str(use_sim_time),
      "paused": "true"
    }.items()
  )

  gazebo_robot_description_spawner = Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    arguments=[
      "-entity", "closed_loop",
      "-topic", "robot_description"
    ],
    parameters=[
      {"use_sim_time": use_sim_time}
    ]
  )

  nodes = [
    robot_state_publisher_node,
    gazebo_node,
    gazebo_robot_description_spawner
  ]

  return LaunchDescription(launch_arguments + nodes)
