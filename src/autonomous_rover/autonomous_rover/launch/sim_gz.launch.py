"""Spawn the WAVE ROVER URDF in Gazebo Harmonic with controllers + ros_gz bridge."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("autonomous_rover")
    desc = os.path.join(pkg, "description")
    xacro_file = os.path.join(desc, "urdf", "wave_rover.urdf.xacro")
    bridge_cfg = os.path.join(desc, "config", "ros_gz_bridge.yaml")

    # robot_state_publisher takes the expanded xacro via Command substitution.
    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    world = LaunchConfiguration("world")

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={"gz_args": ["-r -v3 ", world]}.items(),
    )

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "wave_rover", "-z", "0.06"],
    )

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        parameters=[{"config_file": bridge_cfg, "use_sim_time": True}],
    )

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster"], output="screen")
    ddc = Node(package="controller_manager", executable="spawner",
               arguments=["diff_drive_controller"], output="screen")

    # Spawn controllers only after the entity exists.
    ctrl_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb, ddc])
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty.sdf"),
        gz, rsp, spawn, bridge, ctrl_after_spawn,
    ])
