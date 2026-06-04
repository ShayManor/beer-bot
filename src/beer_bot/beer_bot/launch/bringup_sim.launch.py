from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("beer_bot")

    system_yaml = os.path.join(pkg_share, "params", "system.yaml")
    camera_yaml = os.path.join(pkg_share, "params", "camera.yaml")
    safety_yaml = os.path.join(pkg_share, "params", "safety.yaml")
    pathfinder_yaml = os.path.join(pkg_share, "params", "pathfinder.yaml")
    localization_yaml = os.path.join(pkg_share, "params", "localization.yaml")
    e_comms_yaml = os.path.join(pkg_share, "params", "e_comms.yaml")
    master_yaml = os.path.join(pkg_share, "params", "master.yaml")

    sim_mode = LaunchConfiguration("simulation_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation_mode", default_value="true"),
            GroupAction(
                [
                    SetParametersFromFile(system_yaml),
                    SetParameter(name="simulation_mode", value=sim_mode),
                    Node(
                        package="beer_bot",
                        executable="camera_node",
                        name="camera_node",
                        parameters=[camera_yaml],
                    ),
                    Node(
                        package="beer_bot",
                        executable="pathfinder_node",
                        name="pathfinder_node",
                        parameters=[pathfinder_yaml, safety_yaml],
                    ),
                    Node(
                        package="beer_bot",
                        executable="localization_node",
                        name="localization_node",
                        parameters=[localization_yaml],
                    ),
                    Node(
                        package="beer_bot",
                        executable="e_comms_node",
                        name="e_comms_node",
                        parameters=[e_comms_yaml],
                    ),
                    Node(
                        package="beer_bot",
                        executable="master_node",
                        name="master_node",
                        parameters=[master_yaml],
                    ),
                ]
            ),
        ]
    )
