from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("autonomous_rover")

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
                        package="autonomous_rover",
                        executable="camera_node",
                        name="camera_node",
                        parameters=[camera_yaml],
                    ),
                    Node(
                        package="autonomous_rover",
                        executable="pathfinder_node",
                        name="pathfinder_node",
                        parameters=[pathfinder_yaml, safety_yaml],
                    ),
                    Node(
                        package="autonomous_rover",
                        executable="localization_node",
                        name="localization_node",
                        parameters=[localization_yaml],
                    ),
                    Node(
                        package="rtabmap_odom",
                        executable="rgbd_odometry",
                        name="rgbd_odometry",
                        output="screen",
                        parameters=[{
                            "frame_id": "base_link",
                            "approx_sync": True,
                            "subscribe_depth": True,
                        }],
                        remappings=[
                            ("rgb/image", "/camera/image_raw"),
                            ("depth/image", "/camera/depth"),
                            ("rgb/camera_info", "/camera/camera_info"),
                            ("odom", "/odom"),
                        ],
                    ),
                    Node(
                        package="rtabmap_slam",
                        executable="rtabmap",
                        name="rtabmap",
                        output="screen",
                        parameters=[{
                            "frame_id": "base_link",
                            "subscribe_depth": True,
                            "approx_sync": True,
                            "Rtabmap/DetectionRate": "2.0",
                        }],
                        remappings=[
                            ("rgb/image", "/camera/image_raw"),
                            ("depth/image", "/camera/depth"),
                            ("rgb/camera_info", "/camera/camera_info"),
                            ("odom", "/odom"),
                            ("cloud_map", "/cloud_map"),
                            ("grid_map", "/grid"),
                        ],
                        arguments=["--delete_db_on_start"],
                    ),
                    Node(
                        package="autonomous_rover",
                        executable="e_comms_node",
                        name="e_comms_node",
                        parameters=[e_comms_yaml],
                    ),
                    Node(
                        package="autonomous_rover",
                        executable="master_node",
                        name="master_node",
                        parameters=[master_yaml],
                    ),
                ]
            ),
        ]
    )
