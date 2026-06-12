from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from launch_ros.parameter_descriptions import ParameterValue
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
    xacro_file = os.path.join(pkg_share, "description", "urdf", "wave_rover.urdf.xacro")

    sim_mode = LaunchConfiguration("simulation_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation_mode", default_value="false"),
            GroupAction(
                [
                    SetParametersFromFile(system_yaml),
                    SetParameter(name="simulation_mode", value=sim_mode),
                    # Publishes the static base_link -> camera_link_optical TF (and the rest
                    # of the tree) that rgbd_odometry/rtabmap need. No joint_states on the
                    # encoder-less rover, but the fixed sensor joints publish immediately.
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        name="robot_state_publisher",
                        parameters=[{
                            "robot_description": ParameterValue(
                                Command(["xacro ", xacro_file]), value_type=str
                            )
                        }],
                    ),
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
                        # The QNN HTP backend needs the DSP skel/shell search path and the
                        # QAIRT companion libs on the loader path; without these the depth
                        # net silently falls back to CPU (~40x slower). Device-specific.
                        additional_env={
                            "ADSP_LIBRARY_PATH": "/usr/lib/rfsa/adsp",
                            "LD_LIBRARY_PATH": [
                                EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                                ":/home/evc/qairt/2.35.0.250530/lib/aarch64-ubuntu-gcc9.4",
                            ],
                        },
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
                            # Depth lags ~0.6s behind its source image (estimator runs ~1.6Hz
                            # vs 30Hz camera); hold ~1s of image/info history so stamps match.
                            "topic_queue_size": 30,
                            "sync_queue_size": 30,
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
                            "topic_queue_size": 30,
                            "sync_queue_size": 30,
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
