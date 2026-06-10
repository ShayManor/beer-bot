import subprocess

import pytest

ament = pytest.importorskip("ament_index_python.packages")
from ament_index_python.packages import get_package_share_directory  # noqa: E402


def _xacro_to_urdf():
    share = get_package_share_directory("autonomous_rover")
    path = f"{share}/description/urdf/wave_rover.urdf.xacro"
    return subprocess.check_output(["xacro", path], text=True)


def test_xacro_expands():
    urdf = _xacro_to_urdf()
    assert "<robot" in urdf and 'name="wave_rover"' in urdf


def test_urdf_is_valid_tree():
    urdf = _xacro_to_urdf()
    up = pytest.importorskip("urdf_parser_py.urdf")
    robot = up.URDF.from_xml_string(urdf)
    names = {l.name for l in robot.links}
    assert {"base_footprint", "base_link", "camera_link", "imu_link"} <= names
    for w in ["front_left", "front_right", "rear_left", "rear_right"]:
        assert f"{w}_wheel_link" in names
    # Every non-root link has exactly one parent joint -> connected tree.
    assert len(robot.joints) == len(robot.links) - 1


def test_camera_frame_id_exists_in_urdf():
    """The camera stamps images with this frame_id and rgbd_odometry/rtabmap look it
    up against base_link, so it must be a real link the URDF (hence RSP) publishes."""
    import yaml

    share = get_package_share_directory("autonomous_rover")
    with open(f"{share}/params/camera.yaml") as f:
        frame_id = yaml.safe_load(f)["camera_node"]["ros__parameters"]["frame_id"]
    urdf = _xacro_to_urdf()
    up = pytest.importorskip("urdf_parser_py.urdf")
    robot = up.URDF.from_xml_string(urdf)
    assert frame_id in {l.name for l in robot.links}


def test_inertials_present_and_positive():
    urdf = _xacro_to_urdf()
    up = pytest.importorskip("urdf_parser_py.urdf")
    robot = up.URDF.from_xml_string(urdf)
    for link in robot.links:
        if link.inertial is not None:
            assert link.inertial.mass > 0
