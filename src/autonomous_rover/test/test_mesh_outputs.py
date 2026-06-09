import os
import math
import pytest

trimesh = pytest.importorskip("trimesh")
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.join(HERE, "..", "autonomous_rover", "description")
MESH = os.path.join(DESC, "meshes")
GEOM = os.path.join(DESC, "config", "geometry.yaml")

pytestmark = pytest.mark.skipif(
    not os.path.exists(GEOM), reason="run scripts/process_meshes.py first"
)


def test_meshes_exist_and_are_metric():
    chassis = trimesh.load(os.path.join(MESH, "chassis.stl"), force="mesh")
    wheel = trimesh.load(os.path.join(MESH, "wheel.stl"), force="mesh")
    # Chassis body fits in ~0.25 m (metres, not mm) but isn't cm-scaled either.
    assert 0.10 < max(chassis.extents) < 0.25
    # Wheel ~ cylinder d=0.08, w=0.0425.
    ext = sorted(wheel.extents)
    assert abs(ext[0] - 0.0425) < 0.015           # width
    assert abs(ext[2] - 0.08) < 0.012             # diameter


def test_geometry_has_four_symmetric_wheels():
    g = yaml.safe_load(open(GEOM))
    wc = g["wheel_centers"]
    assert set(wc) == {"front_left", "front_right", "rear_left", "rear_right"}
    # Left/right mirror in y, front/rear mirror in x, all near axle plane.
    assert math.isclose(wc["front_left"][1], -wc["front_right"][1], abs_tol=0.01)
    assert math.isclose(wc["front_left"][0], -wc["rear_left"][0], abs_tol=0.02)
    for c in wc.values():
        assert abs(c[2]) < 0.01
    assert 0.08 < g["track_width"] < 0.20
    assert 0.08 < g["wheelbase"] < 0.20
