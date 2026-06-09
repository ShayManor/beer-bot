import os
import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "autonomous_rover", "description", "config")
PARAMS = os.path.join(CONFIG, "inertial_params.yaml")

pytestmark = pytest.mark.skipif(
    not os.path.exists(PARAMS), reason="run scripts/build_inertial_prior.py first"
)


def _tensor(d):
    return np.array([[d["ixx"], d["ixy"], d["ixz"]],
                     [d["ixy"], d["iyy"], d["iyz"]],
                     [d["ixz"], d["iyz"], d["izz"]]])


def _assert_physical(block):
    assert block["mass"] > 0
    w = np.linalg.eigvalsh(_tensor(block["inertia"]))
    assert (w > 0).all(), f"inertia not positive-definite: {w}"
    # Principal-moment triangle inequalities.
    a, b, c = sorted(w)
    assert a + b >= c - 1e-9


def test_base_and_wheel_are_physical():
    p = yaml.safe_load(open(PARAMS))
    _assert_physical(p["base_link"])
    _assert_physical(p["wheel"])


def test_total_mass_plausible():
    p = yaml.safe_load(open(PARAMS))
    total = p["base_link"]["mass"] + 4 * p["wheel"]["mass"]
    assert 0.8 < total < 1.5  # base 0.86 kg + this rover's payload
