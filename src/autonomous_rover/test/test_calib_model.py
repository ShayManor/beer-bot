import numpy as np
from autonomous_rover.nodes.master.calibration.model_calib import floor_truth, fit_affine


def test_fit_affine_recovers_known_coeffs():
    raw = np.linspace(0.3, 4.0, 200)
    true = 1.3 * raw - 0.05
    a, b, res = fit_affine(raw, true)
    assert abs(a - 1.3) < 1e-3
    assert abs(b + 0.05) < 1e-3
    assert res < 1e-6


def test_fit_affine_rejects_outliers():
    rng = np.random.default_rng(0)
    raw = np.linspace(0.3, 4.0, 200)
    true = 1.1 * raw + 0.02
    true[::20] += 5.0  # gross outliers (untextured-wall style)
    a, b, res = fit_affine(raw, true)
    assert abs(a - 1.1) < 0.05
    assert abs(b - 0.02) < 0.05


def test_fit_affine_unsorted_odd_with_outliers():
    rng = np.random.default_rng(1)
    raw = np.linspace(0.3, 4.0, 201)   # odd length
    true = 1.1 * raw + 0.02
    true[::20] += 5.0                  # gross outliers
    perm = rng.permutation(len(raw))   # shuffle so input is unsorted
    a, b, res = fit_affine(raw[perm], true[perm])
    assert abs(a - 1.1) < 0.05
    assert abs(b - 0.02) < 0.05


def test_floor_truth_matches_geometry():
    # Build points exactly on a plane at height h with a known unit normal.
    h = 0.19
    normal = np.array([0.0, np.cos(0.2), np.sin(0.2)])  # tilted "up-ish" in cam frame
    normal = normal / np.linalg.norm(normal)
    # Rays toward the floor; pick z>0 points and place them ON the true plane.
    rays = np.array([[0.1, 0.4, 1.0], [-0.2, 0.5, 1.0], [0.0, 0.6, 1.0]])
    # depth so the point sits at distance h from origin along the normal:
    z = h / (rays @ normal)
    pts = rays * z[:, None]
    raw_z, true_z = floor_truth(pts, normal, h)
    assert np.allclose(true_z, z, atol=1e-9)
    assert np.allclose(raw_z, pts[:, 2], atol=1e-9)
