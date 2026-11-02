"""Simulated scene like the Oct 15 test: camera 0.55 m up, tilted 30 deg, with a berm, a boulder
and a crater (see terrain_core.default_terrain)."""
import numpy as np
import pytest

from rover_perception.terrain_core import (BUMP, FREE, TerrainParams, analyze_depth, synthetic_scene,
                                           to_occupancy)


@pytest.fixture(scope="module")
def scene():
    return synthetic_scene(320, 240)


def _find(feats, kind, x, y, tol=0.15):
    hits = [f for f in feats if f.kind == kind and abs(f.x - x) < tol and abs(f.y - y) < tol]
    assert hits, f"no {kind} near ({x}, {y}); got {[f.as_dict() for f in feats]}"
    return hits[0]


def test_camera_height_from_ground_fit(scene):
    depth, K, up = scene
    res = analyze_depth(depth, *K, up_prior=up)
    assert res.ok
    assert abs(res.camera_height - 0.55) < 0.02


def test_boulder_height_and_position(scene):
    depth, K, up = scene
    b = _find(analyze_depth(depth, *K, up_prior=up).features, "bump", 1.0, -0.45)
    assert abs(b.height - 0.20) < 0.03


def test_berm_size_and_volume(scene):
    depth, K, up = scene
    b = _find(analyze_depth(depth, *K, up_prior=up).features, "bump", 1.2, 0.6)
    assert abs(b.height - 0.15) < 0.03
    assert abs(b.width - 0.9) < 0.1
    assert 35 < b.volume_l < 60


def test_crater_found_from_hidden_area(scene):
    depth, K, up = scene
    c = _find(analyze_depth(depth, *K, up_prior=up).features, "dip", 1.78, 0.0)
    assert c.hidden_cells > 0
    assert 0.3 < c.length < 0.6


def test_works_without_imu(scene):
    depth, K, _ = scene
    res = analyze_depth(depth, *K, up_prior=None)
    _find(res.features, "bump", 1.0, -0.45)


def test_wrong_imu_prior_falls_back(scene):
    depth, K, up = scene
    res = analyze_depth(depth, *K, up_prior=-up)          # IMU axes flipped
    assert res.ok and "without the IMU" in res.message


def test_flat_ground_is_clear():
    depth, K, up = synthetic_scene(160, 120, terrain=lambda x, y: np.zeros_like(x))
    res = analyze_depth(depth, *K, up_prior=up, params=TerrainParams(max_range=2.5))
    assert res.features == []
    assert (res.classes == FREE).sum() > 300


def test_occupancy_values(scene):
    depth, K, up = scene
    res = analyze_depth(depth, *K, up_prior=up)
    occ = to_occupancy(res.classes)
    assert set(np.unique(occ)) <= {-1, 0, 100}
    assert np.all(occ[res.classes == BUMP] == 100)
