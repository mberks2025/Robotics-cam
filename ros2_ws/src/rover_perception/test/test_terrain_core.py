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


# ---------------------------------------------------------------- rolling map

from rover_perception.terrain_core import RollingGrid, ground_pose_in_odom  # noqa: E402


def _optical_to_world(cam_x, cam_y, yaw_deg, tilt_deg=30.0, cam_h=0.55):
    t, y = np.radians(tilt_deg), np.radians(yaw_deg)
    R = np.array([[0.0, -np.sin(t), np.cos(t)], [-1.0, 0.0, 0.0], [0.0, -np.cos(t), -np.sin(t)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ R, np.array([cam_x, cam_y, cam_h])


def test_ground_pose_from_tf():
    depth, K, up = synthetic_scene(160, 120, cam_x=0.4, cam_y=-0.2, yaw_deg=20)
    res = analyze_depth(depth, *K, up_prior=up)
    R, t = _optical_to_world(0.4, -0.2, 20)
    x, y, yaw = ground_pose_in_odom(R, t, res)
    assert abs(x - 0.4) < 0.02 and abs(y + 0.2) < 0.02 and abs(np.degrees(yaw) - 20) < 1.5


@pytest.fixture(scope="module")
def drive():
    """Drive forward 0.9 m in four steps, then turn 25 deg left."""
    rg = RollingGrid()
    poses = [(0.0, 0.0, 0.0), (0.3, 0.0, 0.0), (0.6, 0.0, 0.0), (0.9, 0.0, 0.0), (0.9, 0.0, 25.0)]
    for k, (x, y, yaw) in enumerate(poses):
        depth, K, up = synthetic_scene(240, 180, cam_x=x, cam_y=y, yaw_deg=yaw)
        res = analyze_depth(depth, *K, up_prior=up)
        R, t = _optical_to_world(x, y, yaw)
        rg.update(res, ground_pose_in_odom(R, t, res), float(k))
    return rg


def test_rolling_map_keeps_hazards_in_place(drive):
    feats = drive.result().features
    b = _find(feats, "bump", 1.0, -0.45)
    assert abs(b.height - 0.20) < 0.03
    _find(feats, "bump", 1.2, 0.6)
    _find(feats, "dip", 1.78, 0.0)


def test_rolling_map_sees_further_into_crater(drive):
    c = _find(drive.result().features, "dip", 1.78, 0.0)
    assert c.height > 0.18                       # true depth 0.25 m; one frame from 1.8 m away sees ~0.14


def test_forget_and_recentre():
    rg = RollingGrid(forget_s=5.0, recentre_m=0.5)
    depth, K, up = synthetic_scene(160, 120)
    res = analyze_depth(depth, *K, up_prior=up)
    rg.update(res, (0.0, 0.0, 0.0), 0.0)
    seen = int((rg.nobs > 0).sum())
    rg.update(res, (2.0, 0.0, 0.0), 1.0)         # pretend we jumped 2 m: grid recentres, old cells shift
    assert rg.origin[0] == pytest.approx(-1.0)      # grid now centred on x = 2 m (6 m map)
    rg.update(res, (2.0, 0.0, 0.0), 20.0)        # cells from t=0 and t=1 are stale now
    assert int((rg.nobs > 0).sum()) <= seen
