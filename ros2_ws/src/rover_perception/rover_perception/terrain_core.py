"""Depth-only terrain detection core (numpy only, no ROS, easy to test).

Pipeline for one depth frame:
  1. Back-project depth pixels to 3D points in the camera optical frame (x right, y down, z forward).
  2. Find the ground plane with RANSAC, using the IMU gravity vector (if given) as the "up" prior.
  3. Express points in a ground frame: origin on the ground below the camera, x forward along
     the ground, y left, z up. Height = distance above the fitted ground plane.
  4. Bin into a 5 cm height map (per cell: max, min, median-ish mean, hit count).
  5. Classify cells: bump (above step_up), dip (below -step_down), steep (local slope), free,
     hidden (unseen but inside the camera's view, behind something it did see), unknown.
  6. Group bump / dip+hidden cells into features with size, height or depth, volume and position.

Thresholds come from the 2026 guidebook obstacles (boulders 30-40 cm, craters up to 40-50 cm);
set step_up / step_down / max_slope_deg to the rover's real limits once they are known.
"""
from dataclasses import dataclass, field

import numpy as np

# Cell classes (also the order of the legend colours)
UNKNOWN, FREE, BUMP, DIP, STEEP, HIDDEN = 0, 1, 2, 3, 4, 5
CLASS_NAMES = {UNKNOWN: "unknown", FREE: "free", BUMP: "bump", DIP: "dip", STEEP: "steep", HIDDEN: "hidden"}
# BGR colours for the debug image: red bumps, blue dips, yellow hidden (the Oct 15 legend)
CLASS_BGR = {
    UNKNOWN: (40, 40, 40), FREE: (90, 160, 90), BUMP: (40, 40, 230),
    DIP: (230, 120, 40), STEEP: (40, 140, 255), HIDDEN: (40, 220, 230),
}


@dataclass
class TerrainParams:
    resolution: float = 0.05        # m per cell (5 cm, as in the Oct 15 script)
    max_range: float = 3.0          # m forward; D435i depth is trustworthy to about 2.5 m
    min_range: float = 0.3          # m; the D435i sees nothing closer
    width: float = 3.0              # m, grid spans +-width/2 left/right
    step_up: float = 0.08           # m above ground that counts as a bump
    step_down: float = 0.08         # m below ground that counts as a dip
    max_slope_deg: float = 25.0     # local slope that counts as steep
    min_hits: int = 2               # points a cell needs before it counts as observed
    min_feature_cells: int = 3      # smaller blobs are treated as noise
    pixel_step: int = 2             # use every Nth pixel in u and v (speed vs detail)
    ransac_iters: int = 120
    ransac_tol: float = 0.02        # m inlier distance for the ground fit
    max_tilt_from_imu_deg: float = 15.0   # ground normal must be within this of IMU "up"
    seed: int = 0


@dataclass
class Feature:
    kind: str                       # "bump" or "dip"
    cells: int
    x: float                        # m forward of the camera, centroid on the ground
    y: float                        # m left of the camera
    length: float                   # m along x
    width: float                    # m along y
    height: float                   # m: max height (bump) or max observed depth (dip, positive)
    volume_l: float                 # litres above (bump) or below (dip) ground, observed part only
    hidden_cells: int = 0           # cells of the feature the camera could not see

    def as_dict(self):
        return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in self.__dict__.items()}


@dataclass
class TerrainResult:
    classes: np.ndarray             # (nx, ny) uint8 cell classes; row i = x forward, col j = y (left is high j)
    height: np.ndarray              # (nx, ny) float32 mean height per cell, NaN where unobserved
    hmax: np.ndarray
    hmin: np.ndarray
    hits: np.ndarray
    features: list = field(default_factory=list)
    camera_height: float = float("nan")
    ground_normal_opt: np.ndarray = None     # unit "up" in the camera optical frame
    R_ground_from_opt: np.ndarray = None     # rotation optical -> ground frame
    hidden_below: np.ndarray = None          # hidden cells where the ground is lower than expected (in a hole)
    shadow: np.ndarray = None                # hidden cells blocked by something in front
    gap_free: np.ndarray = None              # unobserved cells whose depth matches flat ground
    points_ground: np.ndarray = None         # (N, 3) points: x fwd, y left (relative to camera), z = height
    ok: bool = True
    message: str = ""

    def grid_origin(self, params):
        """(x, y) of the grid's cell (0, 0) corner in the ground frame."""
        return 0.0, -params.width / 2


# ---------------------------------------------------------------- geometry

def depth_to_points(depth_m, fx, fy, cx, cy, step=1):
    """Depth image in metres -> (N, 3) points in the optical frame, zeros dropped."""
    h, w = depth_m.shape
    v, u = np.mgrid[0:h:step, 0:w:step]
    z = depth_m[::step, ::step]
    ok = np.isfinite(z) & (z > 0)
    z, u, v = z[ok], u[ok], v[ok]
    return np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)


def fit_ground(points, up_prior=None, params=TerrainParams()):
    """RANSAC plane fit. Returns (unit normal pointing up, d) with normal . p + d = 0 on the ground,
    so normal . p + d is the height of p above the ground. None if no plane is found.

    up_prior: unit "up" vector in the optical frame (from IMU gravity). Without it, any plane whose
    normal is within 45 deg of the optical -y axis (camera roughly upright) is accepted.
    """
    if len(points) < 50:
        return None
    rng = np.random.default_rng(params.seed)
    prior = np.array([0.0, -1.0, 0.0]) if up_prior is None else np.asarray(up_prior, float) / np.linalg.norm(up_prior)
    max_angle = params.max_tilt_from_imu_deg if up_prior is not None else 45.0
    cos_max = np.cos(np.radians(max_angle))

    # Ground is usually the lower part of the scene: sample candidates from the lowest 60 % along the prior.
    along = points @ prior
    cand = points[along <= np.quantile(along, 0.6)]
    if len(cand) < 3:
        cand = points
    best, best_count = None, 0
    for _ in range(params.ransac_iters):
        p = cand[rng.choice(len(cand), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n @ prior < 0:
            n = -n
        if n @ prior < cos_max:
            continue
        d = -n @ p[0]
        if d <= 0.05:                     # camera must sit above the ground (also rejects a flipped "up")
            continue
        count = int(np.count_nonzero(np.abs(points @ n + d) < params.ransac_tol))
        if count > best_count:
            best, best_count = (n, d), count
    if best is None:
        return None
    # Refine with least squares on the inliers.
    n, d = best
    inl = points[np.abs(points @ n + d) < params.ransac_tol]
    if len(inl) >= 3:
        c = inl.mean(axis=0)
        _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
        n2 = vt[2] / np.linalg.norm(vt[2])
        if n2 @ prior < 0:
            n2 = -n2
        if n2 @ prior >= cos_max and -n2 @ c > 0.05:
            n, d = n2, -n2 @ c
    return n, d


def ground_frame(n, d):
    """Rotation optical -> ground frame (x forward along ground, y left, z up) and camera height."""
    z_axis = n
    fwd = np.array([0.0, 0.0, 1.0])                 # optical z = camera forward
    x_axis = fwd - (fwd @ z_axis) * z_axis
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)               # left
    R = np.stack([x_axis, y_axis, z_axis])          # rows: ground axes expressed in optical
    cam_height = d                                  # height of the camera origin above ground = n.0 + d
    return R, cam_height


# ---------------------------------------------------------------- main entry

def analyze_depth(depth_m, fx, fy, cx, cy, up_prior=None, params=TerrainParams()):
    """Run the whole pipeline on one depth frame (metres). Returns TerrainResult."""
    pts = depth_to_points(depth_m, fx, fy, cx, cy, params.pixel_step)
    rng_ = np.linalg.norm(pts, axis=1)
    pts = pts[(rng_ > params.min_range) & (pts[:, 2] < params.max_range * 1.5)]
    nx = int(round(params.max_range / params.resolution))
    ny = int(round(params.width / params.resolution))
    empty = np.full((nx, ny), np.nan, np.float32)

    plane = fit_ground(pts, up_prior, params)
    used_prior = up_prior is not None
    if plane is None and up_prior is not None:
        plane = fit_ground(pts, None, params)     # IMU prior wrong (bad axes or moving fast)? try without
        used_prior = False
    if plane is None:
        return TerrainResult(np.zeros((nx, ny), np.uint8), empty, empty.copy(), empty.copy(),
                             np.zeros((nx, ny), np.int32), ok=False, message="no ground plane found")
    n, d = plane
    R, cam_h = ground_frame(n, d)
    g = pts @ R.T                                    # points in ground axes, origin at the camera
    g[:, 2] += cam_h                                 # heights above ground

    hmax, hmin, hsum, hits = _bin(g, params, nx, ny)
    with np.errstate(invalid="ignore", divide="ignore"):
        hmean = np.where(hits > 0, hsum / np.maximum(hits, 1), np.nan).astype(np.float32)
    observed = hits >= params.min_hits

    classes = classify(hmean, hmax, hmin, observed, params)
    shadow, below, gap_free = _check_unobserved(observed, params, nx, ny, depth_m, (fx, fy, cx, cy), R, cam_h)
    classes[gap_free] = FREE
    classes[shadow | below] = HIDDEN

    res = TerrainResult(classes, hmean, _nanify(hmax, observed), _nanify(hmin, observed), hits,
                        camera_height=float(cam_h), ground_normal_opt=n, R_ground_from_opt=R,
                        hidden_below=below, shadow=shadow, gap_free=gap_free, points_ground=g)
    res.features = extract_features(res, params)
    if up_prior is not None and not used_prior:
        res.message = "ground found without the IMU prior (check IMU axes)"
    return res


def classify(hmean, hmax, hmin, observed, params):
    """Observed cells -> FREE / STEEP / DIP / BUMP; the rest UNKNOWN."""
    classes = np.full(observed.shape, UNKNOWN, np.uint8)
    classes[observed] = FREE
    classes[_steep_cells(hmean, observed, params)] = STEEP
    with np.errstate(invalid="ignore"):
        classes[observed & (hmin < -params.step_down)] = DIP
        classes[observed & (hmax > params.step_up)] = BUMP
    return classes


def _nanify(a, mask):
    out = a.astype(np.float32).copy()
    out[~mask] = np.nan
    return out


def _bin(g, params, nx, ny):
    i = np.floor(g[:, 0] / params.resolution).astype(np.int64)
    j = np.floor((g[:, 1] + params.width / 2) / params.resolution).astype(np.int64)
    ok = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
    i, j, h = i[ok], j[ok], g[ok, 2]
    flat = i * ny + j
    size = nx * ny
    hits = np.bincount(flat, minlength=size)
    hsum = np.bincount(flat, weights=h, minlength=size)
    hmax = np.full(size, -np.inf)
    hmin = np.full(size, np.inf)
    np.maximum.at(hmax, flat, h)
    np.minimum.at(hmin, flat, h)
    shape = (nx, ny)
    return hmax.reshape(shape), hmin.reshape(shape), hsum.reshape(shape), hits.reshape(shape)


def _steep_cells(hmean, observed, params):
    """Slope from height differences to 4-neighbours, both cells observed."""
    tan_max = np.tan(np.radians(params.max_slope_deg))
    steep = np.zeros_like(observed)
    r = params.resolution
    for axis in (0, 1):
        dh = np.abs(np.diff(hmean, axis=axis)) / r
        both = np.logical_and(np.take(observed, range(observed.shape[axis] - 1), axis=axis),
                              np.take(observed, range(1, observed.shape[axis]), axis=axis))
        s = both & (dh > tan_max)
        if axis == 0:
            steep[:-1, :] |= s
            steep[1:, :] |= s
        else:
            steep[:, :-1] |= s
            steep[:, 1:] |= s
    return steep & observed


def _check_unobserved(observed, params, nx, ny, depth_m, K, R, cam_h):
    """Look up each unobserved cell's ground point in the depth image.

    Projects the point (x, y, height 0) into the image and compares the measured depth there
    with the depth the flat ground would give:
      - measured much SHORTER: something in front blocks the view -> shadow (hidden).
      - measured much LONGER: the ray went past the ground level -> the ground is lower here,
        i.e. the unseen inside of a hole or crater (hidden, counted with dips).
      - about the same: flat ground that was just sparsely sampled -> free.
      - no depth / outside the image: stays unknown.
    Returns (shadow, below, gap_free) boolean grids.
    """
    fx, fy, cx, cy = K
    h_img, w_img = depth_m.shape
    r = params.resolution
    xs = (np.arange(nx) + 0.5) * r
    ys = (np.arange(ny) + 0.5) * r - params.width / 2
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    g = np.stack([X, Y, np.full_like(X, -cam_h)], axis=-1)       # relative to the camera, ground axes
    p = g @ R                                                    # -> optical frame (R rows = ground axes)
    z_exp = p[..., 2]
    with np.errstate(invalid="ignore", divide="ignore"):
        u = np.round(fx * p[..., 0] / z_exp + cx)
        v = np.round(fy * p[..., 1] / z_exp + cy)
    inside = (z_exp > params.min_range) & (u >= 0) & (u < w_img) & (v >= 0) & (v < h_img) & ~observed
    meas = np.zeros_like(z_exp)
    ui, vi = u[inside].astype(int), v[inside].astype(int)
    meas[inside] = depth_m[vi, ui]
    valid = inside & np.isfinite(meas) & (meas > 0)
    tol = 0.04 + 0.03 * z_exp                                    # depth noise grows with distance
    shadow = valid & (meas < z_exp - tol)
    below = valid & (meas > z_exp + tol)
    gap_free = valid & ~shadow & ~below
    return shadow, below, gap_free


# ---------------------------------------------------------------- features

def _components(mask, reach=2):
    """Connected components of a boolean grid, joining cells up to `reach` cells apart (so a
    crater's measured floor and its unseen part, split by a sliver of rim, stay one feature).
    Returns list of (rows, cols) index arrays."""
    offsets = [(a, b) for a in range(-reach, reach + 1) for b in range(-reach, reach + 1) if (a, b) != (0, 0)]
    seen = np.zeros_like(mask, bool)
    out = []
    nx, ny = mask.shape
    for si, sj in zip(*np.nonzero(mask)):
        if seen[si, sj]:
            continue
        stack, cells = [(si, sj)], []
        seen[si, sj] = True
        while stack:
            i, j = stack.pop()
            cells.append((i, j))
            for da, db in offsets:
                a, b = i + da, j + db
                if 0 <= a < nx and 0 <= b < ny and mask[a, b] and not seen[a, b]:
                    seen[a, b] = True
                    stack.append((a, b))
        c = np.array(cells)
        out.append((c[:, 0], c[:, 1]))
    return out


def extract_features(res, params, origin=None):
    """Group hazard cells into features. origin = (x, y) of cell (0, 0)'s corner in the grid's
    frame; default is the single-frame grid (camera ground frame)."""
    x0, y0 = origin if origin is not None else (0.0, -params.width / 2)
    r = params.resolution
    area = r * r
    feats = []
    below = res.hidden_below if res.hidden_below is not None else np.zeros_like(res.classes, bool)
    groups = (("bump", res.classes == BUMP), ("dip", (res.classes == DIP) | below))
    for kind, mask in groups:
        for rows, cols in _components(mask):
            if len(rows) < params.min_feature_cells:
                continue
            hidden_n = int(np.count_nonzero(res.classes[rows, cols] == HIDDEN))
            xs = x0 + (rows + 0.5) * r
            ys = y0 + (cols + 0.5) * r
            if kind == "bump":
                h = res.hmax[rows, cols]
                height = float(np.nanmax(h))
                volume = float(np.nansum(np.clip(h, 0, None)) * area * 1000)
            else:
                h = res.hmin[rows, cols]
                height = float(-np.nanmin(h)) if np.any(np.isfinite(h)) else float("nan")
                volume = float(np.nansum(np.clip(-h, 0, None)) * area * 1000)
            feats.append(Feature(
                kind=kind, cells=int(len(rows)),
                x=float(xs.mean()), y=float(ys.mean()),
                length=float(np.ptp(xs) + r), width=float(np.ptp(ys) + r),
                height=height, volume_l=volume, hidden_cells=hidden_n))
    feats.sort(key=lambda f: f.x)
    return feats


def _touches(classes, rows, cols, cls):
    nx, ny = classes.shape
    for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        a, b = rows + di, cols + dj
        ok = (a >= 0) & (a < nx) & (b >= 0) & (b < ny)
        if np.any(classes[a[ok], b[ok]] == cls):
            return True
    return False


# ---------------------------------------------------------------- output helpers

def to_occupancy(classes):
    """Nav2-style occupancy values: -1 unknown, 0 free, 100 lethal (bump, dip, steep, hidden)."""
    occ = np.full(classes.shape, -1, np.int8)
    occ[classes == FREE] = 0
    occ[np.isin(classes, (BUMP, DIP, STEEP, HIDDEN))] = 100
    return occ


def render_map(classes, scale=6):
    """Top-down BGR image of the class grid: forward is up, left is left."""
    img = np.zeros(classes.shape + (3,), np.uint8)
    for c, bgr in CLASS_BGR.items():
        img[classes == c] = bgr
    img = img[::-1, ::-1]                       # row 0 (near) at the bottom, +y (left) on the left
    return np.repeat(np.repeat(img, scale, axis=0), scale, axis=1)


# ---------------------------------------------------------------- synthetic scene (for tests)

def synthetic_scene(width=320, height=240, hfov_deg=87.0, cam_height=0.55, tilt_deg=30.0,
                    terrain=None, max_dist=6.0, step=0.005, cam_x=0.0, cam_y=0.0, yaw_deg=0.0):
    """Ray-cast a depth image (metres) of a heightfield, like the Oct 15 simulated test.

    terrain(x, y) -> ground height (m), x forward, y left, in the world frame. The camera sits at
    (cam_x, cam_y, cam_height) facing yaw_deg (0 = +x); depth is in the camera's own frame.
    Returns depth_m, (fx, fy, cx, cy), up vector in the optical frame (what the IMU would give).
    """
    if terrain is None:
        terrain = default_terrain
    fx = (width / 2) / np.tan(np.radians(hfov_deg / 2))
    fy, cx, cy = fx, (width - 1) / 2, (height - 1) / 2
    v, u = np.mgrid[0:height, 0:width]
    d_opt = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], axis=-1)
    d_opt /= np.linalg.norm(d_opt, axis=-1, keepdims=True)
    t = np.radians(tilt_deg)
    # optical -> ground: optical z (forward) pitched down by tilt; optical x right = ground -y; optical y down.
    R = np.array([[0.0, -np.sin(t), np.cos(t)],
                  [-1.0, 0.0, 0.0],
                  [0.0, -np.cos(t), -np.sin(t)]])
    c, s_ = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    Rz = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    d_g = d_opt @ (Rz @ R).T
    origin = np.array([cam_x, cam_y, cam_height])
    depth = np.zeros((height, width))
    alive = np.ones((height, width), bool)
    for s in np.arange(0.1, max_dist, step):
        p = origin + s * d_g
        hit = alive & (p[..., 2] <= terrain(p[..., 0], p[..., 1]))
        depth[hit] = s * d_opt[..., 2][hit]          # depth image stores z, not range
        alive &= ~hit
        if not alive.any():
            break
    up_opt = R.T @ np.array([0.0, 0.0, 1.0])
    return depth, (fx, fy, cx, cy), up_opt


def default_terrain(x, y):
    """Berm 0.90 x 0.40 x 0.15 m, boulder 0.20 m high, crater 0.45 m wide at 1.78 m ahead."""
    z = np.zeros_like(x, float)
    # berm: flat-topped ridge, 0.9 m along y, 0.4 m along x, centred 1.2 m ahead, 0.6 m left
    bx, by = np.abs(x - 1.2) / 0.20, np.abs(y - 0.6) / 0.45
    z = np.maximum(z, np.where((bx < 1) & (by < 1), 0.15 * np.clip((1 - bx) * 3, 0, 1), 0))
    # boulder: half-ellipsoid, 0.36 m across, 0.20 m high, 1.0 m ahead, 0.45 m right
    rb = ((x - 1.0) ** 2 + (y + 0.45) ** 2) / 0.18 ** 2
    z = np.maximum(z, np.where(rb < 1, 0.20 * np.sqrt(np.clip(1 - rb, 0, 1)), 0))
    # crater: bowl 0.45 m wide, 0.25 m deep, centred 1.78 m ahead on the centre line
    rc = ((x - 1.78) ** 2 + y ** 2) / 0.225 ** 2
    z = np.where(rc < 1, -0.25 * np.sqrt(np.clip(1 - rc, 0, 1)), z)
    return z


# ---------------------------------------------------------------- rolling map (several frames)

def matrix_from_quat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def ground_pose_in_odom(R_odom_opt, t_odom_opt, res):
    """(x, y, yaw) in odom of the ground frame below the camera (TerrainResult from one frame)."""
    origin = R_odom_opt @ (-res.camera_height * res.ground_normal_opt) + t_odom_opt
    x_axis = R_odom_opt @ res.R_ground_from_opt[0]
    return float(origin[0]), float(origin[1]), float(np.arctan2(x_axis[1], x_axis[0]))


class RollingGrid:
    """Robot-centred 5 cm grid in a fixed frame (odom) that fuses single-frame results.

    Each frame's points are placed with the frame's pose in odom (x, y, heading of the ground
    frame below the camera). Heights stay relative to the ground fitted in that frame, so the map
    needs no global elevation. Per cell, the per-frame max / min / mean height are blended with an
    exponential average (alpha), which smooths single-frame noise; a cell seen again overwrites
    "hidden" flags. Cells not seen for forget_s seconds are cleared. The grid recentres on the robot
    when it moves more than recentre_m from the grid centre.
    """

    def __init__(self, params=TerrainParams(), size_m=6.0, alpha=0.4, forget_s=30.0, recentre_m=1.0):
        self.p = params
        self.n = int(round(size_m / params.resolution))
        self.alpha, self.forget_s, self.recentre_m = alpha, forget_s, recentre_m
        self.origin = None          # (x, y) of cell (0, 0)'s corner, in odom
        self.reset()

    def reset(self):
        n = self.n
        self.hmax = np.full((n, n), np.nan, np.float32)
        self.hmin = np.full((n, n), np.nan, np.float32)
        self.hmean = np.full((n, n), np.nan, np.float32)
        self.nobs = np.zeros((n, n), np.int32)
        self.last = np.full((n, n), -np.inf)
        self.below = np.zeros((n, n), bool)
        self.shadow = np.zeros((n, n), bool)
        self.gap_free = np.zeros((n, n), bool)

    # -- grid placement
    def _centre_on(self, x, y):
        r = self.p.resolution
        half = self.n * r / 2
        new = (np.floor((x - half) / r) * r, np.floor((y - half) / r) * r)
        if self.origin is None:
            self.origin = new
            return
        di = int(round((new[0] - self.origin[0]) / r))
        dj = int(round((new[1] - self.origin[1]) / r))
        if di == 0 and dj == 0:
            return
        fills = {"hmax": np.nan, "hmin": np.nan, "hmean": np.nan, "nobs": 0, "last": -np.inf,
                 "below": False, "shadow": False, "gap_free": False}
        for name, fill in fills.items():
            setattr(self, name, _shift(getattr(self, name), di, dj, fill))
        self.origin = (self.origin[0] + di * r, self.origin[1] + dj * r)

    def _cells(self, x, y):
        r = self.p.resolution
        i = np.floor((x - self.origin[0]) / r).astype(np.int64)
        j = np.floor((y - self.origin[1]) / r).astype(np.int64)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        return i, j, ok

    # -- update
    def update(self, res, pose_xy_yaw, stamp):
        """Fuse one TerrainResult. pose_xy_yaw = (x, y, yaw) of the camera's ground frame in odom."""
        if not res.ok:
            return
        x, y, yaw = pose_xy_yaw
        if self.origin is None:
            self._centre_on(x, y)
        else:
            cx = self.origin[0] + self.n * self.p.resolution / 2
            cy = self.origin[1] + self.n * self.p.resolution / 2
            if np.hypot(x - cx, y - cy) > self.recentre_m:
                self._centre_on(x, y)
        c, s = np.cos(yaw), np.sin(yaw)

        def to_odom(gx, gy):
            return x + c * gx - s * gy, y + s * gx + c * gy

        # Observed cells: bin this frame's points directly in odom.
        g = res.points_ground
        ox, oy = to_odom(g[:, 0], g[:, 1])
        fwd = (g[:, 0] >= 0) & (g[:, 0] < self.p.max_range) & (np.abs(g[:, 1]) < self.p.width / 2)
        i, j, ok = self._cells(ox, oy)
        ok &= fwd
        i, j, h = i[ok], j[ok], g[ok, 2]
        n = self.n
        flat = i * n + j
        hits = np.bincount(flat, minlength=n * n).reshape(n, n)
        hsum = np.bincount(flat, weights=h, minlength=n * n).reshape(n, n)
        fmax = np.full(n * n, -np.inf)
        fmin = np.full(n * n, np.inf)
        np.maximum.at(fmax, flat, h)
        np.minimum.at(fmin, flat, h)
        fmax, fmin = fmax.reshape(n, n), fmin.reshape(n, n)
        seen = hits >= self.p.min_hits
        fmean = np.where(seen, hsum / np.maximum(hits, 1), np.nan)

        a = self.alpha
        first = seen & (self.nobs == 0)
        again = seen & (self.nobs > 0)
        for arr, new in ((self.hmax, fmax), (self.hmin, fmin), (self.hmean, fmean)):
            arr[first] = new[first]
            arr[again] = (1 - a) * arr[again] + a * new[again]
        self.nobs[seen] += 1
        self.last[seen] = stamp
        self.below[seen] = self.shadow[seen] = self.gap_free[seen] = False

        # Unobserved cells of this frame: carry the hidden / gap flags over by cell centre.
        r = self.p.resolution
        for flag, arr in ((res.hidden_below, self.below), (res.shadow, self.shadow), (res.gap_free, self.gap_free)):
            if flag is None:
                continue
            fi, fj = np.nonzero(flag)
            gx = (fi + 0.5) * r
            gy = (fj + 0.5) * r - self.p.width / 2
            ci, cj, cok = self._cells(*to_odom(gx, gy))
            ci, cj = ci[cok], cj[cok]
            fresh = self.nobs[ci, cj] == 0
            arr[ci[fresh], cj[fresh]] = True
            self.last[ci[fresh], cj[fresh]] = stamp

        # Forget stale cells.
        stale = (stamp - self.last) > self.forget_s
        if stale.any():
            for arr in (self.hmax, self.hmin, self.hmean):
                arr[stale] = np.nan
            self.nobs[stale] = 0
            self.below[stale] = self.shadow[stale] = self.gap_free[stale] = False
            self.last[stale] = -np.inf

    # -- outputs
    def classes(self):
        observed = self.nobs > 0
        cls = classify(self.hmean, self.hmax, self.hmin, observed, self.p)
        cls[self.gap_free & ~observed] = FREE
        cls[(self.below | self.shadow) & ~observed] = HIDDEN
        return cls

    def result(self):
        """A TerrainResult view of the map (features positioned in odom)."""
        cls = self.classes()
        res = TerrainResult(cls, self.hmean.copy(), self.hmax.copy(), self.hmin.copy(), self.nobs.copy(),
                            hidden_below=self.below & (self.nobs == 0))
        res.features = extract_features(res, self.p, origin=self.origin)
        return res


def _shift(a, di, dj, fill):
    """Move grid contents so cell (i, j) of the old grid lands on (i - di, j - dj); new cells = fill."""
    out = np.full_like(a, fill)
    n0, n1 = a.shape
    src_i = slice(max(di, 0), n0 + min(di, 0))
    dst_i = slice(max(-di, 0), n0 + min(-di, 0))
    src_j = slice(max(dj, 0), n1 + min(dj, 0))
    dst_j = slice(max(-dj, 0), n1 + min(-dj, 0))
    if abs(di) < n0 and abs(dj) < n1:
        out[dst_i, dst_j] = a[src_i, src_j]
    return out
