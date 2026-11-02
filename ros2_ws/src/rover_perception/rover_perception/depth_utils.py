"""Depth helpers shared by the perception nodes (no ROS imports, easy to unit test)."""
import numpy as np

# realsense2_camera publishes depth as 16UC1 in millimetres.
DEPTH_SCALE_M = 0.001


def box_distance(depth_raw, x1, y1, x2, y2, depth_scale=DEPTH_SCALE_M):
    """Distance in metres to the object in a box: median depth of the middle of the box.

    Ported from tools/realsense_detect.py. Trims 30% off each side so background
    pixels at the edges are ignored; the median ignores stray bad readings; 0 means
    "no depth reading". Returns None when the patch has no valid depth.
    """
    h, w = depth_raw.shape[:2]
    x1, x2 = max(0, int(x1)), min(w - 1, int(x2))
    y1, y2 = max(0, int(y1)), min(h - 1, int(y2))
    if x2 <= x1 or y2 <= y1:
        return None
    mx, my = int((x2 - x1) * 0.3), int((y2 - y1) * 0.3)
    patch = depth_raw[y1 + my:y2 - my + 1, x1 + mx:x2 - mx + 1]
    valid = patch[patch > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) * depth_scale


def pixel_to_point(u, v, z, fx, fy, cx, cy):
    """Back-project pixel (u, v) at depth z (m) into the camera optical frame (x right, y down, z forward)."""
    return (u - cx) * z / fx, (v - cy) * z / fy, z
