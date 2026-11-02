import numpy as np

from rover_perception.depth_utils import box_distance, pixel_to_point


def test_median_of_box_centre_in_metres():
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[100:200, 100:200] = 1500          # 1.5 m object
    depth[100:110, 100:200] = 9000          # background bleeding in at the top edge
    assert abs(box_distance(depth, 100, 100, 200, 200) - 1.5) < 1e-6


def test_no_depth_returns_none():
    assert box_distance(np.zeros((480, 640), dtype=np.uint16), 10, 10, 50, 50) is None


def test_box_clipped_to_image():
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    assert abs(box_distance(depth, -20, -20, 700, 500) - 2.0) < 1e-6


def test_degenerate_box():
    assert box_distance(np.full((10, 10), 1000, dtype=np.uint16), 5, 5, 5, 5) is None


def test_pixel_to_point_centre_is_on_axis():
    assert pixel_to_point(320, 240, 2.0, 600, 600, 320, 240) == (0.0, 0.0, 2.0)
