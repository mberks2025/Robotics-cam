# terrain_detect.py
# Depth-only terrain detection on the RealSense D435i, without ROS.
# Finds the ground plane, builds a 5 cm height map, and reports bumps (boulders, berms),
# dips (craters) and hidden areas, with size, height, volume and position. Prints RAM use.
#
# Uses the same code as the ROS node (ros2_ws/src/rover_perception/rover_perception/terrain_core.py).
# It opens the camera itself, so do NOT run it while the ROS camera node is up.
#
#   python3 tools/terrain_detect.py              # live camera, map window
#   python3 tools/terrain_detect.py --headless   # print only (Jetson over SSH)
#   python3 tools/terrain_detect.py --sim        # simulated scene, no camera needed
#   python3 tools/terrain_detect.py --fps 30 --no-imu
# Quit: 'q' or Esc in the window, Ctrl+C when headless.

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2_ws", "src", "rover_perception"))
from rover_perception.terrain_core import (CLASS_BGR, CLASS_NAMES, TerrainParams, analyze_depth,  # noqa: E402
                                           render_map, synthetic_scene)

try:
    import psutil
    _proc = psutil.Process(os.getpid())

    def ram_mb():
        return _proc.memory_info().rss / 2**20
except ImportError:
    def ram_mb():
        return float("nan")


def parse():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sim", action="store_true", help="simulated scene (0.55 m, 30 deg tilt), no camera")
    ap.add_argument("--headless", action="store_true", help="no window, print features once a second")
    ap.add_argument("--fps", type=int, default=15, help="depth fps (15 is safe on USB 2.1)")
    ap.add_argument("--no-imu", action="store_true", help="fit the ground without the IMU gravity prior")
    ap.add_argument("--step-up", type=float, default=0.08, help="m above ground that counts as a bump")
    ap.add_argument("--step-down", type=float, default=0.08, help="m below ground that counts as a dip")
    ap.add_argument("--max-range", type=float, default=3.0)
    return ap.parse_args()


def show(res, params, cv2, headless, extra=""):
    feats = ", ".join(f"{f.kind} {f.x:.2f}m fwd {f.y:+.2f}m left {f.length:.2f}x{f.width:.2f}m "
                      f"h{f.height:.2f} {f.volume_l:.0f}L" for f in res.features) or "clear"
    line = f"cam {res.camera_height:.2f} m | {feats} {extra}"
    if headless or cv2 is None:
        print(line)
        return True
    img = np.ascontiguousarray(render_map(res.classes, scale=5))
    for k, (c, bgr) in enumerate(CLASS_BGR.items()):
        cv2.putText(img, CLASS_NAMES[c], (5, 15 + 15 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1)
    cv2.imshow("terrain (forward is up)", img)
    return (cv2.waitKey(1) & 0xFF) not in (ord("q"), 27)


def main():
    a = parse()
    params = TerrainParams(step_up=a.step_up, step_down=a.step_down, max_range=a.max_range)
    cv2 = None
    if not a.headless:
        try:
            import cv2  # noqa: F811
        except ImportError:
            print("opencv not installed: running headless")
    print(f"[RAM] start {ram_mb():.0f} MB")

    if a.sim:
        depth, K, up = synthetic_scene(640, 480)
        t0 = time.perf_counter()
        res = analyze_depth(depth, *K, up_prior=None if a.no_imu else up, params=params)
        print(f"processed in {(time.perf_counter() - t0) * 1000:.0f} ms, RAM {ram_mb():.0f} MB")
        print("truth: boulder 0.20 m high at 1.0 m / -0.45 m, berm 0.90 x 0.40 x 0.15 m at 1.2 m / +0.6 m, "
              "crater 0.45 m wide at 1.78 m / 0")
        for f in res.features:
            print(f.as_dict())
        if cv2 is not None and not a.headless:
            show(res, params, cv2, False)
            cv2.waitKey(0)
        return

    import pyrealsense2 as rs
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, a.fps)
    if not a.no_imu:
        cfg.enable_stream(rs.stream.accel)
    profile = pipe.start(cfg)
    sensor = profile.get_device().first_depth_sensor()
    if sensor.supports(rs.option.emitter_enabled):
        sensor.set_option(rs.option.emitter_enabled, 1)        # emitter on: better depth on sand
    scale = sensor.get_depth_scale()
    intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    K = (intr.fx, intr.fy, intr.ppx, intr.ppy)
    spatial, temporal = rs.spatial_filter(), rs.temporal_filter()
    accel = None
    print(f"[RAM] camera started {ram_mb():.0f} MB")

    last_print, peak = 0.0, 0.0
    try:
        while True:
            frames = pipe.wait_for_frames()
            if not a.no_imu:
                m = frames.first_or_default(rs.stream.accel)
                if m:
                    v = m.as_motion_frame().get_motion_data()
                    a_now = np.array([v.x, v.y, v.z])
                    accel = a_now if accel is None else 0.9 * accel + 0.1 * a_now
            dframe = frames.get_depth_frame()
            if not dframe:
                continue
            dframe = temporal.process(spatial.process(dframe))
            depth = np.asanyarray(dframe.get_data()).astype(np.float32) * scale
            # D435i accel reads +g "up" in the depth optical axes (x right, y down, z forward).
            up = None if accel is None else accel / np.linalg.norm(accel)
            t0 = time.perf_counter()
            res = analyze_depth(depth, *K, up_prior=up, params=params)
            ms = (time.perf_counter() - t0) * 1000
            peak = max(peak, ram_mb())
            if not res.ok:
                continue
            now = time.monotonic()
            if a.headless and now - last_print < 1.0:
                continue
            last_print = now
            if not show(res, params, cv2, a.headless, f"| {ms:.0f} ms | RAM {ram_mb():.0f} MB {res.message}"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        if cv2 is not None and not a.headless:
            cv2.destroyAllWindows()
        print(f"[RAM] peak script RAM {peak:.0f} MB")


if __name__ == "__main__":
    main()
