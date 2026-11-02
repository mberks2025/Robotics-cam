# tools

Standalone scripts that open the camera directly with `pyrealsense2`. They **cannot run while the
ROS camera node is up**; use them for quick checks with nothing else running.

- `realsense_detect.py`: the team's original YOLO test (Oct 5). On the Jetson: activate the venv,
  set `MODEL` to `../models/yolo26n.engine` and `SHOW_WINDOW = False`, then `python3 realsense_detect.py`.
  The ROS version is `rover_perception/yolo_detector.py`.
- `terrain_detect.py`: depth-only terrain detection (ground plane, 5 cm height map, bumps, dips,
  hidden areas, feature size / height / volume / position). Same code as the ROS terrain node
  (`rover_perception/terrain_core.py`). `--sim` runs a simulated scene like the Oct 15 test
  (camera 0.55 m up, tilted 30 deg) with no camera; `--headless` prints instead of showing a window.

Simulated result (640x480, about 60 ms per frame on a desktop CPU):

| Feature | Truth | Measured |
| --- | --- | --- |
| Boulder | 0.20 m high, 0.36 m across, 1.0 m ahead / 0.45 m right | 0.20 m high, 0.30 x 0.35 m, 0.95 m / 0.42 m right (front half only) |
| Berm | 0.90 x 0.40 x 0.15 m, 1.2 m ahead / 0.6 m left | 0.90 x 0.35 x 0.15 m, 1.18 m / 0.60 m left, 46 L |
| Crater | 0.45 m wide, 1.78 m ahead | dip 0.45 x 0.40 m at 1.78 m, mostly hidden cells (far wall out of view) |
