# tools

- `realsense_detect.py`: the team's original standalone YOLO test (Oct 5). It opens the
  camera directly with `pyrealsense2`, so it **cannot run while the ROS camera node is up**.
  Use it only for quick checks with nothing else running. On the Jetson: activate the venv,
  set `MODEL` to `../models/yolo26n.engine` and `SHOW_WINDOW = False`, then `python3 realsense_detect.py`.
  The ROS version is `ros2_ws/src/rover_perception/rover_perception/yolo_detector.py`.
- `terrain_detect.py`: **not in the repo yet.** Add the team's script here; it gets ported to
  `rover_perception/terrain_node.py` (see the plan in that file).
