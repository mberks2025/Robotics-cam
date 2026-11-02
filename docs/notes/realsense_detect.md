# RealSense D435i — Live Object Detection

Python script for YOLO object detection with aligned depth estimates, FPS and RAM monitoring.

## Requirements

- RealSense D435i camera
- `yolo26n.pt` model weights in the working directory
- Python dependencies: `ultralytics`, `psutil`, `pyrealsense2`, `numpy`, `opencv-python`

## Run

```powershell
py -m pip install ultralytics psutil
py realsense_detect.py
```

Press `q` or `Esc` to exit the video window.

## Source code (`realsense_detect.py`)

```python
# realsense_detect.py
# Live object detection on the RealSense D435i.
# Draws a box around each detected object, labels it with its name, confidence and
# distance (from the depth stream), and reports FPS and RAM usage.
#
# Install once:   py -m pip install ultralytics psutil
#                 (pyrealsense2, numpy and opencv-python are already installed from before)
# Run:            py realsense_detect.py
# Quit:           press 'q' or Esc in the video window (Ctrl+C if SHOW_WINDOW is False)

import os
import time

import cv2
import numpy as np
import psutil
import pyrealsense2 as rs

# ---------------- Settings ----------------
MODEL = "yolo26n.pt"        # detection model; "n" = nano (smallest). Try "yolo26s.pt" to compare RAM/speed.
                            # On the Jetson, point this at the exported TensorRT file, e.g. "yolo26n.engine"
WIDTH, HEIGHT = 640, 480    # camera resolution (USB 2 friendly)
FPS = 15                    # camera frame rate (15 is safe on USB 2.1; try 30 on USB 3)
CONF = 0.4                  # ignore detections the model is less than 40% sure about
SHOW_WINDOW = True          # False = no video window, print detections instead (saves RAM on the Jetson)

# ---------------- RAM reporting ----------------
proc = psutil.Process(os.getpid())
peak_system_mb = 0.0


def ram_mb():
    """Return (RAM used by this script, RAM used by the whole system), both in MB."""
    return proc.memory_info().rss / 2**20, psutil.virtual_memory().used / 2**20


def report(stage):
    """Print a RAM snapshot so you can see what each step costs."""
    mine, system = ram_mb()
    print(f"[RAM] {stage:<24} script {mine:7.0f} MB | system used {system:7.0f} MB")


report("start")

# Imported here (not at the top) on purpose, so its RAM cost shows up in the report.
from ultralytics import YOLO  # noqa: E402

report("after importing YOLO")
model = YOLO(MODEL)                       # downloads the model file the first time
report("after loading model")

# ---------------- Camera setup ----------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
profile = pipeline.start(config)

depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()  # raw depth units -> meters
align = rs.align(rs.stream.color)         # line depth pixels up with color pixels
report("after starting camera")


def box_distance(depth_raw, x1, y1, x2, y2):
    """Distance in meters to the object in a box: median depth of the middle of the box.

    Using the middle (not the whole box) avoids background pixels around the object's edges,
    and the median ignores stray bad readings.
    """
    h, w = depth_raw.shape
    x1, x2 = max(0, x1), min(w - 1, x2)
    y1, y2 = max(0, y1), min(h - 1, y2)
    mx, my = int((x2 - x1) * 0.3), int((y2 - y1) * 0.3)      # trim 30% off each side
    patch = depth_raw[y1 + my:y2 - my + 1, x1 + mx:x2 - mx + 1]
    valid = patch[patch > 0]                                  # 0 means "no depth reading"
    if valid.size == 0:
        return None
    return float(np.median(valid)) * depth_scale


WINDOW = "D435i object detection"
fps_smooth = 0.0
frame_count = 0
ram_line = ""
last_print = time.perf_counter()
last_frame = time.perf_counter()

try:
    while True:
        frames = align.process(pipeline.wait_for_frames())
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        if not depth or not color:
            continue

        color_img = np.asanyarray(color.get_data()).copy()   # copy so we can draw on it
        depth_raw = np.asanyarray(depth.get_data())          # raw 16-bit depth, aligned to color

        # ---- Run the detector ----
        t0 = time.perf_counter()
        result = model(color_img, conf=CONF, verbose=False)[0]
        infer_ms = (time.perf_counter() - t0) * 1000

        frame_count += 1
        if frame_count == 1:
            report("after first inference")

        # ---- Read out each detection ----
        found = []
        for box in result.boxes:
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
            name = result.names[int(box.cls[0])]
            conf = float(box.conf[0])
            dist = box_distance(depth_raw, x1, y1, x2, y2)
            dist_txt = f"{dist:.2f} m" if dist else "no depth"
            found.append(f"{name} {dist_txt}")

            if SHOW_WINDOW:
                cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_img, f"{name} {conf:.0%}  {dist_txt}", (x1, max(y1 - 8, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # ---- FPS (smoothed so the number is readable) ----
        now = time.perf_counter()
        inst_fps = 1.0 / max(now - last_frame, 1e-6)
        last_frame = now
        fps_smooth = inst_fps if fps_smooth == 0 else 0.9 * fps_smooth + 0.1 * inst_fps

        # ---- RAM (refresh about once a second) ----
        if frame_count % FPS == 1:
            mine, system = ram_mb()
            peak_system_mb = max(peak_system_mb, system)
            total_gb = psutil.virtual_memory().total / 2**30
            ram_line = f"RAM script {mine:.0f} MB | system {system / 1024:.1f}/{total_gb:.1f} GB"

        if SHOW_WINDOW:
            cv2.putText(color_img, f"{fps_smooth:.1f} FPS | inference {infer_ms:.0f} ms", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(color_img, ram_line, (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow(WINDOW, color_img)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):    # 'q' or Esc quits
                break
        elif now - last_print >= 1.0:                        # headless: one status line per second
            last_print = now
            print(f"{fps_smooth:5.1f} FPS | {infer_ms:4.0f} ms | {ram_line} | {', '.join(found) or 'nothing detected'}")
except KeyboardInterrupt:
    pass
finally:
    pipeline.stop()
    if SHOW_WINDOW:
        cv2.destroyAllWindows()
    report("at exit")
    print(f"[RAM] peak system use while running: {peak_system_mb:.0f} MB")
```
