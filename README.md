# Robotics-cam: D435i vision stack on Jetson Orin Nano

Bring-up kit for the rover's Intel RealSense D435i on a Jetson Orin Nano 8 GB:
one ROS 2 camera node for color, depth, stereo IR and IMU, plus a YOLO26n TensorRT detector.
Obstacles come from **depth** (anything that isn't open ground); YOLO is only for things that
need a name (people, targets).

| Item | Version |
| --- | --- |
| Board | Jetson Orin Nano 8 GB dev kit |
| OS | JetPack 7.2 (Ubuntu 24.04, CUDA 13.2, TensorRT 10.x) |
| ROS 2 | Jazzy |
| SDK | librealsense v2.58.4, source build, RSUSB backend + CUDA |
| ROS wrapper | realsense-ros v4.58.4, source build |
| Detector | YOLO26n, TensorRT FP16 engine at 640 px |

## Quick start

Clone into your home folder (the launch defaults expect `~/Robotics-cam`):

```bash
git clone https://github.com/mberks2025/Robotics-cam.git ~/Robotics-cam
cd ~/Robotics-cam
```

Then run the steps in order. Each script is safe to re-run.

| Step | Command | What it does | Time |
| --- | --- | --- | --- |
| 1 | `./scripts/01_jetson_prep.sh` | Max power mode, USB autosuspend off, build deps, 8 GB swap. **Camera unplugged.** | 5 min |
| 2 | `./scripts/02_install_librealsense.sh` | librealsense from source (RSUSB + CUDA + Python bindings) | 30-60 min |
| 3 | `./scripts/03_install_ros_jazzy.sh` | ROS 2 Jazzy, rviz2, vision_msgs, cv_bridge, colcon | 10 min |
| 4 | `./scripts/04_build_workspace.sh` | Clones realsense-ros into `ros2_ws/src`, builds it with this repo's packages | 15 min |
| 5 | `./scripts/05_setup_yolo_venv.sh` | `~/yolo-venv` with CUDA PyTorch + Ultralytics | 10 min |
| 6 | `source ~/yolo-venv/bin/activate && ./scripts/export_yolo_trt.sh` | `models/yolo26n.pt` to `models/yolo26n.engine` (FP16) | 5-10 min |

If the CUDA build in step 2 fails on JetPack 7.2, rerun it with `WITH_CUDA=OFF`; RSUSB still works,
only alignment moves to the CPU. Versions are overridable: `LIBRS_VERSION=...` (step 2) and
`REALSENSE_ROS_REF=...` (step 4). Keep the two paired (v2.X.Y with v4.X.Y).

After step 2, plug the camera in and check:

```bash
rs-enumerate-devices -s     # must say Usb Type Descriptor 3.2. "2.1" = USB 2 cable, hub or port
realsense-viewer            # Stereo, RGB and Motion modules all stream
```

> **USB 2.1 warning.** The camera has been on a USB 2.1 link so far, which caps it at about 15 fps and
> cannot carry stereo IR + depth + color together. Use the short USB 3 cable straight into the Orin's
> USB 3 port. Until then, use `profile:=usb2` below (no IR, so no VIO).

## Run it

Only one program can open the D435i, so **one camera node publishes everything** and every other
node subscribes. Do not run `realsense-viewer` or `tools/realsense_detect.py` at the same time.

```bash
# Camera only. Profiles live in ros2_ws/src/robot_bringup/config/
ros2 launch robot_bringup camera.launch.py                   # vio (default): color 15, depth+IR 30, IMU 200 Hz, emitter off
ros2 launch robot_bringup camera.launch.py profile:=detect   # + aligned depth for YOLO distances, emitter on
ros2 launch robot_bringup camera.launch.py profile:=terrain  # depth + IMU, emitter on, decimation/spatial/temporal filters
ros2 launch robot_bringup camera.launch.py profile:=usb2     # still on USB 2.1: color + depth 15 fps + IMU

# In a second shell: check topics, rates, IMU and emitter
./scripts/check_streams.sh

# Camera + YOLO detector (the launch file adds ~/yolo-venv to PYTHONPATH; venv:=<path> to change)
ros2 launch robot_bringup detector.launch.py                 # publishes vision_msgs/Detection3DArray on /detections
ros2 launch robot_bringup detector.launch.py debug_image:=true   # also /detections/image for rviz2 (costs CPU)
ros2 topic echo /detections --once
```

Switch the emitter live without restarting: `ros2 param set /camera/camera depth_module.emitter_enabled 1`.
A yellow "not supported" warning for `emitter_enabled` or `align_depth.enable` comes from the launch
file's argument list; the node still applies them.

### Expected topics (default namespace `/camera/camera`)

| Topic | Type | Rate (vio profile) |
| --- | --- | --- |
| `/camera/camera/color/image_raw` | sensor_msgs/Image | ~15 Hz |
| `/camera/camera/depth/image_rect_raw` | sensor_msgs/Image | ~30 Hz |
| `/camera/camera/infra1/image_rect_raw`, `infra2/...` | sensor_msgs/Image | ~30 Hz |
| `/camera/camera/imu` | sensor_msgs/Imu | ~200 Hz |
| `/camera/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | detect profile only |
| `/detections` | vision_msgs/Detection3DArray | up to color rate |

### First-day checklist

- [ ] `rs-enumerate-devices -s` shows USB 3.x
- [ ] `check_streams.sh`: color ~15 Hz, depth and IR ~30 Hz, IMU ~200 Hz
- [ ] IMU still and level: |linear_acceleration| about 9.8 m/s², angular_velocity near 0
- [ ] rviz2 (Fixed Frame `camera_link`) shows color, depth, both IR images and TF
- [ ] `./scripts/record_vio_bag.sh` saved a 1-2 minute walk-around bag (for OpenVINS / cuVSLAM)
- [ ] Engine exported; `detector.launch.py` publishes `/detections`
- [ ] Benchmarks logged in [docs/BENCHMARKS.md](docs/BENCHMARKS.md)

## Repo layout

```
scripts/                     install steps 01-05, TensorRT export, stream check, bag recording, tegrastats logging
ros2_ws/src/robot_bringup/   camera profiles (config/*.yaml) and launch files
ros2_ws/src/rover_perception/
  rover_perception/yolo_detector.py   YOLO node: color + aligned depth in, Detection3DArray out
  rover_perception/depth_utils.py     box_distance() from the original script, pixel -> 3D
  rover_perception/terrain_node.py    placeholder + design for the depth terrain grid
models/yolo26n.pt            COCO weights the team tested; the .engine is built on the Jetson (gitignored)
tools/realsense_detect.py    original standalone test (opens the camera itself)
docs/                        roadmap, benchmark plan, troubleshooting, original team notes
```

Unit tests for the depth helpers: `cd ros2_ws && colcon test --packages-select rover_perception && colcon test-result --verbose`.

## More

- [docs/ROADMAP.md](docs/ROADMAP.md): phases from sensor bring-up to hands-off runs, and the navigation stack plan
- [docs/BENCHMARKS.md](docs/BENCHMARKS.md): what to measure on the Jetson and where to record it
- [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md): common camera and build failures
- [docs/notes/](docs/notes/): the team's original notes (RAM tests, terrain detection, stereo/IMU task)
