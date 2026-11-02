# Roadmap and navigation stack

Goal: a rover that detects obstacles and terrain hazards (boulders 30-40 cm, craters up to 40-50 cm),
localizes with stereo + IMU + AprilTags, and drives to preset targets with no help, all inside the
Orin Nano's 8 GB shared CPU/GPU memory.

## Stack

One ROS 2 Jazzy graph. Every node reads three transforms (REP 105): `map -> odom` from AprilTags,
`odom -> base_link` from visual-inertial odometry, `base_link -> camera` from the URDF.

| Part | Default | Fallback / benchmark | Notes |
| --- | --- | --- | --- |
| Odometry | Isaac ROS Visual SLAM (cuVSLAM), GPU, stereo IR + IMU | OpenVINS (CPU), run on the same bags | Isaac ROS 4.x is the Jazzy / JetPack 7 line; confirm Orin Nano support |
| Tags | `isaac_ros_apriltag` (GPU), tag36h11 | `apriltag_ros` (CPU, ~5 Hz) | small `tag_localizer` node publishes smoothed `map -> odom`; tag poses in a YAML map |
| Terrain | depth-only 5 cm grid (`rover_perception/terrain_node.py`, single-frame today); next: rolling grid in `odom` + custom Nav2 costmap layer | `elevation_mapping_cupy` (GPU, more RAM) | emitter-on depth; IMU gravity for "up" |
| Navigation | Nav2: Smac Planner 2D + Regulated Pure Pursuit | Smac Hybrid-A* (car-like), MPPI only if needed | recoveries trimmed to back up and wait; collision monitor stop zone |
| Labels | YOLO26n TensorRT FP16 (this repo) | Isaac ROS YOLOv8 or bare TensorRT runtime if RAM is tight | optional; only for people / targets |
| Mapping | deferred | RTAB-Map fed by cuVSLAM odometry | only if a saved 3D map is needed |

**Emitter conflict.** VIO and tags want the IR emitter off (its dots look like features); terrain depth
wants it on (sand has little texture). Plan: `depth_module.emitter_on_off: true` so frames alternate,
then split them (Isaac ROS has a RealSense splitter): emitter-off IR to VIO and tags, emitter-on depth to
terrain. Each consumer gets half the frame rate.

## Phases

Each phase ends with a gate the robot must pass before the next starts. Phase 3 needs only Phase 1's
odometry, so terrain work can run alongside Phase 2.

| Phase | Work | Gate |
| --- | --- | --- |
| **0. Sensor bring-up (now, this repo)** | realsense-ros on Jazzy / JetPack 7.2, USB 3, top power mode; IR pair, depth, IMU; YOLO TensorRT benchmark; record walk-around bags | 30 Hz images and 200 Hz IMU steady; test bag saved; RAM measured |
| 1. Odometry | Kalibr camera-IMU calibration; cuVSLAM and OpenVINS on the same bags | Odometry chosen on measured drift, CPU and RAM |
| 2. AprilTag localization | Survey tag poses into a tag map; `tag_localizer` publishes `map -> odom`; heading from the first tag at start-up | Pose snaps back to the tag map whenever a tag is seen |
| 3. Terrain grid | ROS 2 terrain node done (single frame, tested in simulation); next: rolling odom grid, custom costmap layer, physical tests | 30-40 cm boulders and craters flagged on real terrain |
| 4. Nav2 autonomy | URDF, both costmaps, Smac 2D + Pure Pursuit; preset targets from YAML | Repeated runs reach preset targets without help |
| 5. Hardening | Rear D435i into odometry, tags and terrain; dust and tracking-loss drills; optional YOLO, MPPI | Full mission runs end to end, hands off |

## Open questions

- [ ] Where do the 2026 guidebook rules allow AprilTags or other markers?
- [ ] Drive type (skid-steer, differential, car-like) and wheel encoders?
- [ ] Rover's maximum climbable step and slope (sets the bump / hole / slope thresholds)?
- [ ] Second D435i for the rear confirmed? Can the carrier's USB 3 ports carry two cameras?
- [ ] Do Isaac ROS 4.x and OpenVINS both build and run on JetPack 7.2 / Jazzy?
- [ ] Does anything need recognising by name? If not, YOLO stays off during runs.
- [ ] Dust: lens cover or wiper check between runs.

Full write-ups: [setup guide](https://claude.ai/artifact/AJENwyZDgnJrgFJSUovBhD),
[YOLO recommendation](https://claude.ai/artifact/DDhsPXow3K4EXCK4rPBZto),
[navigation stack plan](https://claude.ai/artifact/PoDHeVKvndkjo2NBY9qRwj) (may need access from Marty).
