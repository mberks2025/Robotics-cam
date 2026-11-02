# RealSense Stereo IR + IMU Streaming for ROS 2

## Goal

Get the RealSense's stereo infrared images and IMU streaming as ROS 2 topics. Those are the inputs OpenVINS needs to track how the robot moves. Running OpenVINS itself comes in the next stage.

See the project page [Robot Localization Pipeline Creation](https://app.notion.com/p/Robot-Localization-Pipeline-Creation-3ffc5b61d5c3807388b6fe7eddf4a2e1?pvs=21) for how nodes and topics work.

## Sensors

| Sensor | Have it? | Qty | Notes |
| --- | --- | --- | --- |
| RealSense D435i | Yes | 1 | Two IR cameras (stereo), color camera, depth, and an IMU. A second depth camera is under review. |
| Monocular RGB camera | No | 0 (up to 4 planned) | Under review; not needed for this task. |

Earlier RealSense work by Madison, Jackson, and Marty (YOLO RAM tests, terrain detection) is on [Stereo VIO - CAMERA + IMU](https://app.notion.com/p/Stereo-VIO-CAMERA-IMU-3e1c5b61d5c380f78d85e1e2a359cdac?pvs=21).

## What to Do

1. Install `librealsense` and the `realsense2_camera` ROS 2 package on the Jetson. Check that the camera works in `realsense-viewer`.
2. Launch `realsense2_camera` with both IR streams and the gyro and accelerometer enabled. Combine the gyro and accelerometer into one IMU topic using `unite_imu_method`.
3. Turn off the IR emitter. Its dot pattern shows up in the IR images and confuses feature tracking.
4. Check the topics with `ros2 topic list`. You should see two IR image topics (`sensor_msgs/Image`) and one IMU topic (`sensor_msgs/Imu`). Exact names depend on the launch namespace.
5. Check rates: approximately **30 Hz** for images and **200 Hz** for the IMU.
6. Walk the camera around a room and record a ROS 2 bag containing the IR and IMU topics. OpenVINS will be tested on this bag before it goes on the robot.
7. The AprilTag task uses this same camera. Only one program can open the D435i, so keep all RealSense streams in this one node and put the launch file somewhere both tasks can use it.

## Done When

- [ ] Both IR streams and the IMU stream at a steady rate and show in `rviz2`.
- [ ] The test bag is saved and linked here.

## Next Stage

Camera–IMU calibration (e.g., Kalibr), then OpenVINS.
