# Troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| `No RealSense devices were found` | Another program has the camera, or udev rules missing | Close `realsense-viewer` and `realsense_detect.py`; rerun `~/librealsense/scripts/setup_udev_rules.sh`; replug |
| Log says USB type 2.1 | USB 2 cable, hub or port | Short USB 3 cable straight into the Orin's USB 3 port; until then `profile:=usb2` |
| No `imu` topic | Gyro or accel disabled, or `unite_imu_method: 0` | Enable both and set 2; check the Motion module streams in `realsense-viewer` |
| `Frames didn't arrive within 5 seconds` | Power saving or USB autosuspend | `sudo nvpmodel -m 0`, `sudo jetson_clocks`, rerun `01_jetson_prep.sh`; profiles already set `initial_reset: true` |
| Profile warning, stream not opened | Combination the sensor does not offer, or depth and IR rates differ | `rs-enumerate-devices` lists valid profiles; keep depth and IR at the same fps |
| Node exits with a librealsense version error | Wrapper built against a different SDK version | Check out paired tags (librealsense v2.X.Y, realsense-ros v4.X.Y) and rebuild both |
| librealsense build killed | Out of memory | `JOBS=2 ./scripts/02_install_librealsense.sh`; make sure swap is on (`swapon --show`) |
| CUDA compile errors in librealsense | JetPack 7 CUDA newer than what the release was tested on | `WITH_CUDA=OFF ./scripts/02_install_librealsense.sh` |
| `import pyrealsense2` fails | Bindings installed outside `sys.path` | `export PYTHONPATH=$PYTHONPATH:/usr/local/lib` or the `python3.12/dist-packages` path `make install` printed |
| `ModuleNotFoundError: ultralytics` from the detector | venv not active when launching | `source ~/yolo-venv/bin/activate`, then launch again |
| `torch.cuda.is_available()` is False | CPU-only torch wheel | Reinstall torch from the cu130 index or NVIDIA's Jetson wheels (see `05_setup_yolo_venv.sh`) |
| `.engine` fails to load | Engine built on another machine or TensorRT version | Rebuild it on this Jetson with `export_yolo_trt.sh` |
| Detector runs but publishes nothing | No aligned depth | Launch the camera with `profile:=detect` (or `usb2`), which turns on `align_depth.enable` |
| IR images show a dot pattern | Emitter on | `ros2 param set /camera/camera depth_module.emitter_enabled 0` (vio profile has it off) |
