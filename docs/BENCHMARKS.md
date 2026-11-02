# Benchmarks to run on the Jetson

The RAM figures so far (YOLO 434 MB after first inference, terrain about 130 MB) were measured on a
Windows desktop with about 22 GB in use, not on the Jetson, and YOLO ran through PyTorch. Re-measure
here. Set full power first (`sudo nvpmodel -m 0 && sudo jetson_clocks`) and log RAM with
`./scripts/log_tegrastats.sh <name>.log` (tegrastats counts the GPU's share of the 8 GB; psutil does not).

## Checklist

- [ ] Camera on USB 3 (`rs-enumerate-devices -s` says 3.x). USB 2.1 halves the frame rate before the model is involved
- [ ] Baseline: camera node alone (vio profile), note RAM and CPU per core from tegrastats
- [ ] `yolo benchmark model=models/yolo26n.pt imgsz=640 device=0` to confirm the published TensorRT times
- [ ] Detector with the FP16 engine at 640, then at 416 (`IMGSZ=416 ./scripts/export_yolo_trt.sh`, launch with `imgsz:=416` param); end-to-end rate with `ros2 topic hz /detections`
- [ ] Terrain node alone (`terrain.launch.py`): processing ms from `/terrain/features`, RAM from tegrastats (desktop estimate was about 130 MB)
- [ ] Terrain on real ground: a 30-40 cm rock and a dug hole at 1-2 m are reported with the right size and position
- [ ] Peak RAM with camera node + detector + terrain node all running
- [ ] Only if named classes are needed: fine-tune YOLO26n on rover-view images, re-export, compare mAP

## Reference numbers (Ultralytics, Orin Nano Super, JetPack 6.1, 640 px, inference only)

| Format | Inference (ms) | Model file (MB) | mAP50-95 (coco128) |
| --- | ---: | ---: | ---: |
| PyTorch `.pt` | 15.60 | 5.3 | 0.479 |
| TensorRT FP32 | 7.53 | 11.3 | 0.477 |
| **TensorRT FP16** | **4.57** | **8.1** | **0.480** |
| TensorRT INT8 | 3.80 | 5.3 | 0.449 |

Expect the camera (15 fps color in the default profile) and Python image conversion to set the end-to-end
rate, not the model. Source: [Ultralytics Jetson guide](https://docs.ultralytics.com/guides/nvidia-jetson).

## Results (fill in)

| Date | Config | Camera profile | End-to-end FPS | Peak RAM (tegrastats) | Notes |
| --- | --- | --- | --- | --- | --- |
| | camera node only | vio | n/a | | baseline |
| | FP16, 640 | detect | | | |
| | FP16, 416 | detect | | | |
| | INT8, 640 | detect | | | needs calibration set |
| | terrain node | terrain | | | |
