#!/usr/bin/env bash
# Export YOLO26n to a TensorRT FP16 engine. Run ON THE JETSON (an .engine is tied to the
# GPU and TensorRT version it was built with), inside the venv from step 5.
#
#   ./scripts/export_yolo_trt.sh                    # models/yolo26n.pt -> models/yolo26n.engine, 640 px FP16
#   IMGSZ=416 ./scripts/export_yolo_trt.sh          # smaller/faster, worse on small or distant objects
#   PRECISION=8 DATA=rover.yaml MODEL=best.pt ./scripts/export_yolo_trt.sh   # INT8, only with your own calibration set
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MODEL="${MODEL:-$REPO_DIR/models/yolo26n.pt}"
IMGSZ="${IMGSZ:-640}"
PRECISION="${PRECISION:-16}"
WORKSPACE_GB="${WORKSPACE_GB:-2}"    # small workspace so the build itself fits in 8 GB

sudo nvpmodel -m 0 && sudo jetson_clocks

ARGS=(model="$MODEL" format=engine imgsz="$IMGSZ" workspace="$WORKSPACE_GB")
if [ "$PRECISION" = "8" ]; then
  : "${DATA:?INT8 needs DATA=<dataset yaml> with ~500 rover-view calibration images}"
  ARGS+=(data="$DATA" batch=8)
fi

# Newer Ultralytics uses quantize=16/8; older releases use half=True / int8=True.
if yolo export "${ARGS[@]}" quantize="$PRECISION"; then
  :
else
  echo "quantize= not accepted by this Ultralytics version, retrying with half/int8 flags"
  if [ "$PRECISION" = "8" ]; then yolo export "${ARGS[@]}" int8=True; else yolo export "${ARGS[@]}" half=True; fi
fi

ENGINE="${MODEL%.pt}.engine"
ls -lh "$ENGINE"
echo "Quick check: yolo predict model=$ENGINE source=$REPO_DIR/docs/notes  (any image folder) or run the ROS node."
