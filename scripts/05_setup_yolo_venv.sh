#!/usr/bin/env bash
# Step 5: Python venv for Ultralytics on JetPack 7.2 (CUDA 13, TensorRT 10.x).
# --system-site-packages keeps rclpy, cv_bridge, pyrealsense2 and JetPack's tensorrt visible.
# Ubuntu 24.04 blocks system-wide pip installs, hence the venv.
set -euo pipefail

VENV="${VENV:-$HOME/yolo-venv}"
python3 -m venv --system-site-packages "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
pip install --upgrade pip
# Install a CUDA 13 build of PyTorch FIRST, or pip pulls a CPU-only / wrong-CUDA one.
# If this index has no aarch64 wheel for your JetPack, use NVIDIA's Jetson wheels instead
# (see https://docs.ultralytics.com/guides/nvidia-jetson and NVIDIA's PyTorch-for-Jetson page).
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu130
pip install ultralytics psutil
# Keep JetPack's TensorRT 10.x (Ultralytics: TensorRT 11 does not support Jetson). Do not pip install tensorrt.

python - <<'PY'
import torch, ultralytics
print("torch", torch.__version__, "cuda:", torch.cuda.is_available())
print("ultralytics", ultralytics.__version__)
try:
    import tensorrt; print("tensorrt", tensorrt.__version__)
except Exception as e:
    print("tensorrt import failed:", e)
PY
echo
echo "Activate in each shell that runs YOLO:  source $VENV/bin/activate"
echo "Next: scripts/export_yolo_trt.sh"
