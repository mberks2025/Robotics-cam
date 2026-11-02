#!/usr/bin/env bash
# Record the stereo IR + IMU test bag for OpenVINS / cuVSLAM (vio profile, emitter off).
# Walk the camera around a room for 1-2 minutes, moving and rotating on all axes. Ctrl+C to stop.
# Record to an SSD or fast microSD: two raw IR streams at 30 fps write about 18 MB/s.
set -euo pipefail
NS="${NS:-/camera/camera}"
OUT="${1:-d435i_vio_test_$(date +%Y%m%d_%H%M%S)}"
ros2 bag record -o "$OUT" \
  "$NS/infra1/image_rect_raw" "$NS/infra1/camera_info" \
  "$NS/infra2/image_rect_raw" "$NS/infra2/camera_info" \
  "$NS/imu" /tf_static
