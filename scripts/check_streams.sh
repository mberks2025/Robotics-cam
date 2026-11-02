#!/usr/bin/env bash
# With the camera node running in another shell, check topics, rates, IMU and emitter.
# Expected (vio profile on USB 3): color ~15 Hz, depth ~30 Hz, infra1/2 ~30 Hz, imu ~200 Hz.
set -uo pipefail
NS="${NS:-/camera/camera}"
SECS="${SECS:-8}"

echo "== Topics =="
ros2 topic list | grep "$NS" || { echo "No $NS topics. Is the camera node running?"; exit 1; }

hz() {
  local topic="$1"
  echo "== $topic =="
  timeout "$SECS" ros2 topic hz "$topic" 2>/dev/null | grep -E "average rate" | tail -1 || echo "  no messages"
}
for t in color/image_raw depth/image_rect_raw infra1/image_rect_raw infra2/image_rect_raw imu aligned_depth_to_color/image_raw; do
  ros2 topic list | grep -qx "$NS/$t" && hz "$NS/$t"
done

echo "== IMU sample (still and level: |accel| ~9.8 m/s^2, gyro ~0) =="
ros2 topic echo "$NS/imu" --once 2>/dev/null | sed -n '/angular_velocity:/,/linear_acceleration_covariance/p' | head -12

echo "== Emitter =="
ros2 param get "$NS" depth_module.emitter_enabled
