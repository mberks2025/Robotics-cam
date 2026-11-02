#!/usr/bin/env bash
# Step 4: clone realsense-ros into this repo's ros2_ws and build everything.
#
#   REALSENSE_ROS_REF=v4.58.4 ./scripts/04_build_workspace.sh
#
# Use the realsense-ros tag whose release notes name the librealsense version from step 2.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO_DIR/ros2_ws"
REALSENSE_ROS_REF="${REALSENSE_ROS_REF:-v4.58.4}"   # pairs with librealsense v2.58.4

source /opt/ros/jazzy/setup.bash

if [ ! -d "$WS/src/realsense-ros/.git" ]; then
  git clone https://github.com/realsenseai/realsense-ros.git "$WS/src/realsense-ros"
fi
git -C "$WS/src/realsense-ros" fetch --tags origin
git -C "$WS/src/realsense-ros" checkout "$REALSENSE_ROS_REF"

cd "$WS"
sudo rosdep init 2>/dev/null || true
rosdep update
# librealsense2 is skipped: we built it from source in step 2.
rosdep install -i --from-path src --rosdistro jazzy --skip-keys=librealsense2 -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

LINE="source $WS/install/setup.bash"
grep -qF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc
echo
echo "Built. Open a new shell, then smoke test:"
echo "  ros2 launch robot_bringup camera.launch.py          # USB 3"
echo "  ros2 launch robot_bringup camera.launch.py profile:=usb2   # still on USB 2.1"
echo "  ./scripts/check_streams.sh"
