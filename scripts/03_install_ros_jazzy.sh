#!/usr/bin/env bash
# Step 3: ROS 2 Jazzy (ros-base + rviz2) plus the packages this repo's nodes need.
set -euo pipefail

if [ ! -f /etc/apt/sources.list.d/ros2.list ] && ! dpkg -l ros2-apt-source >/dev/null 2>&1; then
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-rviz2 \
  ros-jazzy-vision-msgs ros-jazzy-cv-bridge ros-jazzy-message-filters \
  ros-jazzy-diagnostic-updater ros-jazzy-xacro \
  python3-rosdep python3-colcon-common-extensions python3-pytest

grep -q '/opt/ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo "Done. Open a new shell (or: source /opt/ros/jazzy/setup.bash). Next: scripts/04_build_workspace.sh"
