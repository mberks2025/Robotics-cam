#!/usr/bin/env bash
# Step 1: power mode, USB autosuspend off, build deps, swap. Run with the camera UNPLUGGED.
set -euo pipefail

echo "== Power mode =="
sudo nvpmodel -q || true
sudo nvpmodel -m 0            # mode 0 = MAXN / MAXN SUPER on Orin Nano; check `sudo nvpmodel -q` output above
sudo jetson_clocks

echo "== USB autosuspend off (now and on every boot) =="
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend >/dev/null
if ! grep -q "usbcore.autosuspend=-1" /boot/extlinux/extlinux.conf 2>/dev/null; then
  echo "NOTE: to keep autosuspend off after reboot, append 'usbcore.autosuspend=-1' to the APPEND line"
  echo "      in /boot/extlinux/extlinux.conf (not done automatically)."
fi

echo "== Build dependencies =="
sudo apt-get update
sudo apt-get install -y git cmake build-essential libssl-dev libusb-1.0-0-dev libudev-dev \
  pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev python3-dev python3-venv \
  curl software-properties-common

echo "== Swap (the CUDA build and TensorRT export can run out of 8 GB) =="
if [ -z "$(swapon --show --noheadings)" ]; then
  sudo fallocate -l 8G /swapfile
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  sudo swapon /swapfile
  grep -q '^/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
else
  swapon --show
fi
echo "Done. Next: scripts/02_install_librealsense.sh"
