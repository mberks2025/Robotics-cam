#!/usr/bin/env bash
# Step 2: build librealsense from source with the RSUSB backend (no kernel patches on L4T),
# CUDA, and Python bindings.
#
#   LIBRS_VERSION=v2.58.4 ./scripts/02_install_librealsense.sh
#   WITH_CUDA=OFF ./scripts/02_install_librealsense.sh      # if the CUDA build fails on JetPack 7.2
#
# Pick LIBRS_VERSION to match the realsense-ros release you use in step 4: the two are
# released in pairs, and the realsense-ros release notes name the librealsense version.
set -euo pipefail

LIBRS_VERSION="${LIBRS_VERSION:-v2.58.4}"
WITH_CUDA="${WITH_CUDA:-ON}"
SRC_DIR="${SRC_DIR:-$HOME/librealsense}"
JOBS="${JOBS:-4}"            # drop to 2 if the build is killed for memory

if [ ! -d "$SRC_DIR/.git" ]; then
  git clone https://github.com/realsenseai/librealsense.git "$SRC_DIR"
fi
cd "$SRC_DIR"
git fetch --tags
git checkout "$LIBRS_VERSION"
./scripts/setup_udev_rules.sh

mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_WITH_CUDA="$WITH_CUDA" -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DBUILD_EXAMPLES=ON -DBUILD_GRAPHICAL_EXAMPLES=ON \
  -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE="$(command -v python3)"
make -j"$JOBS"
sudo make install
sudo ldconfig

echo
echo "Plug the D435i into a USB 3 port (blue) with a USB 3 cable, then run:"
echo "  rs-enumerate-devices -s     # expect 'Usb Type Descriptor 3.2'; 2.1 = wrong cable/port"
echo "  realsense-viewer            # Stereo, RGB and Motion modules should all stream"
echo "  python3 -c 'import pyrealsense2 as rs; print(rs.__version__)'"
echo "If the import fails: export PYTHONPATH=\$PYTHONPATH:/usr/local/lib (or the dist-packages path make install printed)."
echo "Next: scripts/03_install_ros_jazzy.sh"
