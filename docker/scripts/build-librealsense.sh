#!/bin/bash
# Build librealsense from source
set -e

VERSION=${1:-v2.55.1}
if [[ "$1" == "-v" ]]; then
    VERSION=$2
fi

cd /opt
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout $VERSION

mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=false \
    -DBUILD_GRAPHICAL_EXAMPLES=false \
    -DBUILD_PYTHON_BINDINGS=false \
    -DFORCE_RSUSB_BACKEND=true

make -j$(nproc)
make install
ldconfig