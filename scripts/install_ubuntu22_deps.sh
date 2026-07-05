#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y build-essential ca-certificates cmake git libyaml-cpp-dev

if apt-cache show libvrpn-dev >/dev/null 2>&1; then
  sudo apt install -y libvrpn-dev
else
  workdir="$(mktemp -d)"
  echo "libvrpn-dev is not available from the current apt sources."
  echo "Building VRPN from source in ${workdir} ..."

  git clone --recursive --depth 1 https://github.com/vrpn/vrpn.git "${workdir}/vrpn"
  cmake -S "${workdir}/vrpn" -B "${workdir}/vrpn/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DVRPN_BUILD_CLIENT_LIBRARY=ON \
    -DVRPN_BUILD_SERVER_LIBRARY=OFF \
    -DVRPN_BUILD_CLIENTS=OFF \
    -DVRPN_BUILD_SERVERS=OFF \
    -DVRPN_BUILD_PYTHON=OFF \
    -DVRPN_BUILD_PYTHON_HANDCODED_2X=OFF \
    -DVRPN_BUILD_PYTHON_HANDCODED_3X=OFF \
    -DVRPN_BUILD_JAVA=OFF \
    -DBUILD_TESTING=OFF
  cmake --build "${workdir}/vrpn/build" -j
  sudo cmake --install "${workdir}/vrpn/build"
  sudo ldconfig
fi
