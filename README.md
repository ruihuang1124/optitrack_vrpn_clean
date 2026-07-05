# optitrack_vrpn_clean

Standalone OptiTrack VRPN client for Ubuntu without ROS.

This package was split from the local ROS/catkin package at
`/home/ray/software/optitrack_vrpn`.  It keeps the useful parts: connecting to
Motive through VRPN, converting OptiTrack poses to ENU/NED, aligning Motive
timestamps with local wall time, and writing poses to the terminal, CSV, or UDP.

## Dependencies

Required:

```bash
sudo apt update
sudo apt install -y build-essential ca-certificates cmake git libyaml-cpp-dev
```

No ROS package is required.

On some Ubuntu 22.04 mirrors, `libvrpn-dev` is not packaged. Use the helper
script below; it tries `libvrpn-dev` first, then falls back to building VRPN
from the official source tree:

```bash
./scripts/install_ubuntu22_deps.sh
```

Manual fallback if you prefer to run the VRPN source build yourself:

```bash
workdir="$(mktemp -d)"
git clone --recursive --depth 1 https://github.com/vrpn/vrpn.git "$workdir/vrpn"
cmake -S "$workdir/vrpn" -B "$workdir/vrpn/build" \
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
cmake --build "$workdir/vrpn/build" -j
sudo cmake --install "$workdir/vrpn/build"
sudo ldconfig
```

## Build

```bash
cd /home/ray/software/optitrack_vrpn_clean
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

If you only want to build the conversion/config tests on a machine that does
not have VRPN installed yet:

```bash
cmake -S . -B build_no_vrpn -DOPTITRACK_BUILD_CLIENT=OFF
cmake --build build_no_vrpn -j
ctest --test-dir build_no_vrpn --output-on-failure
```

## Run

List rigid bodies that Motive exposes through VRPN:

```bash
./build/optitrack_vrpn_clean_node --config config/default.yaml --host 192.168.151.100 --list
```

Print one rigid body:

```bash
./build/optitrack_vrpn_clean_node --config config/default.yaml --host 192.168.151.100 --tracker go2
```

Write CSV while printing at 10 Hz:

```bash
./build/optitrack_vrpn_clean_node --config config/default.yaml --host 192.168.151.100 --print-rate 10 --csv /tmp/optitrack.csv
```

Send UDP JSON to localhost:

```bash
./build/optitrack_vrpn_clean_node --config config/default.yaml --host 192.168.151.100 --udp 127.0.0.1:15001
```

## Motive Setup Checklist

In Motive, enable VRPN streaming and make sure the rigid body is actively
tracked. The rigid body name shown in Motive is the name to pass to
`--tracker`.

The client machine must be on the same network as the Motive machine. If the
default VRPN host uses a non-default port, pass it as `HOST:PORT`.

## Coordinate Frames

OptiTrack default axes:

- `x`: forward
- `y`: up
- `z`: right

ENU output:

- `x/east = OptiTrack z`
- `y/north = OptiTrack x`
- `z/up = OptiTrack y`

NED output:

- `x/north = OptiTrack x`
- `y/east = OptiTrack z`
- `z/down = -OptiTrack y`

The ENU orientation conversion follows the original ROS package: it first maps
the OptiTrack quaternion into a right-front-up body frame, then rotates body
axes into forward-left-up.

## Notes

The first `offset_samples` frames use local arrival time while the client
estimates the Motive-to-local timestamp offset. After that, the client uses the
Motive timestamp plus the best observed offset, which reduces timestamp jitter.
