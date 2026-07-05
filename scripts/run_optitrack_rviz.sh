#!/usr/bin/env bash
set -Eeuo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
PACKAGE_DIR="${PACKAGE_DIR:-/home/ray/software/optitrack_vrpn_clean}"
NODE_BIN="${NODE_BIN:-${PACKAGE_DIR}/build/optitrack_vrpn_clean_node}"
RVIZ_BRIDGE="${RVIZ_BRIDGE:-${PACKAGE_DIR}/scripts/rviz_udp_frames.py}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${PACKAGE_DIR}/config/mocap_frames.rviz}"
CONFIG="${CONFIG:-${PACKAGE_DIR}/config/default.yaml}"
VRPN_HOST="${VRPN_HOST:-192.168.151.100}"
UDP_PORT="${UDP_PORT:-15001}"
CONTROL_UDP_PORT="${CONTROL_UDP_PORT:-15002}"
OPEN_RVIZ="${OPEN_RVIZ:-1}"
SET_WORLD_FROM_BASE="${SET_WORLD_FROM_BASE:-1}"

PIDS=()

cleanup() {
  trap - EXIT INT TERM
  echo
  echo "[run_optitrack_rviz] stopping..."
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${PIDS[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
}

start_bg() {
  local name="$1"
  shift
  echo "[run_optitrack_rviz] starting ${name}: $*"
  "$@" &
  PIDS+=("$!")
}

start_ros_bg() {
  local name="$1"
  shift
  echo "[run_optitrack_rviz] starting ${name} with ROS env: $*"
  ROS_SETUP="${ROS_SETUP}" bash -lc \
    'set +u; source "${ROS_SETUP}"; set -u; exec "$@"' \
    bash "$@" &
  PIDS+=("$!")
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -x "${NODE_BIN}" ]]; then
  echo "OptiTrack node not found or not executable: ${NODE_BIN}" >&2
  exit 1
fi
if [[ ! -x "${RVIZ_BRIDGE}" ]]; then
  echo "RViz bridge not found or not executable: ${RVIZ_BRIDGE}" >&2
  exit 1
fi

trap cleanup EXIT INT TERM

start_bg "optitrack_vrpn_clean_node" \
  "${NODE_BIN}" \
  --config "${CONFIG}" \
  --host "${VRPN_HOST}" \
  --no-print \
  --udp "127.0.0.1:${UDP_PORT}"

RVIZ_ARGS=(
  --udp-port "${UDP_PORT}"
  --frame enu
  --axis-map z,-y,x
  --trackers go2_base,piper_ee
  --forward-corrected "127.0.0.1:${CONTROL_UDP_PORT}"
  --show-marker-frames
  --local-axis-map go2_base:z,-x,-y
  --local-axis-map piper_ee:y,-x,z
  --local-rpy-deg piper_ee:1.604,9.046,-2.787
)
if [[ "${SET_WORLD_FROM_BASE}" == "1" ]]; then
  RVIZ_ARGS+=(--set-world-from-base)
fi

sleep 0.5
start_ros_bg "rviz_udp_frames" "${RVIZ_BRIDGE}" "${RVIZ_ARGS[@]}"

sleep 1.0
if [[ "${OPEN_RVIZ}" == "1" ]]; then
  start_ros_bg "rviz2" rviz2 -d "${RVIZ_CONFIG}"
fi

echo "[run_optitrack_rviz] running. Corrected control UDP: 127.0.0.1:${CONTROL_UDP_PORT}. Press Ctrl+C to stop all processes."
wait -n "${PIDS[@]}" || true
