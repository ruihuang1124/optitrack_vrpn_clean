#!/usr/bin/env bash
set -Eeuo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
MUJOCO_BUILD="${MUJOCO_BUILD:-/home/ray/projects/unitree_mujoco/simulate/build}"
MUJOCO_BIN="${MUJOCO_BIN:-${MUJOCO_BUILD}/unitree_mujoco}"
MUJOCO_RVIZ_BRIDGE="${MUJOCO_RVIZ_BRIDGE:-/home/ray/projects/unitree_mujoco/simulate/tools/rviz_mujoco_frames.py}"
RVIZ_CONFIG="${RVIZ_CONFIG:-/home/ray/projects/unitree_mujoco/simulate/tools/mujoco_frames.rviz}"
UDP_PORT="${UDP_PORT:-16001}"
OPEN_RVIZ="${OPEN_RVIZ:-1}"

PIDS=()

cleanup() {
  trap - EXIT INT TERM
  echo
  echo "[run_mujoco_rviz] stopping..."
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
  echo "[run_mujoco_rviz] starting ${name}: $*"
  "$@" &
  PIDS+=("$!")
}

start_ros_bg() {
  local name="$1"
  shift
  echo "[run_mujoco_rviz] starting ${name} with ROS env: $*"
  ROS_SETUP="${ROS_SETUP}" bash -lc \
    'set +u; source "${ROS_SETUP}"; set -u; exec "$@"' \
    bash "$@" &
  PIDS+=("$!")
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -x "${MUJOCO_BIN}" ]]; then
  echo "MuJoCo binary not found or not executable: ${MUJOCO_BIN}" >&2
  exit 1
fi
if [[ ! -x "${MUJOCO_RVIZ_BRIDGE}" ]]; then
  echo "MuJoCo RViz bridge not found or not executable: ${MUJOCO_RVIZ_BRIDGE}" >&2
  exit 1
fi

trap cleanup EXIT INT TERM

start_ros_bg "mujoco_rviz_bridge" \
  "${MUJOCO_RVIZ_BRIDGE}" \
  --udp-port "${UDP_PORT}"

sleep 0.5

start_bg "unitree_mujoco" \
  bash -lc "cd '${MUJOCO_BUILD}' && exec '${MUJOCO_BIN}' --debug_rviz --debug_rviz_udp_port '${UDP_PORT}'"

sleep 1.0

if [[ "${OPEN_RVIZ}" == "1" ]]; then
  start_ros_bg "rviz2" rviz2 -d "${RVIZ_CONFIG}"
fi

echo "[run_mujoco_rviz] running. Press Ctrl+C to stop all processes."
wait -n "${PIDS[@]}" || true
