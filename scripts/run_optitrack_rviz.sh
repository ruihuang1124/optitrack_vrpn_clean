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
EXTRA_CONTROL_UDP_PORTS="${EXTRA_CONTROL_UDP_PORTS:-}"
OPEN_RVIZ="${OPEN_RVIZ:-1}"
TRACKERS="${TRACKERS:-go2_body_marker,piper_ee}"
NODE_TRACKER="${NODE_TRACKER:-}"
SET_WORLD_FROM="${SET_WORLD_FROM:-go2_body_marker}"
SET_WORLD_FROM_BASE="${SET_WORLD_FROM_BASE:-}"

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

tracker_list_contains() {
  local needle="$1"
  local item
  local -a items
  IFS=',' read -ra items <<< "${TRACKERS}"
  for item in "${items[@]}"; do
    item="${item//[[:space:]]/}"
    if [[ "${item}" == "${needle}" ]]; then
      return 0
    fi
  done
  return 1
}

first_tracker_in_list() {
  local item
  local -a items
  IFS=',' read -ra items <<< "${TRACKERS}"
  for item in "${items[@]}"; do
    item="${item//[[:space:]]/}"
    if [[ -n "${item}" ]]; then
      echo "${item}"
      return 0
    fi
  done
  return 1
}

resolve_world_from_tracker() {
  if [[ "${SET_WORLD_FROM_BASE}" == "1" ]]; then
    echo "go2_body_marker"
    return 0
  fi

  case "${SET_WORLD_FROM}" in
    ""|"auto")
      ;;
    "0"|"false"|"False"|"none"|"off")
      return 0
      ;;
    *)
      echo "${SET_WORLD_FROM}"
      return 0
      ;;
  esac

  if [[ -z "${NODE_TRACKER}" || "${NODE_TRACKER}" == "go2_body_marker" || "${NODE_TRACKER}" == "go2_base" ]]; then
    if [[ -z "${TRACKERS}" ]] || tracker_list_contains "go2_body_marker"; then
      echo "go2_body_marker"
      return 0
    fi
  fi

  if [[ -n "${NODE_TRACKER}" ]]; then
    echo "${NODE_TRACKER}"
    return 0
  fi

  first_tracker_in_list || true
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

NODE_ARGS=(
  --config "${CONFIG}"
  --host "${VRPN_HOST}"
  --no-print
  --udp "127.0.0.1:${UDP_PORT}"
)
if [[ -n "${NODE_TRACKER}" ]]; then
  NODE_ARGS+=(--tracker "${NODE_TRACKER}")
fi

start_bg "optitrack_vrpn_clean_node" \
  "${NODE_BIN}" \
  "${NODE_ARGS[@]}"

RVIZ_ARGS=(
  --udp-port "${UDP_PORT}"
  --frame enu
  --axis-map z,-y,x
  --trackers "${TRACKERS}"
  --forward-corrected "127.0.0.1:${CONTROL_UDP_PORT}"
  --show-marker-frames
  --local-axis-map go2_body_marker:z,-x,-y
  --local-axis-map piper_ee_marker:y,-x,z
  --local-axis-map piper_ee:y,-x,z
  --local-rpy-deg piper_ee:1.604,9.046,-2.787
)
if [[ -n "${EXTRA_CONTROL_UDP_PORTS}" ]]; then
  IFS=',' read -ra extra_ports <<< "${EXTRA_CONTROL_UDP_PORTS}"
  for extra_port in "${extra_ports[@]}"; do
    extra_port="${extra_port//[[:space:]]/}"
    if [[ -n "${extra_port}" ]]; then
      RVIZ_ARGS+=(--forward-corrected "127.0.0.1:${extra_port}")
    fi
  done
fi
WORLD_FROM_TRACKER="$(resolve_world_from_tracker)"
if [[ -n "${WORLD_FROM_TRACKER}" ]]; then
  RVIZ_ARGS+=(--set-world-from "${WORLD_FROM_TRACKER}")
fi

sleep 0.5
start_ros_bg "rviz_udp_frames" "${RVIZ_BRIDGE}" "${RVIZ_ARGS[@]}"

sleep 1.0
if [[ "${OPEN_RVIZ}" == "1" ]]; then
  start_ros_bg "rviz2" rviz2 -d "${RVIZ_CONFIG}"
fi

echo "[run_optitrack_rviz] running. Trackers=${TRACKERS}, node_tracker=${NODE_TRACKER:-all}, set_world_from=${WORLD_FROM_TRACKER:-none}, corrected control UDP=127.0.0.1:${CONTROL_UDP_PORT}. Press Ctrl+C to stop all processes."
wait -n "${PIDS[@]}" || true
