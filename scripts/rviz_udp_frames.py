#!/usr/bin/python3
import argparse
import json
import math
import select
import socket
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish optitrack_vrpn_clean UDP pose packets as ROS2 TF frames."
    )
    parser.add_argument("--bind", default="127.0.0.1", help="UDP bind host, default 127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=15001, help="UDP port, default 15001")
    parser.add_argument("--frame", choices=("enu", "ned"), default="enu", help="Packet frame to visualize")
    parser.add_argument("--world-frame", default="world", help="RViz fixed/world frame name")
    parser.add_argument("--origin-frame", default="world_origin", help="Static child frame at the world origin")
    parser.add_argument(
        "--axis-map",
        default="x,y,z",
        help=(
            "Remap packet world axes before publishing TF. Format gives output x,y,z "
            "from input axes, e.g. z,-y,x means out=(in_z,-in_y,in_x). Default x,y,z"
        ),
    )
    parser.add_argument(
        "--trackers",
        default="",
        help="Comma-separated tracker names to publish. Empty means publish every tracker.",
    )
    parser.add_argument("--frame-prefix", default="", help="Prefix added to each tracker TF child frame")
    parser.add_argument(
        "--forward",
        action="append",
        default=[],
        metavar="HOST:PORT",
        help="Forward every received UDP packet to HOST:PORT. Repeat to forward to multiple consumers.",
    )
    parser.add_argument(
        "--forward-corrected",
        action="append",
        default=[],
        metavar="HOST:PORT",
        help=(
            "Forward corrected pose packets after axis-map/local-frame/set-world transforms. "
            "Use this for robot controllers that should consume exactly what RViz displays."
        ),
    )
    parser.add_argument(
        "--show-marker-frames",
        action="store_true",
        help="Also publish uncorrected Motive marker frames as <tracker>_marker.",
    )
    parser.add_argument(
        "--local-axis-map",
        action="append",
        default=[],
        metavar="TRACKER:MAP",
        help=(
            "Apply a per-tracker local frame correction. MAP gives corrected child axes "
            "expressed in the marker frame, e.g. go2_base:x,y,z or piper_ee:z,-y,x. "
            "Repeat once per tracker."
        ),
    )
    parser.add_argument(
        "--local-offset",
        action="append",
        default=[],
        metavar="TRACKER:X,Y,Z",
        help=(
            "Apply a per-tracker local translation offset in marker-frame meters before publishing "
            "the corrected frame. Repeat once per tracker."
        ),
    )
    parser.add_argument(
        "--local-rpy-deg",
        action="append",
        default=[],
        metavar="TRACKER:ROLL,PITCH,YAW",
        help=(
            "Apply an additional per-tracker local RPY correction in degrees after --local-axis-map. "
            "Repeat once per tracker."
        ),
    )
    parser.add_argument(
        "--child-offset",
        action="append",
        default=[],
        metavar="TRACKER:X,Y,Z",
        help=(
            "Apply a per-tracker translation offset in the final corrected child frame, in meters. "
            "Repeat once per tracker."
        ),
    )
    parser.add_argument(
        "--set-world-from",
        default="",
        metavar="TRACKER",
        help=(
            "Set the displayed planar world frame from the first corrected TRACKER sample. "
            "Only x, y, and yaw are reset; z, roll, and pitch are kept."
        ),
    )
    parser.add_argument(
        "--set-world-from-base",
        action="store_true",
        help="Shortcut for --set-world-from go2_base.",
    )
    parser.add_argument("--status-rate", type=float, default=1.0, help="Console status print rate in Hz")
    return parser.parse_args()


def axis_vector(term):
    term = term.strip().lower()
    sign = 1.0
    if term.startswith("-"):
        sign = -1.0
        term = term[1:]
    elif term.startswith("+"):
        term = term[1:]
    axis_index = {"x": 0, "y": 1, "z": 2}
    if term not in axis_index:
        raise ValueError(f"invalid axis term {term!r}")
    vector = [0.0, 0.0, 0.0]
    vector[axis_index[term]] = sign
    return term, vector


def parse_axis_map(value):
    terms = [term.strip().lower() for term in value.split(",")]
    if len(terms) != 3:
        raise ValueError("--axis-map expects three comma-separated terms, e.g. z,-y,x")

    matrix = []
    used_axes = []
    for term in terms:
        axis, vector = axis_vector(term)
        used_axes.append(axis)
        matrix.append(vector)

    if len(set(used_axes)) != 3:
        raise ValueError("--axis-map must use x, y, and z exactly once")
    determinant = det3(matrix)
    if abs(determinant - 1.0) > 1e-9:
        raise ValueError(
            f"--axis-map must be a proper right-handed rotation with determinant +1, got det={determinant}"
        )
    return matrix


def parse_local_axis_map(value):
    name, separator, axis_map_text = value.partition(":")
    if not separator or not name or not axis_map_text:
        raise ValueError(f"expected TRACKER:MAP for --local-axis-map, got {value!r}")
    terms = [term.strip().lower() for term in axis_map_text.split(",")]
    if len(terms) != 3:
        raise ValueError(f"local axis map for {name!r} must have three terms")

    columns = []
    used_axes = []
    for term in terms:
        axis, vector = axis_vector(term)
        used_axes.append(axis)
        columns.append(vector)
    if len(set(used_axes)) != 3:
        raise ValueError(f"local axis map for {name!r} must use x, y, and z exactly once")

    matrix = [
        [columns[0][0], columns[1][0], columns[2][0]],
        [columns[0][1], columns[1][1], columns[2][1]],
        [columns[0][2], columns[1][2], columns[2][2]],
    ]
    determinant = det3(matrix)
    if abs(determinant - 1.0) > 1e-9:
        raise ValueError(
            f"local axis map for {name!r} must be right-handed with determinant +1, got det={determinant}"
        )
    return name, matrix


def parse_local_offset(value):
    name, separator, offset_text = value.partition(":")
    if not separator or not name or not offset_text:
        raise ValueError(f"expected TRACKER:X,Y,Z for --local-offset, got {value!r}")
    fields = [field.strip() for field in offset_text.split(",")]
    if len(fields) != 3:
        raise ValueError(f"local offset for {name!r} must have three values")
    return name, [float(field) for field in fields]


def parse_tracker_vector(value, option_name):
    name, separator, vector_text = value.partition(":")
    if not separator or not name or not vector_text:
        raise ValueError(f"expected TRACKER:X,Y,Z for {option_name}, got {value!r}")
    fields = [field.strip() for field in vector_text.split(",")]
    if len(fields) != 3:
        raise ValueError(f"{option_name} for {name!r} must have three values")
    return name, [float(field) for field in fields]


def det3(matrix):
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def mat_vec(matrix, vector):
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def quat_normalize_xyzw(quat):
    norm = math.sqrt(sum(value * value for value in quat))
    if norm < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / norm for value in quat]


def quat_multiply_xyzw(lhs, rhs):
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


def quat_from_rpy_deg_xyzw(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr = math.cos(0.5 * roll)
    sr = math.sin(0.5 * roll)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    return quat_normalize_xyzw([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quat_from_yaw_xyzw(yaw_rad):
    return [0.0, 0.0, math.sin(0.5 * yaw_rad), math.cos(0.5 * yaw_rad)]


def yaw_from_quat_xyzw(quat):
    x, y, z, w = quat_normalize_xyzw(quat)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def quat_conjugate_xyzw(quat):
    return [-quat[0], -quat[1], -quat[2], quat[3]]


def quat_rotate_xyzw(quat, vector):
    q = quat_normalize_xyzw(quat)
    rotated = quat_multiply_xyzw(
        quat_multiply_xyzw(q, [vector[0], vector[1], vector[2], 0.0]),
        quat_conjugate_xyzw(q),
    )
    return [rotated[0], rotated[1], rotated[2]]


def quat_from_matrix_xyzw(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2][1] - matrix[1][2]) / scale
        qy = (matrix[0][2] - matrix[2][0]) / scale
        qz = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        qw = (matrix[2][1] - matrix[1][2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0][1] + matrix[1][0]) / scale
        qz = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        qw = (matrix[0][2] - matrix[2][0]) / scale
        qx = (matrix[0][1] + matrix[1][0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        qw = (matrix[1][0] - matrix[0][1]) / scale
        qx = (matrix[0][2] + matrix[2][0]) / scale
        qy = (matrix[1][2] + matrix[2][1]) / scale
        qz = 0.25 * scale
    return quat_normalize_xyzw([qx, qy, qz, qw])


def parse_host_port(value):
    host, separator, port_text = value.rpartition(":")
    if not separator or not host or not port_text:
        raise ValueError(f"expected HOST:PORT, got {value!r}")
    port = int(port_text)
    if port <= 0 or port > 65535:
        raise ValueError(f"invalid port in {value!r}")
    return host, port


def sanitize_frame_id(name):
    cleaned = []
    for char in name:
        if char.isalnum() or char in ("_", "-", "/"):
            cleaned.append(char)
        else:
            cleaned.append("_")
    return "".join(cleaned).strip("/") or "tracker"


def make_transform(parent, child, position, quaternion_xyzw, stamp):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = float(position[0])
    transform.transform.translation.y = float(position[1])
    transform.transform.translation.z = float(position[2])
    transform.transform.rotation.x = float(quaternion_xyzw[0])
    transform.transform.rotation.y = float(quaternion_xyzw[1])
    transform.transform.rotation.z = float(quaternion_xyzw[2])
    transform.transform.rotation.w = float(quaternion_xyzw[3])
    return transform


def make_udp_socket(bind_host, udp_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_host, udp_port))
    sock.setblocking(False)
    return sock


def initialize_planar_world_alignment(state, tracker_name, position, quaternion_xyzw):
    state["ready"] = True
    state["tracker_name"] = tracker_name
    state["origin_x"] = position[0]
    state["origin_y"] = position[1]
    state["yaw"] = yaw_from_quat_xyzw(quaternion_xyzw)
    state["rotation"] = quat_from_yaw_xyzw(-state["yaw"])


def apply_planar_world_alignment(state, position, quaternion_xyzw):
    if not state["ready"]:
        return position, quaternion_xyzw
    dx = position[0] - state["origin_x"]
    dy = position[1] - state["origin_y"]
    c = math.cos(-state["yaw"])
    s = math.sin(-state["yaw"])
    aligned_position = [
        c * dx - s * dy,
        s * dx + c * dy,
        position[2],
    ]
    aligned_quaternion = quat_normalize_xyzw(
        quat_multiply_xyzw(state["rotation"], quaternion_xyzw)
    )
    return aligned_position, aligned_quaternion


def main():
    args = parse_args()
    if args.set_world_from_base:
        if args.set_world_from and args.set_world_from != "go2_base":
            raise ValueError("--set-world-from-base and --set-world-from disagree")
        args.set_world_from = "go2_base"

    axis_map = parse_axis_map(args.axis_map)
    axis_map_quat = quat_from_matrix_xyzw(axis_map)
    local_axis_maps = {}
    for value in args.local_axis_map:
        tracker_name, matrix = parse_local_axis_map(value)
        local_axis_maps[tracker_name] = quat_from_matrix_xyzw(matrix)
    local_offsets = {}
    for value in args.local_offset:
        tracker_name, offset = parse_local_offset(value)
        local_offsets[tracker_name] = offset
    local_rpy_quats = {}
    for value in args.local_rpy_deg:
        tracker_name, rpy_deg = parse_tracker_vector(value, "--local-rpy-deg")
        local_rpy_quats[tracker_name] = quat_from_rpy_deg_xyzw(*rpy_deg)
    child_offsets = {}
    for value in args.child_offset:
        tracker_name, offset = parse_tracker_vector(value, "--child-offset")
        child_offsets[tracker_name] = offset
    tracker_filter = {
        name.strip() for name in args.trackers.split(",") if name.strip()
    }

    rclpy.init()
    node = rclpy.create_node("optitrack_udp_tf")
    tf_broadcaster = TransformBroadcaster(node)
    static_broadcaster = StaticTransformBroadcaster(node)

    zero = [0.0, 0.0, 0.0]
    identity = [0.0, 0.0, 0.0, 1.0]
    static_broadcaster.sendTransform(
        make_transform(
            args.world_frame,
            sanitize_frame_id(args.origin_frame),
            zero,
            identity,
            node.get_clock().now().to_msg(),
        )
    )

    sock = make_udp_socket(args.bind, args.udp_port)
    node.get_logger().info(
        f"Listening on {args.bind}:{args.udp_port}, publishing {args.frame} poses as TF under "
        f"{args.world_frame}, axis_map={args.axis_map}"
    )
    if tracker_filter:
        node.get_logger().info(f"Tracker filter: {sorted(tracker_filter)}")
    if local_axis_maps:
        node.get_logger().info(
            "Local tracker corrections: " + ", ".join(sorted(local_axis_maps.keys()))
        )
    if local_offsets:
        node.get_logger().info(
            "Local tracker offsets: " + ", ".join(
                f"{name}={offset}" for name, offset in sorted(local_offsets.items())
            )
        )
    if local_rpy_quats:
        node.get_logger().info(
            "Local tracker RPY corrections: " + ", ".join(sorted(local_rpy_quats.keys()))
        )
    if child_offsets:
        node.get_logger().info(
            "Child-frame offsets: " + ", ".join(
                f"{name}={offset}" for name, offset in sorted(child_offsets.items())
            )
        )
    forward_targets = [parse_host_port(value) for value in args.forward]
    if forward_targets:
        node.get_logger().info(f"Forwarding raw UDP packets to: {forward_targets}")
    corrected_forward_targets = [parse_host_port(value) for value in args.forward_corrected]
    if corrected_forward_targets:
        node.get_logger().info(f"Forwarding corrected UDP packets to: {corrected_forward_targets}")
    world_alignment = {
        "enabled": bool(args.set_world_from),
        "ready": False,
        "tracker_name": args.set_world_from,
        "origin_x": 0.0,
        "origin_y": 0.0,
        "yaw": 0.0,
        "rotation": [0.0, 0.0, 0.0, 1.0],
    }
    if world_alignment["enabled"]:
        node.get_logger().info(
            f"Planar set-world enabled from {world_alignment['tracker_name']} "
            "(resets x/y/yaw only)"
        )

    last_status_wall = 0.0
    counts = {}
    last_packet_wall = {}
    last_corrected = {}

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            readable, _, _ = select.select([sock], [], [], 0.05)
            if not readable:
                continue

            data, _ = sock.recvfrom(65535)
            for target in forward_targets:
                sock.sendto(data, target)
            try:
                packet = json.loads(data.decode("utf-8"))
                tracker_name = packet["name"]
                if tracker_filter and tracker_name not in tracker_filter:
                    continue
                pose = packet[args.frame]
                position = mat_vec(axis_map, pose["p"])
                marker_quaternion = quat_normalize_xyzw(
                    quat_multiply_xyzw(axis_map_quat, pose["q_xyzw"])
                )
            except (KeyError, ValueError, json.JSONDecodeError) as exc:
                node.get_logger().warn(f"Skipping malformed UDP packet: {exc}")
                continue

            stamp = node.get_clock().now().to_msg()

            local_offset = local_offsets.get(tracker_name, [0.0, 0.0, 0.0])
            corrected_position_delta = quat_rotate_xyzw(marker_quaternion, local_offset)
            corrected_position = [
                position[0] + corrected_position_delta[0],
                position[1] + corrected_position_delta[1],
                position[2] + corrected_position_delta[2],
            ]
            local_quaternion = local_axis_maps.get(tracker_name, [0.0, 0.0, 0.0, 1.0])
            if tracker_name in local_rpy_quats:
                local_quaternion = quat_normalize_xyzw(
                    quat_multiply_xyzw(local_quaternion, local_rpy_quats[tracker_name])
                )
            corrected_quaternion = quat_normalize_xyzw(
                quat_multiply_xyzw(marker_quaternion, local_quaternion)
            )
            child_offset = child_offsets.get(tracker_name, [0.0, 0.0, 0.0])
            child_position_delta = quat_rotate_xyzw(corrected_quaternion, child_offset)
            corrected_position = [
                corrected_position[0] + child_position_delta[0],
                corrected_position[1] + child_position_delta[1],
                corrected_position[2] + child_position_delta[2],
            ]

            if world_alignment["enabled"]:
                if not world_alignment["ready"]:
                    if tracker_name != world_alignment["tracker_name"]:
                        continue
                    initialize_planar_world_alignment(
                        world_alignment,
                        tracker_name,
                        corrected_position,
                        corrected_quaternion,
                    )
                    node.get_logger().info(
                        "Planar set-world initialized from "
                        f"{tracker_name}: origin_xy=({world_alignment['origin_x']:.6f}, "
                        f"{world_alignment['origin_y']:.6f}) "
                        f"yaw_deg={math.degrees(world_alignment['yaw']):.6f}"
                    )
                position, marker_quaternion = apply_planar_world_alignment(
                    world_alignment,
                    position,
                    marker_quaternion,
                )
                corrected_position, corrected_quaternion = apply_planar_world_alignment(
                    world_alignment,
                    corrected_position,
                    corrected_quaternion,
                )

            now_wall = time.monotonic()
            corrected_velocity = [0.0, 0.0, 0.0]
            prev = last_corrected.get(tracker_name)
            if prev is not None:
                dt = max(now_wall - prev["time"], 1.0e-6)
                corrected_velocity = [
                    (corrected_position[0] - prev["position"][0]) / dt,
                    (corrected_position[1] - prev["position"][1]) / dt,
                    (corrected_position[2] - prev["position"][2]) / dt,
                ]
            last_corrected[tracker_name] = {
                "time": now_wall,
                "position": list(corrected_position),
            }

            if corrected_forward_targets:
                corrected_packet = {
                    "name": tracker_name,
                    "sequence": packet.get("sequence", 0),
                    "timestamp_s": packet.get("timestamp_s", time.time()),
                    "source_timestamp_s": packet.get("source_timestamp_s", 0.0),
                    "corrected": True,
                    args.frame: {
                        "p": corrected_position,
                        "q_xyzw": corrected_quaternion,
                        "v": corrected_velocity,
                    },
                }
                payload = (json.dumps(corrected_packet, separators=(",", ":")) + "\n").encode("utf-8")
                for target in corrected_forward_targets:
                    sock.sendto(payload, target)

            corrected_frame = sanitize_frame_id(args.frame_prefix + tracker_name)
            tf_broadcaster.sendTransform(
                make_transform(
                    args.world_frame,
                    corrected_frame,
                    corrected_position,
                    corrected_quaternion,
                    stamp,
                )
            )
            if args.show_marker_frames:
                marker_frame = sanitize_frame_id(args.frame_prefix + tracker_name + "_marker")
                tf_broadcaster.sendTransform(
                    make_transform(args.world_frame, marker_frame, position, marker_quaternion, stamp)
                )

            counts[tracker_name] = counts.get(tracker_name, 0) + 1
            last_packet_wall[tracker_name] = now_wall

            if args.status_rate > 0.0 and now_wall - last_status_wall >= 1.0 / args.status_rate:
                last_status_wall = now_wall
                status = []
                for name in sorted(counts):
                    age = now_wall - last_packet_wall[name]
                    status.append(f"{name}:count={counts[name]} age={age:.3f}s")
                node.get_logger().info(" | ".join(status))
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
