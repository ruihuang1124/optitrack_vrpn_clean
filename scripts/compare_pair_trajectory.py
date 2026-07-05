#!/usr/bin/env python3
import argparse
import csv
import math
import statistics

try:
    import numpy as np
except ImportError as exc:
    raise SystemExit(
        "compare_pair_trajectory.py requires numpy. Install it with: "
        "sudo apt install python3-numpy"
    ) from exc


POSITION_COLUMNS = ("ee_in_base_x", "ee_in_base_y", "ee_in_base_z")
QUATERNION_COLUMNS_XYZW = (
    "ee_in_base_qx",
    "ee_in_base_qy",
    "ee_in_base_qz",
    "ee_in_base_qw",
)
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def load_rows(path):
    rows = []
    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            parsed = {}
            for key, value in row.items():
                try:
                    parsed[key] = float(value)
                except (TypeError, ValueError):
                    pass
            rows.append(parsed)
    return rows


def filter_rows_by_time(rows, start_s=None, end_s=None, last_s=None):
    if not rows:
        return rows
    if "time_s" not in rows[0]:
        return rows

    file_t0 = rows[0]["time_s"]
    file_t1 = rows[-1]["time_s"]
    if last_s is not None:
        abs_start = file_t1 - last_s
        abs_end = file_t1
    else:
        abs_start = file_t0 + (start_s if start_s is not None else 0.0)
        abs_end = file_t0 + end_s if end_s is not None else file_t1

    return [
        row for row in rows
        if "time_s" in row and abs_start <= row["time_s"] <= abs_end
    ]


def time_summary(rows):
    if not rows or "time_s" not in rows[0]:
        return "empty"
    t0 = rows[0]["time_s"]
    t1 = rows[-1]["time_s"]
    duration = t1 - t0
    if duration > 0.0 and len(rows) > 1:
        hz = (len(rows) - 1) / duration
        return f"rows={len(rows)} t0={t0:.6f} t1={t1:.6f} duration={duration:.3f}s mean_hz={hz:.3f}"
    return f"rows={len(rows)} t0={t0:.6f} t1={t1:.6f} duration={duration:.3f}s"


def make_series(rows):
    if not rows:
        raise ValueError("empty row selection")
    t0 = rows[0]["time_s"]
    times = np.array([row["time_s"] - t0 for row in rows], dtype=float)
    positions = np.array(
        [[row[column] for column in POSITION_COLUMNS] for row in rows],
        dtype=float,
    )
    quats = np.array(
        [[row[column] for column in QUATERNION_COLUMNS_XYZW] for row in rows],
        dtype=float,
    )
    quats = normalize_quat_array(quats)
    quats = make_quat_continuous(quats)
    return {"times": times, "positions": positions, "quats": quats}


def normalize_quat(quat):
    quat = np.asarray(quat, dtype=float)
    norm = float(np.linalg.norm(quat))
    if norm < 1.0e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quat / norm


def normalize_quat_array(quats):
    normalized = np.empty_like(quats)
    for index, quat in enumerate(quats):
        normalized[index] = normalize_quat(quat)
    return normalized


def make_quat_continuous(quats):
    if len(quats) == 0:
        return quats
    continuous = quats.copy()
    for index in range(1, len(continuous)):
        if float(np.dot(continuous[index - 1], continuous[index])) < 0.0:
            continuous[index] *= -1.0
    return continuous


def quat_multiply(lhs, rhs):
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return np.array([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ], dtype=float)


def quat_conjugate(quat):
    return np.array([-quat[0], -quat[1], -quat[2], quat[3]], dtype=float)


def quat_slerp(lhs, rhs, fraction):
    lhs = normalize_quat(lhs)
    rhs = normalize_quat(rhs)
    dot = float(np.dot(lhs, rhs))
    if dot < 0.0:
        rhs = -rhs
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_quat(lhs + fraction * (rhs - lhs))

    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    left_scale = math.sin((1.0 - fraction) * theta) / sin_theta
    right_scale = math.sin(fraction * theta) / sin_theta
    return normalize_quat(left_scale * lhs + right_scale * rhs)


def quat_angle_deg(lhs, rhs):
    lhs = normalize_quat(lhs)
    rhs = normalize_quat(rhs)
    cosine = abs(float(np.dot(lhs, rhs)))
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(2.0 * math.acos(cosine))


def quat_to_matrix(quat):
    x, y, z, w = normalize_quat(quat)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=float)


def quat_to_rpy_deg(quat):
    x, y, z, w = normalize_quat(quat)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return np.degrees([roll, pitch, yaw])


def mean_quat(quats):
    if len(quats) == 0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    reference = quats[0]
    accum = np.zeros(4, dtype=float)
    for quat in quats:
        if float(np.dot(reference, quat)) < 0.0:
            quat = -quat
        accum += quat
    return normalize_quat(accum)


def interp_positions(series, sample_times):
    positions = np.empty((len(sample_times), 3), dtype=float)
    for axis in range(3):
        positions[:, axis] = np.interp(
            sample_times,
            series["times"],
            series["positions"][:, axis],
        )
    return positions


def interp_quats(series, sample_times):
    times = series["times"]
    quats = series["quats"]
    indices = np.searchsorted(times, sample_times, side="right") - 1
    out = np.empty((len(sample_times), 4), dtype=float)
    for out_index, (sample_time, index) in enumerate(zip(sample_times, indices)):
        if index < 0:
            out[out_index] = quats[0]
            continue
        if index >= len(times) - 1:
            out[out_index] = quats[-1]
            continue
        dt = times[index + 1] - times[index]
        fraction = 0.0 if dt <= 0.0 else (sample_time - times[index]) / dt
        out[out_index] = quat_slerp(quats[index], quats[index + 1], fraction)
    return make_quat_continuous(out)


def common_grid(real, sim, offset_s, rate_hz, min_overlap_s):
    # Offset convention: real_time = sim_time + offset_s.
    start = max(sim["times"][0], real["times"][0] - offset_s)
    end = min(sim["times"][-1], real["times"][-1] - offset_s)
    if end - start < min_overlap_s:
        return None
    step = 1.0 / rate_hz
    sample_times = np.arange(start, end, step, dtype=float)
    if len(sample_times) < 2:
        return None
    return sample_times


def parse_align_axes(text):
    axes = []
    for item in text.split(","):
        item = item.strip().lower()
        if not item:
            continue
        if item not in AXIS_INDEX:
            raise ValueError("--align-cols must contain x, y, z entries")
        axes.append(AXIS_INDEX[item])
    if not axes:
        raise ValueError("--align-cols selected no axes")
    return axes


def normalized_shape_score(real_pos, sim_pos, axes):
    real_signal = real_pos[:, axes]
    sim_signal = sim_pos[:, axes]
    real_std = real_signal.std(axis=0)
    sim_std = sim_signal.std(axis=0)
    usable = (real_std > 1.0e-8) & (sim_std > 1.0e-8)
    if not np.any(usable):
        return float("inf")
    real_norm = (real_signal[:, usable] - real_signal[:, usable].mean(axis=0)) / real_std[usable]
    sim_norm = (sim_signal[:, usable] - sim_signal[:, usable].mean(axis=0)) / sim_std[usable]
    return float(np.sqrt(np.mean((sim_norm - real_norm) ** 2)))


def find_best_offset(real, sim, args, axes):
    if args.real_time_offset is not None:
        return args.real_time_offset, None

    best = (float("inf"), 0.0)
    offsets = np.arange(-args.max_offset, args.max_offset + 0.5 * args.offset_step, args.offset_step)
    for offset_s in offsets:
        grid = common_grid(real, sim, offset_s, args.rate, args.min_overlap)
        if grid is None:
            continue
        real_pos = interp_positions(real, grid + offset_s)
        sim_pos = interp_positions(sim, grid)
        score = normalized_shape_score(real_pos, sim_pos, axes)
        if score < best[0]:
            best = (score, float(offset_s))

    if not math.isfinite(best[0]):
        raise RuntimeError("could not find a valid alignment window")

    fine_step = args.offset_step / 5.0
    fine_start = best[1] - args.offset_step
    fine_end = best[1] + args.offset_step
    fine_offsets = np.arange(fine_start, fine_end + 0.5 * fine_step, fine_step)
    for offset_s in fine_offsets:
        grid = common_grid(real, sim, offset_s, args.rate, args.min_overlap)
        if grid is None:
            continue
        real_pos = interp_positions(real, grid + offset_s)
        sim_pos = interp_positions(sim, grid)
        score = normalized_shape_score(real_pos, sim_pos, axes)
        if score < best[0]:
            best = (score, float(offset_s))
    return best[1], best[0]


def vector_stats(values):
    bias = values.mean(axis=0)
    std = values.std(axis=0)
    rmse = np.sqrt(np.mean(values * values, axis=0))
    max_abs = np.max(np.abs(values), axis=0)
    p95_abs = np.percentile(np.abs(values), 95.0, axis=0)
    return bias, std, rmse, max_abs, p95_abs


def format_vec(values, precision=6):
    return "(" + ", ".join(f"{float(value):.{precision}f}" for value in values) + ")"


def compute_child_offset_hint(real_quats, diffs):
    local_offsets = []
    for quat, diff in zip(real_quats, diffs):
        rotation = quat_to_matrix(quat)
        local_offsets.append(rotation.T @ diff)
    local_offsets = np.array(local_offsets, dtype=float)
    hint = local_offsets.mean(axis=0)
    corrected_diffs = []
    for quat, diff in zip(real_quats, diffs):
        rotation = quat_to_matrix(quat)
        corrected_diffs.append(diff - rotation @ hint)
    return hint, np.array(corrected_diffs, dtype=float)


def compute_rotation_hints(real_quats, sim_quats):
    local_residuals = []
    parent_residuals = []
    for real_quat, sim_quat in zip(real_quats, sim_quats):
        local = normalize_quat(quat_multiply(quat_conjugate(real_quat), sim_quat))
        parent = normalize_quat(quat_multiply(sim_quat, quat_conjugate(real_quat)))
        if local_residuals and float(np.dot(local_residuals[0], local)) < 0.0:
            local = -local
        if parent_residuals and float(np.dot(parent_residuals[0], parent)) < 0.0:
            parent = -parent
        local_residuals.append(local)
        parent_residuals.append(parent)
    return mean_quat(local_residuals), mean_quat(parent_residuals)


def write_aligned_csv(path, sim_times, real_times, real_pos, sim_pos, diffs, angle_errors):
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "sim_time_s",
            "real_time_s",
            "real_x",
            "real_y",
            "real_z",
            "sim_x",
            "sim_y",
            "sim_z",
            "diff_x",
            "diff_y",
            "diff_z",
            "orientation_error_deg",
        ])
        for index in range(len(sim_times)):
            writer.writerow([
                f"{sim_times[index]:.9f}",
                f"{real_times[index]:.9f}",
                f"{real_pos[index, 0]:.9f}",
                f"{real_pos[index, 1]:.9f}",
                f"{real_pos[index, 2]:.9f}",
                f"{sim_pos[index, 0]:.9f}",
                f"{sim_pos[index, 1]:.9f}",
                f"{sim_pos[index, 2]:.9f}",
                f"{diffs[index, 0]:.9f}",
                f"{diffs[index, 1]:.9f}",
                f"{diffs[index, 2]:.9f}",
                f"{angle_errors[index]:.9f}",
            ])


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Time-align and compare real mocap and MuJoCo base->EE trajectory CSV files. "
            "Position errors are reported as sim minus real in the base frame."
        )
    )
    parser.add_argument("--real", required=True, help="Real mocap CSV path")
    parser.add_argument("--sim", required=True, help="MuJoCo CSV path")
    parser.add_argument("--real-start", type=float, default=0.0, help="Real CSV start offset in seconds from its first sample")
    parser.add_argument("--real-end", type=float, help="Real CSV end offset in seconds from its first sample")
    parser.add_argument("--real-last", type=float, help="Use only the last N seconds of the real CSV")
    parser.add_argument("--sim-start", type=float, default=0.0, help="MuJoCo CSV start offset in seconds from its first sample")
    parser.add_argument("--sim-end", type=float, help="MuJoCo CSV end offset in seconds from its first sample")
    parser.add_argument("--sim-last", type=float, help="Use only the last N seconds of the MuJoCo CSV")
    parser.add_argument("--rate", type=float, default=50.0, help="Common comparison sample rate, default 50")
    parser.add_argument("--align-cols", default="x,z", help="Position axes used for shape alignment, default x,z")
    parser.add_argument("--max-offset", type=float, default=20.0, help="Search +/- this many seconds for time alignment")
    parser.add_argument("--offset-step", type=float, default=0.02, help="Coarse offset search step in seconds")
    parser.add_argument("--min-overlap", type=float, default=10.0, help="Minimum accepted aligned overlap in seconds")
    parser.add_argument(
        "--real-time-offset",
        type=float,
        help="Manual offset in seconds using convention: real_time = sim_time + offset",
    )
    parser.add_argument("--out-aligned", help="Optional CSV path for the aligned sample-by-sample comparison")
    args = parser.parse_args()

    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")
    if args.offset_step <= 0.0:
        raise SystemExit("--offset-step must be positive")
    if args.max_offset < 0.0:
        raise SystemExit("--max-offset must be non-negative")
    axes = parse_align_axes(args.align_cols)

    real_rows_all = load_rows(args.real)
    sim_rows_all = load_rows(args.sim)
    real_rows = filter_rows_by_time(real_rows_all, args.real_start, args.real_end, args.real_last)
    sim_rows = filter_rows_by_time(sim_rows_all, args.sim_start, args.sim_end, args.sim_last)
    if not real_rows:
        raise SystemExit(f"No rows selected from {args.real}")
    if not sim_rows:
        raise SystemExit(f"No rows selected from {args.sim}")

    real = make_series(real_rows)
    sim = make_series(sim_rows)
    offset_s, align_score = find_best_offset(real, sim, args, axes)
    grid = common_grid(real, sim, offset_s, args.rate, args.min_overlap)
    if grid is None:
        raise SystemExit("Aligned overlap is too short; adjust window or --min-overlap")

    real_times = grid + offset_s
    sim_times = grid
    real_pos = interp_positions(real, real_times)
    sim_pos = interp_positions(sim, sim_times)
    real_quats = interp_quats(real, real_times)
    sim_quats = interp_quats(sim, sim_times)

    diffs = sim_pos - real_pos
    bias, std, rmse, max_abs, p95_abs = vector_stats(diffs)
    centered = diffs - bias
    centered_rmse = np.sqrt(np.mean(centered * centered, axis=0))
    norm_rmse = math.sqrt(float(np.mean(np.sum(diffs * diffs, axis=1))))

    angle_errors = np.array([
        quat_angle_deg(real_quat, sim_quat)
        for real_quat, sim_quat in zip(real_quats, sim_quats)
    ], dtype=float)
    local_rotation, parent_rotation = compute_rotation_hints(real_quats, sim_quats)
    child_offset_hint, child_corrected_diffs = compute_child_offset_hint(real_quats, diffs)
    child_bias, child_std, child_rmse, child_max_abs, child_p95_abs = vector_stats(child_corrected_diffs)

    print(f"real_all: {time_summary(real_rows_all)}")
    print(f"sim_all : {time_summary(sim_rows_all)}")
    print(f"real_sel: {time_summary(real_rows)}")
    print(f"sim_sel : {time_summary(sim_rows)}")
    print()
    print("alignment:")
    print(f"  convention: real_time = sim_time + offset_s")
    print(f"  offset_s={offset_s:.6f}")
    if align_score is not None:
        print(f"  normalized_shape_score={align_score:.6f}")
    print(f"  align_cols={args.align_cols}")
    print(f"  overlap_sim_time_s=({grid[0]:.6f}, {grid[-1]:.6f})")
    print(f"  overlap_real_time_s=({real_times[0]:.6f}, {real_times[-1]:.6f})")
    print(f"  overlap_duration_s={grid[-1] - grid[0]:.6f}")
    print(f"  samples={len(grid)} rate_hz={args.rate:.3f}")
    print()
    print("position_error_sim_minus_real_m:")
    print("axis,bias,rmse,std,max_abs,p95_abs")
    for axis, name in enumerate(("x", "y", "z")):
        print(
            f"{name},{bias[axis]:.9f},{rmse[axis]:.9f},{std[axis]:.9f},"
            f"{max_abs[axis]:.9f},{p95_abs[axis]:.9f}"
        )
    print(f"norm_rmse_m={norm_rmse:.9f}")
    print(f"mean_bias_m={format_vec(bias)}")
    print(f"centered_rmse_after_parent_bias_m={format_vec(centered_rmse)}")
    print()
    print("orientation_error_deg:")
    print(f"  mean={float(angle_errors.mean()):.6f}")
    print(f"  rms={math.sqrt(float(np.mean(angle_errors * angle_errors))):.6f}")
    print(f"  std={float(angle_errors.std()):.6f}")
    print(f"  max={float(angle_errors.max()):.6f}")
    print(f"  p95={float(np.percentile(angle_errors, 95.0)):.6f}")
    print()
    print("offset_hints:")
    print("  parent_frame_bias_sim_minus_real_m=" + format_vec(bias))
    print("  child_offset_hint_piper_ee_m=" + format_vec(child_offset_hint))
    print("  after_child_offset_residual_bias_m=" + format_vec(child_bias))
    print("  after_child_offset_residual_rmse_m=" + format_vec(child_rmse))
    print()
    print("rotation_hints:")
    print("  local_extra_rotation_real_to_sim_xyzw=" + format_vec(local_rotation))
    print("  local_extra_rpy_deg_real_to_sim=" + format_vec(quat_to_rpy_deg(local_rotation)))
    print("  parent_extra_rotation_real_to_sim_xyzw=" + format_vec(parent_rotation))
    print("  parent_extra_rpy_deg_real_to_sim=" + format_vec(quat_to_rpy_deg(parent_rotation)))
    print()
    print(
        "Note: offset/rotation hints assume the real trajectory should be moved toward "
        "the MuJoCo trajectory. Use several trajectories before treating them as deploy parameters."
    )

    if args.out_aligned:
        write_aligned_csv(args.out_aligned, sim_times, real_times, real_pos, sim_pos, diffs, angle_errors)
        print(f"wrote_aligned_csv={args.out_aligned}")


if __name__ == "__main__":
    main()
