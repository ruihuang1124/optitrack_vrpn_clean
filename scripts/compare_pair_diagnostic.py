#!/usr/bin/env python3
import argparse
import csv
import math
import statistics


POSITION_COLUMNS = ("ee_in_base_x", "ee_in_base_y", "ee_in_base_z")
QUATERNION_COLUMNS_XYZW = (
    "ee_in_base_qx",
    "ee_in_base_qy",
    "ee_in_base_qz",
    "ee_in_base_qw",
)


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


def column_values(rows, column):
    return [row[column] for row in rows if column in row and math.isfinite(row[column])]


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


def mean(values):
    if not values:
        return float("nan")
    return statistics.fmean(values)


def std(values):
    if len(values) < 2:
        return 0.0
    return statistics.pstdev(values)


def normalize_quat_xyzw(quat):
    norm = math.sqrt(sum(value * value for value in quat))
    if norm < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / norm for value in quat]


def dot(lhs, rhs):
    return sum(a * b for a, b in zip(lhs, rhs))


def mean_quat_xyzw(rows):
    quats = []
    for row in rows:
        if all(column in row for column in QUATERNION_COLUMNS_XYZW):
            quat = [row[column] for column in QUATERNION_COLUMNS_XYZW]
            if all(math.isfinite(value) for value in quat):
                quats.append(normalize_quat_xyzw(quat))
    if not quats:
        return [0.0, 0.0, 0.0, 1.0]

    reference = quats[0]
    accum = [0.0, 0.0, 0.0, 0.0]
    for quat in quats:
        if dot(reference, quat) < 0.0:
            quat = [-value for value in quat]
        for index, value in enumerate(quat):
            accum[index] += value
    return normalize_quat_xyzw(accum)


def quat_angle_deg_xyzw(lhs, rhs):
    lhs = normalize_quat_xyzw(lhs)
    rhs = normalize_quat_xyzw(rhs)
    cosine = max(-1.0, min(1.0, abs(dot(lhs, rhs))))
    return math.degrees(2.0 * math.acos(cosine))


def rotate_z(vector, yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return [
        c * vector[0] - s * vector[1],
        s * vector[0] + c * vector[1],
        vector[2],
    ]


def format_vec(values, precision=6):
    return "(" + ", ".join(f"{value:.{precision}f}" for value in values) + ")"


def main():
    parser = argparse.ArgumentParser(
        description="Compare real mocap and MuJoCo base->EE pair diagnostic CSV files."
    )
    parser.add_argument("--real", required=True, help="Real mocap CSV path")
    parser.add_argument("--sim", required=True, help="MuJoCo CSV path")
    parser.add_argument("--real-start", type=float, default=0.0, help="Real CSV start offset in seconds from its first sample")
    parser.add_argument("--real-end", type=float, help="Real CSV end offset in seconds from its first sample")
    parser.add_argument("--real-last", type=float, help="Use only the last N seconds of the real CSV")
    parser.add_argument("--sim-start", type=float, default=0.0, help="MuJoCo CSV start offset in seconds from its first sample")
    parser.add_argument("--sim-end", type=float, help="MuJoCo CSV end offset in seconds from its first sample")
    parser.add_argument("--sim-last", type=float, help="Use only the last N seconds of the MuJoCo CSV")
    args = parser.parse_args()

    real_rows_all = load_rows(args.real)
    sim_rows_all = load_rows(args.sim)
    real_rows = filter_rows_by_time(
        real_rows_all,
        start_s=args.real_start,
        end_s=args.real_end,
        last_s=args.real_last)
    sim_rows = filter_rows_by_time(
        sim_rows_all,
        start_s=args.sim_start,
        end_s=args.sim_end,
        last_s=args.sim_last)
    if not real_rows:
        raise SystemExit(f"No rows selected from {args.real}")
    if not sim_rows:
        raise SystemExit(f"No rows selected from {args.sim}")

    print(f"real_all: {time_summary(real_rows_all)}")
    print(f"sim_all : {time_summary(sim_rows_all)}")
    print(f"real_sel: {time_summary(real_rows)}")
    print(f"sim_sel : {time_summary(sim_rows)}")
    print("Note: this compares distributions/means, not time-synchronized samples.")
    print()
    print("column,real_mean,real_std,sim_mean,sim_std,sim_minus_real")
    for column in POSITION_COLUMNS:
        real_values = column_values(real_rows, column)
        sim_values = column_values(sim_rows, column)
        real_mean = mean(real_values)
        sim_mean = mean(sim_values)
        print(
            f"{column},{real_mean:.9f},{std(real_values):.9f},"
            f"{sim_mean:.9f},{std(sim_values):.9f},{sim_mean - real_mean:.9f}"
        )

    real_pos_mean = [mean(column_values(real_rows, column)) for column in POSITION_COLUMNS]
    sim_pos_mean = [mean(column_values(sim_rows, column)) for column in POSITION_COLUMNS]
    diff = [sim_value - real_value for sim_value, real_value in zip(sim_pos_mean, real_pos_mean)]
    diff_norm = math.sqrt(sum(value * value for value in diff))

    real_quat = mean_quat_xyzw(real_rows)
    sim_quat = mean_quat_xyzw(sim_rows)
    angle_deg = quat_angle_deg_xyzw(sim_quat, real_quat)

    print()
    print(f"real_mean_pos_b_m={format_vec(real_pos_mean)}")
    print(f"sim_mean_pos_b_m ={format_vec(sim_pos_mean)}")
    print(f"sim_minus_real_m ={format_vec(diff)} norm={diff_norm:.6f}")
    print(f"real_mean_quat_xyzw={format_vec(real_quat)}")
    print(f"sim_mean_quat_xyzw ={format_vec(sim_quat)}")
    print(f"relative_orientation_error_deg={angle_deg:.6f}")

    real_xy_norm = math.hypot(real_pos_mean[0], real_pos_mean[1])
    sim_xy_norm = math.hypot(sim_pos_mean[0], sim_pos_mean[1])
    if real_xy_norm > 1e-9 and sim_xy_norm > 1e-9:
        yaw_real_to_sim = (
            math.atan2(sim_pos_mean[1], sim_pos_mean[0]) -
            math.atan2(real_pos_mean[1], real_pos_mean[0])
        )
        rotated_real_pos = rotate_z(real_pos_mean, yaw_real_to_sim)
        residual = [
            sim_value - real_value
            for sim_value, real_value in zip(sim_pos_mean, rotated_real_pos)
        ]
        print()
        print("yaw_only_alignment_hint:")
        print(f"  yaw_real_to_sim_deg={math.degrees(yaw_real_to_sim):.6f}")
        print(f"  rotated_real_pos_b_m={format_vec(rotated_real_pos)}")
        print(f"  sim_minus_rotated_real_m={format_vec(residual)}")


if __name__ == "__main__":
    main()
