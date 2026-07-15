#!/usr/bin/env python3
"""Record corrected RViz/TF pose statistics for one OptiTrack rigid body."""

import argparse
import csv
import math
import statistics
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def quat_xyzw_to_rpy_deg(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def mean_angle_deg(values):
    if not values:
        return float("nan")
    sin_sum = sum(math.sin(math.radians(v)) for v in values)
    cos_sum = sum(math.cos(math.radians(v)) for v in values)
    return math.degrees(math.atan2(sin_sum / len(values), cos_sum / len(values)))


def angle_error_deg(reference, measured):
    return mean_angle_deg([reference - measured])


def stddev(values):
    return statistics.pstdev(values) if len(values) > 1 else 0.0


class TfPoseRecorder(Node):
    def __init__(self, args):
        super().__init__("marker_pose_stats_recorder")
        self.args = args
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.rows = []

    def sample_once(self):
        tf = self.buffer.lookup_transform(
            self.args.world_frame,
            self.args.child_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=self.args.lookup_timeout),
        )
        t = tf.transform.translation
        q = tf.transform.rotation
        roll, pitch, yaw = quat_xyzw_to_rpy_deg(q.x, q.y, q.z, q.w)
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1.0e-9
        return {
            "time_s": stamp,
            "x": t.x,
            "y": t.y,
            "z": t.z,
            "qx": q.x,
            "qy": q.y,
            "qz": q.z,
            "qw": q.w,
            "roll_deg": roll,
            "pitch_deg": pitch,
            "yaw_deg": yaw,
        }

    def run(self):
        start = time.monotonic()
        next_sample = start + self.args.settle
        period = 1.0 / self.args.rate
        end = start + self.args.duration if self.args.duration > 0.0 else None

        self.get_logger().info(
            f"Recording TF {self.args.world_frame} -> {self.args.child_frame}; "
            f"duration={self.args.duration:.3f}s rate={self.args.rate:.3f}Hz settle={self.args.settle:.3f}s"
        )

        while rclpy.ok():
            now = time.monotonic()
            if end is not None and now >= end:
                break
            rclpy.spin_once(self, timeout_sec=0.01)
            if now < next_sample:
                continue
            next_sample += period
            try:
                self.rows.append(self.sample_once())
            except TransformException as exc:
                self.get_logger().warn(f"TF lookup failed: {exc}", throttle_duration_sec=1.0)

        if self.args.out:
            with open(self.args.out, "w", newline="") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=[
                        "time_s",
                        "x",
                        "y",
                        "z",
                        "qx",
                        "qy",
                        "qz",
                        "qw",
                        "roll_deg",
                        "pitch_deg",
                        "yaw_deg",
                    ],
                )
                writer.writeheader()
                writer.writerows(self.rows)

        self.print_summary()

    def print_summary(self):
        if not self.rows:
            print("No samples recorded.")
            return

        z_values = [r["z"] for r in self.rows]
        roll_values = [r["roll_deg"] for r in self.rows]
        pitch_values = [r["pitch_deg"] for r in self.rows]
        yaw_values = [r["yaw_deg"] for r in self.rows]

        z_mean = statistics.fmean(z_values)
        roll_mean = mean_angle_deg(roll_values)
        pitch_mean = mean_angle_deg(pitch_values)
        yaw_mean = mean_angle_deg(yaw_values)

        print()
        print("marker_pose_stats:")
        print(f"  frame: {self.args.world_frame} -> {self.args.child_frame}")
        print(f"  samples: {len(self.rows)}")
        print(f"  height_z_mean_m: {z_mean:.6f}")
        print(f"  height_z_std_m:  {stddev(z_values):.6f}")
        print(f"  roll_mean_deg:   {roll_mean:.6f}")
        print(f"  pitch_mean_deg:  {pitch_mean:.6f}")
        print(f"  yaw_mean_deg:    {yaw_mean:.6f}")
        print(f"  roll_std_deg:    {stddev(roll_values):.6f}")
        print(f"  pitch_std_deg:   {stddev(pitch_values):.6f}")
        print(f"  yaw_std_deg:     {stddev(yaw_values):.6f}")
        if self.args.out:
            print(f"  csv: {self.args.out}")

        if self.args.ref_height is not None:
            print()
            print("mujoco_reference_correction:")
            print(f"  ref_height_z_m:       {self.args.ref_height:.6f}")
            print(f"  marker_to_base_z_m:   {self.args.ref_height - z_mean:.6f}")
        if self.args.ref_roll_deg is not None:
            print(f"  ref_roll_deg:         {self.args.ref_roll_deg:.6f}")
            print(f"  roll_correction_deg:  {angle_error_deg(self.args.ref_roll_deg, roll_mean):.6f}")
        if self.args.ref_pitch_deg is not None:
            print(f"  ref_pitch_deg:        {self.args.ref_pitch_deg:.6f}")
            print(f"  pitch_correction_deg: {angle_error_deg(self.args.ref_pitch_deg, pitch_mean):.6f}")
        if self.args.ref_yaw_deg is not None:
            print(f"  ref_yaw_deg:          {self.args.ref_yaw_deg:.6f}")
            print(f"  yaw_correction_deg:   {angle_error_deg(self.args.ref_yaw_deg, yaw_mean):.6f}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record average roll/pitch/yaw/height from corrected RViz TF."
    )
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--child-frame", default="go2_body_marker")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--lookup-timeout", type=float, default=0.2)
    parser.add_argument("--out", default="/tmp/go2_body_marker_pose_stats.csv")
    parser.add_argument("--ref-height", type=float)
    parser.add_argument("--ref-roll-deg", type=float)
    parser.add_argument("--ref-pitch-deg", type=float)
    parser.add_argument("--ref-yaw-deg", type=float)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")
    if args.duration <= args.settle:
        raise SystemExit("--duration must be greater than --settle")

    rclpy.init()
    node = TfPoseRecorder(args)
    try:
        node.run()
    except KeyboardInterrupt:
        node.print_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
