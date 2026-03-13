#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
from typing import Optional, Tuple, Set

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

from tf2_ros import Buffer, TransformException
from tf2_msgs.msg import TFMessage


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Quaternion -> yaw (Z axis)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_float_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="rosbag2 directory path (contains metadata.yaml)")
    ap.add_argument("--out", default="poses.csv", help="output csv path")
    ap.add_argument("--target", default="map", help="target frame (e.g., map or odom)")
    ap.add_argument("--source", default="base_footprint", help="source frame (e.g., base_footprint or base_link)")
    ap.add_argument("--min-dt", type=float, default=0.0, help="minimum time interval (sec) between saved poses")
    ap.add_argument("--use-tf-static", action="store_true", help="also read /tf_static")
    ap.add_argument("--list-frames", action="store_true", help="only list observed parent->child frame pairs and exit")
    args = ap.parse_args()

    rclpy.init(args=None)

    # TF buffer (stores transforms and allows lookup with interpolation)
    buffer = Buffer()

    reader = SequentialReader()
    storage_options = StorageOptions(uri=args.bag, storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr",
                                         output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    tf_msg_type = get_message("tf2_msgs/msg/TFMessage")

    frame_edges: Set[Tuple[str, str]] = set()

    last_saved_t: Optional[float] = None
    rows = []

    # Read all messages
    while reader.has_next():
        topic, data, _t = reader.read_next()  # _t is bag record time (ns), TF uses header.stamp

        if topic not in ("/tf", "/tf_static"):
            continue
        if (topic == "/tf_static") and (not args.use_tf_static):
            continue

        msg = deserialize_message(data, tf_msg_type)  # TFMessage

        assert isinstance(msg, TFMessage)
        for tr in msg.transforms:
            parent = tr.header.frame_id.strip()
            child = tr.child_frame_id.strip()
            if parent and child:
                frame_edges.add((parent, child))

            # Feed transform into buffer
            try:
                if topic == "/tf_static":
                    buffer.set_transform_static(tr, "bag")
                else:
                    buffer.set_transform(tr, "bag")
            except Exception:
                # Some bags can contain malformed transforms; skip safely
                continue

            if args.list_frames:
                continue

            # Query pose at the transform timestamp
            ts = tr.header.stamp
            t_sec = stamp_to_float_sec(ts)

            if last_saved_t is not None and (t_sec - last_saved_t) < args.min_dt:
                continue

            try:
                tf = buffer.lookup_transform(args.target, args.source, ts)
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                q = tf.transform.rotation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

                rows.append((t_sec, x, y, yaw))
                last_saved_t = t_sec

            except TransformException:
                # Not enough transforms yet to resolve the chain at this time
                continue

    if args.list_frames:
        # Print all observed edges
        for p, c in sorted(frame_edges):
            print(f"{p} -> {c}")
        rclpy.shutdown()
        return

    # Write CSV
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_sec", "x", "y", "yaw_rad"])
        w.writerows(rows)

    print(f"[OK] wrote {len(rows)} poses to: {args.out}")
    if len(rows) == 0:
        print("[WARN] No poses were written. Common causes:")
        print(" - target/source frame name mismatch")
        print(" - missing /tf_static (try --use-tf-static)")
        print(" - bag has no chain from target to source")

    rclpy.shutdown()


if __name__ == "__main__":
    main()