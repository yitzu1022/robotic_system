#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


CSV_HEADER = [
    "idx",
    "ts_system",
    "world_frame",
    "base_frame",
    "x",
    "y",
    "z",
    "qx",
    "qy",
    "qz",
    "qw",
    "yaw_rad",
    "basename",
    "original_timestamp",
]


def quat_normalize(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    return qx / norm, qy / norm, qz / norm, qw / norm


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def rotmat_to_quat(R: List[List[float]]) -> Tuple[float, float, float, float]:
    r00, r01, r02 = R[0][0], R[0][1], R[0][2]
    r10, r11, r12 = R[1][0], R[1][1], R[1][2]
    r20, r21, r22 = R[2][0], R[2][1], R[2][2]
    trace = r00 + r11 + r22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s

    return quat_normalize(qx, qy, qz, qw)


def load_trajectory(path: Path) -> List[Dict[str, Any]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict):
        for key in ("trajectory", "poses", "frames", "camera_trajectory"):
            if key in data and isinstance(data[key], list):
                data = data[key]
                break
    if not isinstance(data, list):
        raise ValueError("JSON must be a list, or a dict containing trajectory/poses/frames/camera_trajectory")
    return data


def parse_basename_index(basename: Any, fallback_idx: int) -> int:
    if basename is None:
        return fallback_idx
    text = str(basename)
    try:
        return int(text)
    except ValueError:
        digits = "".join(ch for ch in text if ch.isdigit())
        return int(digits) if digits else fallback_idx


def get_pose_fields(item: Dict[str, Any]) -> Tuple[float, float, float, float, float, float, float]:
    if all(k in item for k in ("tx", "ty", "tz")):
        x = float(item["tx"])
        y = float(item["ty"])
        z = float(item["tz"])
    elif "transform_matrix" in item:
        T = item["transform_matrix"]
        x = float(T[0][3])
        y = float(T[1][3])
        z = float(T[2][3])
    else:
        raise ValueError("pose item missing tx/ty/tz and transform_matrix")

    if all(k in item for k in ("qx", "qy", "qz", "qw")):
        qx, qy, qz, qw = quat_normalize(
            float(item["qx"]),
            float(item["qy"]),
            float(item["qz"]),
            float(item["qw"]),
        )
    elif "transform_matrix" in item:
        T = item["transform_matrix"]
        qx, qy, qz, qw = rotmat_to_quat([row[:3] for row in T[:3]])
    else:
        raise ValueError("pose item missing qx/qy/qz/qw and transform_matrix")

    return x, y, z, qx, qy, qz, qw


def choose_time(item: Dict[str, Any], idx: int, basename_idx: int, time_source: str) -> float:
    if time_source == "idx":
        return float(idx)
    if time_source == "basename":
        return float(basename_idx)
    if time_source == "timestamp":
        if "timestamp" not in item:
            raise ValueError("time_source=timestamp but an item has no timestamp")
        return float(item["timestamp"])
    raise ValueError(f"unsupported time_source: {time_source}")


def convert_rows(
    items: Iterable[Dict[str, Any]],
    time_source: str,
    world_frame: str,
    base_frame: str,
) -> List[Dict[str, Any]]:
    rows = []
    for idx, item in enumerate(items):
        basename = item.get("basename", f"{idx:06d}")
        basename_idx = parse_basename_index(basename, idx)
        x, y, z, qx, qy, qz, qw = get_pose_fields(item)
        ts_system = choose_time(item, idx, basename_idx, time_source)
        original_timestamp: Optional[float]
        original_timestamp = float(item["timestamp"]) if "timestamp" in item else None

        rows.append(
            {
                "idx": idx,
                "ts_system": f"{ts_system:.9f}",
                "world_frame": world_frame,
                "base_frame": base_frame,
                "x": f"{x:.9f}",
                "y": f"{y:.9f}",
                "z": f"{z:.9f}",
                "qx": f"{qx:.12f}",
                "qy": f"{qy:.12f}",
                "qz": f"{qz:.12f}",
                "qw": f"{qw:.12f}",
                "yaw_rad": f"{quat_to_yaw(qx, qy, qz, qw):.12f}",
                "basename": basename,
                "original_timestamp": "" if original_timestamp is None else f"{original_timestamp:.9f}",
            }
        )
    return rows


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a 3D camera trajectory JSON to map_alignment_v3-compatible CSV."
    )
    parser.add_argument(
        "--input",
        "-i",
        default="robot_deploy_slam_example0_camera_trajectory.json",
        help="input 3D trajectory JSON path",
    )
    parser.add_argument(
        "--output",
        "-o",
        default="pose3d.csv",
        help="output CSV path for map_alignment_v3.py --pose3d_csv",
    )
    parser.add_argument(
        "--time-source",
        choices=("idx", "basename", "timestamp"),
        default="idx",
        help="value written to ts_system. Use idx/basename to align with pose2d.csv --csv_t idx; use timestamp for real trajectory time.",
    )
    parser.add_argument("--world-frame", default="map3d")
    parser.add_argument("--base-frame", default="camera")
    args = parser.parse_args()

    items = load_trajectory(Path(args.input))
    rows = convert_rows(
        items,
        time_source=args.time_source,
        world_frame=args.world_frame,
        base_frame=args.base_frame,
    )
    write_csv(Path(args.output), rows)

    print(f"Converted {len(rows)} poses")
    print(f"Saved: {args.output}")
    print("map_alignment_v3.py default can read this CSV with --pose3d_csv", args.output)


if __name__ == "__main__":
    main()
