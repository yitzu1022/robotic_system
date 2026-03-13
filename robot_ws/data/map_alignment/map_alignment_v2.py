#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
# -------------------------
# Basic math
# -------------------------

def quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    """Quaternion (x,y,z,w) -> 3x3"""
    x, y, z, w = qx, qy, qz, qw
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array([
        [1 - 2*(yy+zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx+yy)],
    ], dtype=float)
    return R

def rot_to_yaw(R: np.ndarray) -> float:
    """Extract yaw from rotation matrix (assuming z-up)"""
    return math.atan2(R[1,0], R[0,0])

def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T

def inv_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=float)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti

def umeyama_2d(P: np.ndarray, Q: np.ndarray, with_scale: bool = True) -> Tuple[float, np.ndarray, np.ndarray]:
    """Q ~= s*R*P + t ; P,Q: (N,2) row vectors"""
    muP = P.mean(axis=0)
    muQ = Q.mean(axis=0)
    X = P - muP
    Y = Q - muQ

    C = (Y.T @ X) / P.shape[0]
    U, S, Vt = np.linalg.svd(C)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt

    if with_scale:
        varP = (X**2).sum() / P.shape[0]
        s = float(S.sum() / max(varP, 1e-12))
    else:
        s = 1.0

    t = muQ - s * (R @ muP)
    return s, R, t

def nearest_time_match(t_src: np.ndarray, t_tgt: np.ndarray, max_dt: float) -> np.ndarray:
    idx = np.searchsorted(t_tgt, t_src)
    idx0 = np.clip(idx - 1, 0, len(t_tgt) - 1)
    idx1 = np.clip(idx,     0, len(t_tgt) - 1)

    dt0 = np.abs(t_tgt[idx0] - t_src)
    dt1 = np.abs(t_tgt[idx1] - t_src)
    pick = np.where(dt1 < dt0, idx1, idx0)

    dt = np.abs(t_tgt[pick] - t_src)
    return np.where(dt <= max_dt, pick, -1)


# -------------------------
# Data containers
# -------------------------

@dataclass
class Pose3DSeq:
    t: np.ndarray        # (N,)
    T_G_cam: np.ndarray  # (N,4,4)

@dataclass
class Pose2DSeq:
    t: np.ndarray        # (M,)
    p: np.ndarray        # (M,2)


# -------------------------
# IO
# -------------------------

def load_pose3d_txt_tum(path: Path) -> Pose3DSeq:
    """
    TUM: t tx ty tz qx qy qz qw
    """
    ts = []
    Ts = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.replace(",", " ").split()
        if len(parts) < 8:
            continue
        t = float(parts[0])/1000000000.0
        tx, ty, tz = map(float, parts[1:4])
        qx, qy, qz, qw = map(float, parts[4:8])
        R = quat_to_rot(qx, qy, qz, qw)
        T = make_T(R, np.array([tx, ty, tz], dtype=float))
        ts.append(t)
        Ts.append(T)
    t = np.asarray(ts, dtype=float)
    T = np.asarray(Ts, dtype=float)
    order = np.argsort(t)
    return Pose3DSeq(t=t[order], T_G_cam=T[order])

def load_pose2d_csv(path: Path, t_col="t_sec", x_col="x", y_col="y", delimiter=",") -> Pose2DSeq:
    import pandas as pd
    df = pd.read_csv(path, sep=delimiter)
    for c in [t_col, x_col, y_col]:
        if c not in df.columns:
            raise ValueError(f"CSV 缺少欄位 {c}，目前欄位：{list(df.columns)}")
    t = df[t_col].to_numpy(dtype=float)
    p = np.stack([df[x_col].to_numpy(dtype=float),
                  df[y_col].to_numpy(dtype=float)], axis=1)
    order = np.argsort(t)
    return Pose2DSeq(t=t[order], p=p[order])

def load_T_from_yaml_flexible(yaml_path: Path) -> np.ndarray:
    """
    支援幾種常見 YAML：
    1) {translation: {x,y,z}, rotation: {x,y,z,w}}
    2) {t: [x,y,z], q: [x,y,z,w]}
    3) {R: [[...],[...],[...]], t: [x,y,z]}
    4) {T: [[4x4]]}
    """
    import yaml
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if data is None:
        raise ValueError(f"{yaml_path} 是空的")

    # Case: T 4x4
    if isinstance(data, dict) and "T" in data:
        T = np.array(data["T"], dtype=float)
        if T.shape == (4,4):
            return T

    # Case: R + t
    if isinstance(data, dict) and "R" in data and "t" in data:
        R = np.array(data["R"], dtype=float)
        t = np.array(data["t"], dtype=float).reshape(3)
        return make_T(R, t)

    # Case: translation/rotation dict
    if isinstance(data, dict) and "translation" in data and "rotation" in data:
        tr = data["translation"]
        rr = data["rotation"]
        t = np.array([tr["x"], tr["y"], tr["z"]], dtype=float)
        q = np.array([rr["x"], rr["y"], rr["z"], rr["w"]], dtype=float)
        R = quat_to_rot(*q)
        return make_T(R, t)

    # Case: t + q arrays
    if isinstance(data, dict) and "t" in data and "q" in data:
        t = np.array(data["t"], dtype=float).reshape(3)
        q = np.array(data["q"], dtype=float).reshape(4)
        R = quat_to_rot(*q)
        return make_T(R, t)

    raise ValueError(
        f"不支援的 YAML 格式：{yaml_path}\n"
        f"可接受鍵：T / (R,t) / (translation,rotation) / (t,q)"
    )


# -------------------------
# Main
# -------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose3d_txt", default="./pose_3d.txt")
    ap.add_argument("--pose2d_csv", default="./pose_2d.csv")
    ap.add_argument("--out_prefix", default="alignment_out/out")
    ap.add_argument("--max_dt", type=float, default=0.05)
    ap.add_argument("--time_offset_2d", type=float, default=0.0)
    ap.add_argument("--use_scale", type=int, default=0)
    ap.add_argument("--trim_quantile", type=float, default=0.90)
    ap.add_argument("--min_pairs", type=int, default=30)

    # CSV columns
    ap.add_argument("--csv_t", default="t_sec")
    ap.add_argument("--csv_x", default="x")
    ap.add_argument("--csv_y", default="y")
    ap.add_argument("--csv_delim", default=",")

    # Extrinsics
    ap.add_argument("--T_cam_base_yaml", default="./cam_to_base.yaml", help="YAML for cam->base (preferred)")
    ap.add_argument("--T_base_lidar_yaml", default="", help="YAML for base->lidar")
    ap.add_argument("--T_lidar_cam_yaml", default="", help="YAML for lidar->cam (calibration)")
    args = ap.parse_args()

    out_prefix = Path(args.out_prefix)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    pose3d = load_pose3d_txt_tum(Path(args.pose3d_txt))
    pose2d = load_pose2d_csv(Path(args.pose2d_csv),
                             t_col=args.csv_t, x_col=args.csv_x, y_col=args.csv_y,
                             delimiter=args.csv_delim)

    # ---- Build T_cam_base ----
    if args.T_cam_base_yaml.strip():
        T_cam_base = load_T_from_yaml_flexible(Path(args.T_cam_base_yaml))
    else:
        if not (args.T_base_lidar_yaml.strip() and args.T_lidar_cam_yaml.strip()):
            raise RuntimeError(
                "你目前 3D txt 是 camera pose，所以一定要提供外參。\n"
                "請提供：--T_cam_base_yaml\n"
                "或同時提供：--T_base_lidar_yaml + --T_lidar_cam_yaml"
            )
        T_base_lidar = load_T_from_yaml_flexible(Path(args.T_base_lidar_yaml))
        T_lidar_cam  = load_T_from_yaml_flexible(Path(args.T_lidar_cam_yaml))
        T_base_cam = T_base_lidar @ T_lidar_cam
        T_cam_base = T_base_cam

    # ---- Convert camera poses to base poses in 3D map frame ----
    # T_G_base(t) = T_G_cam(t) @ T_cam_base
    T_G_base = pose3d.T_G_cam @ T_cam_base  # broadcast (N,4,4) @ (4,4)

    # project to 2D
    P_all = T_G_base[:, 0:2, 3]  # (N,2) take x,y of base in 3D map frame

    # ---- Time match ----
    t2 = pose2d.t + float(args.time_offset_2d)
    match_idx = nearest_time_match(pose3d.t, t2, max_dt=float(args.max_dt))
    valid = match_idx >= 0
    if valid.sum() < args.min_pairs:
        raise RuntimeError(f"可用配對點太少：{valid.sum()} < {args.min_pairs}。請調大 --max_dt 或檢查時間戳單位")

    P = P_all[valid]                       # (K,2)
    Q = pose2d.p[match_idx[valid], :]      # (K,2)

    # ---- Fit Sim(2)/SE(2) ----
    s, R, t = umeyama_2d(P, Q, with_scale=bool(args.use_scale))
    P_hat = (s * (P @ R.T)) + t
    res = np.linalg.norm(P_hat - Q, axis=1)

    thr = np.quantile(res, float(args.trim_quantile))
    keep = res <= thr
    s2, R2, t2v = umeyama_2d(P[keep], Q[keep], with_scale=bool(args.use_scale))

    # ---- Save ----
    result = {
        "use_scale": bool(args.use_scale),
        "scale": float(s2),
        "R_2x2": R2.tolist(),
        "t_2": t2v.tolist(),
        "matched_pairs_total": int(len(P)),
        "matched_pairs_used": int(int(keep.sum())),
        "max_dt": float(args.max_dt),
        "time_offset_2d": float(args.time_offset_2d),
    }
    out_json = out_prefix.with_suffix(".sim2.json")
    out_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    # ---- Plots ----
    import matplotlib.pyplot as plt
    P_all_hat = (s2 * (P_all @ R2.T)) + t2v

    plt.figure()
    plt.plot(pose2d.p[:, 0], pose2d.p[:, 1], linewidth=1, label="2D (csv) base traj")
    plt.plot(P_all_hat[:, 0], P_all_hat[:, 1], linewidth=1, label="3D cam->base proj + aligned")
    plt.axis("equal"); plt.grid(True); plt.legend()
    plt.title("Trajectory Alignment")
    plt.tight_layout()
    plt.savefig(out_prefix.with_suffix(".align.png"), dpi=200)

    # residuals on kept points
    P2_hat = (s2 * (P[keep] @ R2.T)) + t2v
    res2 = np.linalg.norm(P2_hat - Q[keep], axis=1)
    plt.figure()
    plt.hist(res2, bins=50)
    plt.grid(True)
    plt.title("Residuals (after trimming)")
    plt.xlabel("meters"); plt.ylabel("count")
    plt.tight_layout()
    plt.savefig(out_prefix.with_suffix(".residuals.png"), dpi=200)

    print("=== Done ===")
    print(json.dumps(result, indent=2))
    print(f"Saved: {out_json}")
    print(f"Saved: {out_prefix.with_suffix('.align.png')}")
    print(f"Saved: {out_prefix.with_suffix('.residuals.png')}")
    if abs(s2 - 1.0) > 1e-3:
        print("NOTE: scale != 1，TF 無法表達 scale；請在資料流程中用 Sim(2) 套用。")


if __name__ == "__main__":
    main()