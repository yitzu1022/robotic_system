#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# -----------------------------
# SE(3) helpers
# -----------------------------
def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n <= 0:
        raise ValueError("Quaternion norm is zero.")
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    R = np.array([
        [1 - 2*(yy+zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx+yy)],
    ], dtype=np.float64)
    return R


def make_T(txyz, qxyzw):
    tx, ty, tz = txyz
    qx, qy, qz, qw = qxyzw
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_R(qx, qy, qz, qw)
    T[:3, 3] = [tx, ty, tz]
    return T


def inv_T(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti

# -----------------------------
# Plane (PCA) projection
# -----------------------------
def fit_ground_plane_basis(P3: np.ndarray):
    P3 = np.asarray(P3, dtype=np.float64)
    if P3.ndim != 2 or P3.shape[1] != 3:
        raise ValueError(f"P3 must be (N,3), got {P3.shape}")
    if P3.shape[0] < 3:
        raise ValueError("Need at least 3 points to fit plane.")

    mu = P3.mean(axis=0)
    Q = P3 - mu
    C = (Q.T @ Q) / P3.shape[0]
    w, V = np.linalg.eigh(C)
    n = V[:, 0]
    n = n / (np.linalg.norm(n) + 1e-12)

    # deterministic e1: project x-axis to plane
    x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    e1 = x_axis - (x_axis @ n) * n
    if np.linalg.norm(e1) < 1e-6:
        y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        e1 = y_axis - (y_axis @ n) * n
    e1 = e1 / (np.linalg.norm(e1) + 1e-12)
    e2 = np.cross(n, e1)
    e2 = e2 / (np.linalg.norm(e2) + 1e-12)

    z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    angle_deg = math.degrees(math.acos(np.clip(abs(n @ z_axis), -1.0, 1.0)))
    return mu, n, e1, e2, angle_deg


def project_to_plane_uv(p3, mu, e1, e2):
    d = p3 - mu
    return np.array([d @ e1, d @ e2], dtype=np.float64)


# -----------------------------
# Parsing
# -----------------------------
def load_extrinsic_yaml_as_T(path: str) -> np.ndarray:
    """
    Expect:
      translation: {x,y,z}
      rotation: {x,y,z,w}
    Return a 4x4 transform from YAML directly (no semantics here).
    """
    data = yaml.safe_load(Path(path).read_text())
    t = data["translation"]
    r = data["rotation"]
    return make_T((t["x"], t["y"], t["z"]), (r["x"], r["y"], r["z"], r["w"]))


def load_cam_pose(path: str) -> pd.DataFrame:
    p = Path(path)
    lines = p.read_text().strip().splitlines()
    first = lines[0]
    has_alpha = any(c.isalpha() for c in first)

    if has_alpha:
        df = pd.read_csv(path, sep=r"[,\s\t]+", engine="python")
    else:
        arr = []
        for line in lines:
            if not line.strip() or line.strip().startswith("#"):
                continue
            toks = line.replace(",", " ").split()
            if len(toks) < 8:
                raise ValueError(f"cam pose line needs 8 fields: idx tx ty tz qx qy qz qw, got: {line}")
            arr.append([float(x) for x in toks[:8]])
        df = pd.DataFrame(arr, columns=["idx","tx","ty","tz","qx","qy","qz","qw"])

    df = df.rename(columns={c: c.strip().lower() for c in df.columns})
    if "index" in df.columns and "idx" not in df.columns:
        df = df.rename(columns={"index": "idx"})
    if "ts_system" in df.columns and "idx" not in df.columns:
        df = df.rename(columns={"ts_system": "idx"})

    # pose3d.csv uses x/y/z; older camera trajectory files use tx/ty/tz.
    rename_xyz = {}
    for src, dst in (("x", "tx"), ("y", "ty"), ("z", "tz")):
        if src in df.columns and dst not in df.columns:
            rename_xyz[src] = dst
    if rename_xyz:
        df = df.rename(columns=rename_xyz)

    needed = ["idx","tx","ty","tz","qx","qy","qz","qw"]
    missing = [c for c in needed if c not in df.columns]
    if missing:
        raise ValueError(f"cam pose missing columns: {missing}. columns={list(df.columns)}")

    df["idx"] = df["idx"].astype(int)
    return df[needed].copy()


def load_base2d_pose_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df = df.rename(columns={c: c.strip().lower() for c in df.columns})
    if "index" in df.columns and "idx" not in df.columns:
        df = df.rename(columns={"index": "idx"})
    if "yaw_rad" in df.columns and "yaw" not in df.columns:
        df = df.rename(columns={"yaw_rad": "yaw"})

    needed = ["idx","x","y","yaw"]
    missing = [c for c in needed if c not in df.columns]
    if missing:
        raise ValueError(
            f"base2d pose missing columns: {missing}. Need idx,x,y and yaw or yaw_rad. columns={list(df.columns)}"
        )
    df["idx"] = df["idx"].astype(int)
    return df[needed].copy()


# -----------------------------
# Sim(2)
# -----------------------------
def estimate_sim2(Psrc, Pdst, with_scale=True):
    """
    Pdst ~= s * R * Psrc + t
    cov = X^T Y
    R = V U^T
    """
    Psrc = np.asarray(Psrc, dtype=np.float64)
    Pdst = np.asarray(Pdst, dtype=np.float64)

    n = Psrc.shape[0]
    if n < 2:
        raise ValueError(f"Not enough points: n={n}")

    mask = np.isfinite(Psrc).all(axis=1) & np.isfinite(Pdst).all(axis=1)
    Psrc = Psrc[mask]
    Pdst = Pdst[mask]
    n = Psrc.shape[0]
    if n < 2:
        raise ValueError(f"Not enough finite points: n={n}")

    mu_s = Psrc.mean(axis=0)
    mu_d = Pdst.mean(axis=0)
    X = Psrc - mu_s
    Y = Pdst - mu_d

    cov = (X.T @ Y) / n
    U, S, Vt = np.linalg.svd(cov)

    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    if with_scale:
        varX = (X * X).sum() / n
        if varX <= 1e-12:
            raise ValueError("Degenerate configuration: varX too small")
        s = float(S.sum() / varX)
    else:
        s = 1.0

    t = mu_d - s * (R @ mu_s)
    return s, R, t


def apply_sim2(P, s, R, t):
    P = np.asarray(P, dtype=np.float64)
    return (s * (R @ P.T)).T + t


def rmse(Psrc, Pdst, s, R, t):
    pred = apply_sim2(Psrc, s, R, t)
    e = pred - Pdst
    return float(math.sqrt((e * e).sum() / Psrc.shape[0]))


def robust_estimate_sim2(Psrc, Pdst, with_scale=True, trim_keep=0.8, iters=3, min_inliers=10):
    Psrc = np.asarray(Psrc, dtype=np.float64)
    Pdst = np.asarray(Pdst, dtype=np.float64)
    if Psrc.shape[0] < min_inliers:
        raise ValueError("Too few points for robust.")

    mask = np.ones(Psrc.shape[0], dtype=bool)
    s, R, t = 1.0, np.eye(2), np.zeros(2)

    for _ in range(iters):
        if mask.sum() < min_inliers:
            break
        s, R, t = estimate_sim2(Psrc[mask], Pdst[mask], with_scale=with_scale)
        pred = apply_sim2(Psrc, s, R, t)
        r = np.linalg.norm(pred - Pdst, axis=1)
        thr = np.quantile(r, trim_keep)
        new_mask = r <= thr
        if new_mask.sum() == mask.sum():
            mask = new_mask
            break
        mask = new_mask

    if mask.sum() < min_inliers:
        raise ValueError("Too few inliers after trimming.")

    err = rmse(Psrc[mask], Pdst[mask], s, R, t)
    return err, s, R, t, mask


# -----------------------------
# Main
# -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam_pose", default="./input/pose3d.csv", help="camera trajectory CSV, e.g. pose3d.csv")
    ap.add_argument("--base2d_pose", default="./input/pose2d.csv")
    ap.add_argument("--cam_to_base", default="./input/cam_to_base.yaml")

    ap.add_argument("--extrinsic_mode", choices=["cam_to_base", "base_to_cam", "auto"], default="cam_to_base",
                    help="cam_to_base: YAML is (camera->base). base_to_cam: YAML is (base->camera). auto: try both.")
    ap.add_argument("--index_offset", type=int, default=0)
    ap.add_argument("--auto_offset", action="store_true")
    ap.add_argument("--offset_range", type=int, default=5)
    ap.add_argument("--min_matched", type=int, default=10)

    ap.add_argument("--yaw_deg", action="store_true")
    ap.add_argument("--with_scale", action="store_true")

    ap.add_argument("--robust", action="store_true")
    ap.add_argument("--trim_keep", type=float, default=0.8)
    ap.add_argument("--robust_iters", type=int, default=3)

    ap.add_argument("--out", default="./output/map3d_to_map2d_sim2.yaml")
    ap.add_argument("--plot_out", default="./output/traj_alignment.png")
    args = ap.parse_args()

    cam = load_cam_pose(args.cam_pose)
    base2d = load_base2d_pose_csv(args.base2d_pose)
    if args.yaw_deg:
        base2d["yaw"] = np.deg2rad(base2d["yaw"].astype(float))

    # YAML transform (raw)
    T_yaml = load_extrinsic_yaml_as_T(args.cam_to_base)

    # Build candidates of T_cam_base (cam<-base) that will be used as:
    # T_map3d_base = T_map3d_cam @ T_cam_base
    candidates = []
    if args.extrinsic_mode in ("cam_to_base", "auto"):
        # YAML is (cam->base) == (base<-cam), so invert to get (cam<-base)
        T_base_cam = T_yaml
        T_cam_base = inv_T(T_base_cam)
        candidates.append(("cam_to_base(inv)", T_cam_base))
    if args.extrinsic_mode in ("base_to_cam", "auto"):
        # YAML is (base->cam) == (cam<-base), use directly
        T_cam_base = T_yaml
        candidates.append(("base_to_cam(direct)", T_cam_base))

    def cam_row_to_T(r):
        return make_T((r.tx, r.ty, r.tz), (r.qx, r.qy, r.qz, r.qw))

    def solve_for_offset(off: int, T_cam_base: np.ndarray, tag: str):
        cam_tmp = cam.copy()
        cam_tmp["idx_shift"] = cam_tmp["idx"] + int(off)
        merged = cam_tmp.merge(base2d, left_on="idx_shift", right_on="idx",
                            how="inner", suffixes=("_cam","_base"))
        if len(merged) < int(args.min_matched):
            return None

        # 1) matched 3D base points (map3d) and matched 2D points (map2d)
        P3_3d = []
        P2_xy = []
        for _, r in merged.iterrows():
            T_m3_c = cam_row_to_T(r)
            T_m3_b = T_m3_c @ T_cam_base
            p = T_m3_b[:3, 3]
            P3_3d.append(p)
            P2_xy.append([r["x"], r["y"]])

        P3_3d = np.asarray(P3_3d, dtype=np.float64)
        P2_xy = np.asarray(P2_xy, dtype=np.float64)

        # 2) fit plane from matched points ONLY
        mu_plane, n_plane, e1, e2, angle_deg = fit_ground_plane_basis(P3_3d)

        # 3) try basis sign variants to avoid mirrored uv
        basis_variants = [
            ("(+,+)",  e1,  e2),
            ("(+,-)",  e1, -e2),
            ("(-,+)", -e1,  e2),
            ("(-,-)", -e1, -e2),
        ]

        best_local = None
        for btag, e1v, e2v in basis_variants:
            P3_uv = np.vstack([project_to_plane_uv(p, mu_plane, e1v, e2v) for p in P3_3d])

            try:
                if args.robust:
                    err, s, R, t, in_mask = robust_estimate_sim2(
                        P3_uv, P2_xy,
                        with_scale=args.with_scale,
                        trim_keep=float(args.trim_keep),
                        iters=int(args.robust_iters),
                        min_inliers=int(args.min_matched)
                    )
                    n_in = int(in_mask.sum())
                else:
                    s, R, t = estimate_sim2(P3_uv, P2_xy, with_scale=args.with_scale)
                    err = rmse(P3_uv, P2_xy, s, R, t)
                    in_mask = np.ones(P3_uv.shape[0], dtype=bool)
                    n_in = int(P3_uv.shape[0])
            except Exception:
                continue

            cand = {
                "err": float(err),
                "off": int(off),
                "tag": f"{tag}|basis{btag}",
                "T_cam_base": T_cam_base,
                "s": float(s),
                "R": R,
                "t": t,
                "theta": float(math.atan2(R[1, 0], R[0, 0])),
                "n_all": int(P3_uv.shape[0]),
                "n_in": int(n_in),
                "in_mask": in_mask,
                "merged": merged,
                "mu_plane": mu_plane,
                "n_plane": n_plane,
                "e1": e1v,
                "e2": e2v,
                "angle_deg": float(angle_deg),
            }

            if best_local is None or cand["err"] < best_local["err"]:
                best_local = cand

        return best_local


    # search best
    best = None
    off_list = [int(args.index_offset)]
    if args.auto_offset:
        off_list = list(range(-int(args.offset_range), int(args.offset_range) + 1))

    for off in off_list:
        for tag, T_cam_base in candidates:
            res = solve_for_offset(off, T_cam_base, tag)
            if res is None:
                continue
            if best is None or res["err"] < best["err"]:
                best = res

    if best is None:
        raise RuntimeError("No valid solution: check idx/offset/min_matched/extrinsic.")

    # report
    print(f"[best] extrinsic={best['tag']}, offset={best['off']}, matched={best['n_all']}, inliers={best['n_in']}, rmse={best['err']:.4f}")
    print(f"[plane] angle(normal, map3d Z) = {best['angle_deg']:.3f} deg, normal={best['n_plane']}")
    print(f"  scale s = {best['s']:.8f}")
    print(f"  theta (deg) = {math.degrees(best['theta']):.6f}")
    print(f"  t = [{best['t'][0]:.6f}, {best['t'][1]:.6f}]")

    # plot
    base_xy = base2d.sort_values("idx")[["x", "y"]].to_numpy(np.float64)

    cam_tmp = cam.copy()
    cam_tmp["idx_shift"] = cam_tmp["idx"] + int(best["off"])
    cam_tmp = cam_tmp.sort_values("idx_shift")

    mu_plane, e1, e2 = best["mu_plane"], best["e1"], best["e2"]
    T_cam_base = best["T_cam_base"]

    cam_uv_all = []
    for _, r in cam_tmp.iterrows():
        T_m3_c = cam_row_to_T(r)
        T_m3_b = T_m3_c @ T_cam_base
        p = T_m3_b[:3, 3]
        cam_uv_all.append(project_to_plane_uv(p, mu_plane, e1, e2))
    cam_uv_all = np.asarray(cam_uv_all, dtype=np.float64)
    cam_xy_m2_all = apply_sim2(cam_uv_all, best["s"], best["R"], best["t"])

    merged = best["merged"]
    P2_mat = merged[["x", "y"]].to_numpy(np.float64)

    P3_uv_mat = []
    for _, r in merged.iterrows():
        T_m3_c = cam_row_to_T(r)
        T_m3_b = T_m3_c @ T_cam_base
        p = T_m3_b[:3, 3]
        P3_uv_mat.append(project_to_plane_uv(p, mu_plane, e1, e2))
    P3_uv_mat = np.asarray(P3_uv_mat, dtype=np.float64)
    P3_xy_mat = apply_sim2(P3_uv_mat, best["s"], best["R"], best["t"])

    in_mask = best["in_mask"]
    in_base = P2_mat[in_mask]
    in_tran = P3_xy_mat[in_mask]

    plt.figure(figsize=(8, 8))
    plt.plot(base_xy[:, 0], base_xy[:, 1], linewidth=2, label="2D base trajectory (map2d)")
    plt.plot(cam_xy_m2_all[:, 0], cam_xy_m2_all[:, 1], linewidth=2, label="3D->plane->2D transformed trajectory")
    plt.scatter(P2_mat[:, 0], P2_mat[:, 1], s=10, alpha=0.35, label="matched base2d pts")
    plt.scatter(P3_xy_mat[:, 0], P3_xy_mat[:, 1], s=10, alpha=0.35, label="matched transformed pts")
    plt.scatter(in_base[:, 0], in_base[:, 1], s=18, alpha=0.9, label="inlier base2d pts")
    plt.scatter(in_tran[:, 0], in_tran[:, 1], s=18, alpha=0.9, label="inlier transformed pts")
    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title(
        f"Alignment: extrinsic={best['tag']}, offset={best['off']}, RMSE={best['err']:.4f}, s={best['s']:.6f}, "
        f"theta={math.degrees(best['theta']):.2f} deg\n"
        f"plane angle(normal,Z)={best['angle_deg']:.2f} deg, robust={args.robust}"
    )
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.plot_out, dpi=200)
    print(f"[plot] {args.plot_out}")

    # write yaml
    out = {
        "extrinsic_mode_selected": best["tag"],
        "index_offset_cam_to_base2d": int(best["off"]),
        "with_scale": bool(args.with_scale),
        "robust": bool(args.robust),
        "trim_keep": float(args.trim_keep),
        "robust_iters": int(args.robust_iters),
        "rmse_xy": float(best["err"]),
        "plane_fit": {
            "angle_normal_to_map3d_Z_deg": float(best["angle_deg"]),
            "mu": {"x": float(mu_plane[0]), "y": float(mu_plane[1]), "z": float(mu_plane[2])},
            "normal_n": [float(best["n_plane"][0]), float(best["n_plane"][1]), float(best["n_plane"][2])],
            "basis_e1": [float(e1[0]), float(e1[1]), float(e1[2])],
            "basis_e2": [float(e2[0]), float(e2[1]), float(e2[2])],
            "uv_definition": "u=dot(p-mu,e1), v=dot(p-mu,e2)"
        },
        "sim2": {
            "s": float(best["s"]),
            "R": [[float(best["R"][0, 0]), float(best["R"][0, 1])],
                  [float(best["R"][1, 0]), float(best["R"][1, 1])]],
            "t": {"x": float(best["t"][0]), "y": float(best["t"][1])},
            "theta_rad": float(best["theta"]),
            "theta_deg": float(math.degrees(best["theta"])),
        },
        "counts": {"matched": int(best["n_all"]), "inliers": int(best["n_in"])},
    }
    Path(args.out).write_text(yaml.safe_dump(out, sort_keys=False, allow_unicode=True))
    print(f"[write] {args.out}")


if __name__ == "__main__":
    main()
