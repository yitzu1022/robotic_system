#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np
# -------------------------
# Data containers
# -------------------------
@dataclass
class Pose2DSeq:
    t: np.ndarray        # (M,)
    p: np.ndarray        # (M,2)

# -------------------------
# IO
# -------------------------

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

# -------------------------
# Main
# -------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose2d_csv", default="./poses.csv")
    # CSV columns
    ap.add_argument("--csv_t", default="t_sec")
    ap.add_argument("--csv_x", default="x")
    ap.add_argument("--csv_y", default="y")
    ap.add_argument("--csv_delim", default=",")

    args = ap.parse_args()

    out_prefix = Path("./output/trajectory_2d")
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    pose2d = load_pose2d_csv(Path(args.pose2d_csv),
                             t_col=args.csv_t, x_col=args.csv_x, y_col=args.csv_y,
                             delimiter=args.csv_delim)

    # ---- Plots ----
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(pose2d.p[:, 0], pose2d.p[:, 1], linewidth=1, label="2D (csv) base traj")
    plt.axis("equal"); plt.grid(True); plt.legend()
    plt.title("Trajectory Alignment")
    plt.tight_layout()
    plt.savefig(out_prefix.with_suffix(".align.png"), dpi=200)

if __name__ == "__main__":
    main()