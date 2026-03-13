#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""collect_data_rs_kachaka_api.py

Jetson-friendly data collection using **Python APIs only**:
- Intel RealSense D435 (pyrealsense2): capture RGB + Depth + IR1/IR2
- Depth is **aligned to color** (same FOV & pixel grid) for RGB-D SLAM.
- Kachaka pose (kachaka_api): after RealSense frames are successfully acquired
  and saved, record pose to CSV.

Controls:
- Press 'c' to capture
- Press 'q' to quit

Outputs (default: ./dataset_out):
- rgb/000000.png            (BGR PNG)
- depth_aligned_mm/000000.png (uint16 PNG, mm, aligned to RGB)
- ir_left/000000.png        (uint8 PNG)
- ir_right/000000.png       (uint8 PNG)
- poses.csv                 (idx,t_sec,x,y,yaw_rad,rgb_file,depth_file,ir_left_file,ir_right_file)
- meta/000000.json          (optional capture metadata)

Notes:
- "Can't resolve requests" usually means your stream combination is not supported
  (resolution/fps/format/USB bandwidth). This script resolves config with fallbacks.
"""

import os
import json
import time
import math
import argparse
import threading
from pathlib import Path
from typing import Optional, Dict, Any, Tuple

import numpy as np
import cv2

try:
    import pyrealsense2 as rs
except Exception as e:
    rs = None

try:
    import asyncio
    import kachaka_api
except Exception:
    asyncio = None
    kachaka_api = None


# -------------------------
# Kachaka pose client (aio in background loop)
# -------------------------
class KachakaPoseClient:
    """Synchronous pose getter backed by kachaka_api.aio."""

    def __init__(self, endpoint: str):
        if kachaka_api is None or asyncio is None:
            raise RuntimeError("kachaka_api not available. Install: pip3 install kachaka-api")

        self.endpoint = endpoint
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._client = None
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        if not self._ready.wait(timeout=4.0):
            raise RuntimeError("KachakaPoseClient init timeout")

    def _run_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop

        async def init_client():
            try:
                self._client = kachaka_api.aio.KachakaApiClient(self.endpoint)
            except TypeError:
                # older versions may not take endpoint
                self._client = kachaka_api.aio.KachakaApiClient()

        loop.run_until_complete(init_client())
        self._ready.set()

        async def keep_alive():
            while not self._stop.is_set():
                await asyncio.sleep(0.05)

        try:
            loop.run_until_complete(keep_alive())
        finally:
            try:
                loop.stop()
            except Exception:
                pass
            try:
                loop.close()
            except Exception:
                pass

    def close(self):
        self._stop.set()
        try:
            self._thread.join(timeout=2.0)
        except Exception:
            pass

    def get_pose_sync(self, timeout_s: float = 1.5) -> Tuple[float, float, float]:
        """Return (x,y,yaw_rad)."""
        if self._loop is None or self._client is None:
            raise RuntimeError("KachakaPoseClient not ready")

        async def _get():
            return await self._client.get_robot_pose()

        fut = asyncio.run_coroutine_threadsafe(_get(), self._loop)
        pose = fut.result(timeout=timeout_s)
        return float(pose.x), float(pose.y), float(pose.theta)


# -------------------------
# RealSense
# -------------------------
class RealSenseCollector:
    def __init__(
        self,
        serial: Optional[str],
        enable_ir_left: bool = True,
        enable_ir_right: bool = True,
    ):
        if rs is None:
            raise RuntimeError("pyrealsense2 not available. Install librealsense + python bindings")

        self.serial = serial
        self.enable_ir_left = enable_ir_left
        self.enable_ir_right = enable_ir_right

        self.pipeline = rs.pipeline()
        self.profile = None
        self.align_to_color = rs.align(rs.stream.color)

        self.depth_scale: float = 0.001
        self.color_intr = None

        self.active_cfg = None

    @staticmethod
    def _try_resolve_and_start(cfg: "rs.config", pipeline: "rs.pipeline"):
        wrapper = rs.pipeline_wrapper(pipeline)
        # resolve() raises if cannot satisfy
        cfg.resolve(wrapper)
        return pipeline.start(cfg)

    def start(
        self,
        color_whfps=(640, 480, 15),
        depth_whfps=(640, 480, 15),
        ir_whfps=(640, 480, 15),
        warmup_frames: int = 15,
    ):
        cw, ch, cfps = color_whfps
        dw, dh, dfps = depth_whfps
        iw, ih, ifps = ir_whfps

        # Candidate profiles to avoid "Can't resolve requests"
        # Try same-size first (usually best for alignment + bandwidth)
        candidates = [
            (640, 480, 15, 640, 480, 15, 640, 480, 15),
            (640, 480, 30, 640, 480, 30, 640, 480, 30),
            (848, 480, 15, 848, 480, 15, 848, 480, 15),
            (848, 480, 30, 848, 480, 30, 848, 480, 30),
            (cw, ch, cfps, dw, dh, dfps, iw, ih, ifps),
            # bandwidth-saving fallbacks
            (640, 480, 15, 424, 240, 15, 424, 240, 15),
            (640, 480, 15, 424, 240, 15, 640, 480, 15),
        ]

        last_err = None
        for (cw2, ch2, cfps2, dw2, dh2, dfps2, iw2, ih2, ifps2) in candidates:
            try:
                self.pipeline = rs.pipeline()
                cfg = rs.config()
                if self.serial:
                    cfg.enable_device(self.serial)

                cfg.enable_stream(rs.stream.color, cw2, ch2, rs.format.bgr8, cfps2)
                cfg.enable_stream(rs.stream.depth, dw2, dh2, rs.format.z16, dfps2)

                if self.enable_ir_left:
                    cfg.enable_stream(rs.stream.infrared, 1, iw2, ih2, rs.format.y8, ifps2)
                if self.enable_ir_right:
                    cfg.enable_stream(rs.stream.infrared, 2, iw2, ih2, rs.format.y8, ifps2)

                profile = self._try_resolve_and_start(cfg, self.pipeline)
                self.profile = profile
                self.active_cfg = {
                    "color": (cw2, ch2, cfps2),
                    "depth": (dw2, dh2, dfps2),
                    "ir": (iw2, ih2, ifps2),
                    "ir_left": self.enable_ir_left,
                    "ir_right": self.enable_ir_right,
                }

                depth_sensor = profile.get_device().first_depth_sensor()
                self.depth_scale = float(depth_sensor.get_depth_scale())

                color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
                self.color_intr = color_stream.get_intrinsics()

                # warm-up
                for _ in range(max(0, int(warmup_frames))):
                    self.pipeline.wait_for_frames()

                print(f"[RealSense] started: color={cw2}x{ch2}@{cfps2}, depth={dw2}x{dh2}@{dfps2}, ir={iw2}x{ih2}@{ifps2} (L={self.enable_ir_left}, R={self.enable_ir_right})")
                return
            except Exception as e:
                last_err = e
                print(f"[RealSense] start failed: color={cw2}x{ch2}@{cfps2}, depth={dw2}x{dh2}@{dfps2}, ir={iw2}x{ih2}@{ifps2} -> {e}")

        raise RuntimeError(f"RealSense start failed for all candidates. Last error: {last_err}")

    def stop(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass

    def grab_aligned(self, timeout_ms: int = 2000) -> Dict[str, Any]:
        """Grab ONE frameset. Returns dict with color_bgr, depth_aligned_z16, ir_left_u8, ir_right_u8 + meta."""
        fs = self.pipeline.wait_for_frames(timeout_ms)

        # Keep IR from original frameset; align affects depth->color.
        ir_left = fs.get_infrared_frame(1) if self.enable_ir_left else None
        ir_right = fs.get_infrared_frame(2) if self.enable_ir_right else None

        aligned = self.align_to_color.process(fs)
        color = aligned.get_color_frame()
        depth = aligned.get_depth_frame()

        if not color or not depth:
            raise RuntimeError("Missing color/depth frame")

        color_bgr = np.asanyarray(color.get_data())  # bgr8
        depth_z16 = np.asanyarray(depth.get_data()).astype(np.uint16)  # aligned z16

        out: Dict[str, Any] = {
            "color_bgr": color_bgr,
            "depth_aligned_z16": depth_z16,
            "ir_left_u8": np.asanyarray(ir_left.get_data()).copy() if ir_left else None,
            "ir_right_u8": np.asanyarray(ir_right.get_data()).copy() if ir_right else None,
            "meta": {
                "t_system": time.time(),
                "depth_scale_m": self.depth_scale,
                "active_cfg": self.active_cfg,
                "rs_timestamp_ms": {
                    "color": float(color.get_timestamp()),
                    "depth_aligned": float(depth.get_timestamp()),
                    "ir_left": float(ir_left.get_timestamp()) if ir_left else None,
                    "ir_right": float(ir_right.get_timestamp()) if ir_right else None,
                },
                "rs_frame_number": {
                    "color": int(color.get_frame_number()),
                    "depth_aligned": int(depth.get_frame_number()),
                    "ir_left": int(ir_left.get_frame_number()) if ir_left else None,
                    "ir_right": int(ir_right.get_frame_number()) if ir_right else None,
                },
            },
        }
        return out


# -------------------------
# IO helpers
# -------------------------
def mkdirs(out_dir: Path) -> Dict[str, Path]:
    d = {
        "rgb": out_dir / "rgb",
        "depth": out_dir / "depth_aligned_mm",
        "ir_left": out_dir / "ir_left",
        "ir_right": out_dir / "ir_right",
        "meta": out_dir / "meta",
    }
    for p in d.values():
        p.mkdir(parents=True, exist_ok=True)
    return d


def resume_index(folders: Dict[str, Path]) -> int:
    # Prefer meta/*.json, else rgb/*.png
    mx = -1
    for p in folders["meta"].glob("*.json"):
        try:
            mx = max(mx, int(p.stem))
        except Exception:
            pass
    if mx >= 0:
        return mx + 1

    for p in folders["rgb"].glob("*.png"):
        try:
            mx = max(mx, int(p.stem))
        except Exception:
            pass
    return mx + 1 if mx >= 0 else 0


def depth_z16_to_mm(depth_z16: np.ndarray, depth_scale_m: float) -> np.ndarray:
    # mm = z16 * scale(m) * 1000
    mm = depth_z16.astype(np.float32) * float(depth_scale_m) * 1000.0
    mm = np.clip(mm, 0.0, 65535.0).astype(np.uint16)
    return mm


def write_csv_header_if_needed(csv_path: Path):
    if csv_path.exists() and csv_path.stat().st_size > 0:
        return
    csv_path.write_text(
        "idx,t_sec,x,y,yaw_rad,rgb_file,depth_file,ir_left_file,ir_right_file\n",
        encoding="utf-8",
    )


# -------------------------
# Preview utilities
# -------------------------
def depth_preview_u8(depth_mm: np.ndarray, max_mm: int = 10000) -> np.ndarray:
    d = depth_mm.astype(np.float32)
    valid = d > 0
    out = np.zeros_like(d, dtype=np.uint8)
    if np.any(valid):
        dn = np.clip(d, 0.0, float(max_mm)) / float(max_mm)
        out[valid] = ((1.0 - dn[valid]) * 255.0).astype(np.uint8)
    return out


# -------------------------
# Main
# -------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=str, default="dataset_out")
    parser.add_argument("--resume", action="store_true")

    parser.add_argument("--serial", type=str, default=None, help="RealSense serial (optional)")

    parser.add_argument("--color", type=str, default="640,480,15", help="w,h,fps")
    parser.add_argument("--depth", type=str, default="640,480,15", help="w,h,fps")
    parser.add_argument("--ir", type=str, default="640,480,15", help="w,h,fps")

    parser.add_argument("--ir-mode", type=str, default="both", choices=["both", "left", "right", "off"])

    parser.add_argument("--kachaka-endpoint", type=str, default="192.168.0.157:26400")

    parser.add_argument("--png-level", type=int, default=1, help="0(fast)..9(small)")
    parser.add_argument("--save-meta", action="store_true")

    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--preview-max-depth-m", type=float, default=3.0)

    args = parser.parse_args()

    try:
        cv2.setNumThreads(0)
    except Exception:
        pass

    def parse_whfps(s: str):
        a = [int(x.strip()) for x in s.split(",")]
        if len(a) != 3:
            raise ValueError("expect w,h,fps")
        return (a[0], a[1], a[2])

    color_whfps = parse_whfps(args.color)
    depth_whfps = parse_whfps(args.depth)
    ir_whfps = parse_whfps(args.ir)

    ir_left = args.ir_mode in ("both", "left")
    ir_right = args.ir_mode in ("both", "right")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    folders = mkdirs(out_dir)

    idx = resume_index(folders) if args.resume else 0
    print(f"[INFO] start idx = {idx}")

    poses_csv = out_dir / "poses.csv"
    write_csv_header_if_needed(poses_csv)

    # Init Kachaka pose client
    pose_client = KachakaPoseClient(args.kachaka_endpoint)

    # Init RealSense
    rs_collector = RealSenseCollector(serial=args.serial, enable_ir_left=ir_left, enable_ir_right=ir_right)
    rs_collector.start(color_whfps=color_whfps, depth_whfps=depth_whfps, ir_whfps=ir_whfps)

    png_params = [cv2.IMWRITE_PNG_COMPRESSION, int(args.png_level)]

    def capture_once():
        nonlocal idx
        print("[CAPTURE] capturing...")

        # 1) Grab frames (this is the success criterion)
        frames = rs_collector.grab_aligned(timeout_ms=2000)
        t_capture = time.time()

        color_bgr = frames["color_bgr"]
        depth_mm = depth_z16_to_mm(frames["depth_aligned_z16"], frames["meta"]["depth_scale_m"])
        ir_l = frames["ir_left_u8"]
        ir_r = frames["ir_right_u8"]

        # 2) Save images
        name = f"{idx:06d}.png"
        rgb_rel = f"rgb/{name}"
        depth_rel = f"depth_aligned_mm/{name}"
        ir_l_rel = f"ir_left/{name}" if ir_l is not None else ""
        ir_r_rel = f"ir_right/{name}" if ir_r is not None else ""

        ok_rgb = cv2.imwrite(str(folders["rgb"] / name), color_bgr, png_params)
        ok_d = cv2.imwrite(str(folders["depth"] / name), depth_mm, png_params)
        ok_l = True
        ok_r = True
        if ir_l is not None:
            ok_l = cv2.imwrite(str(folders["ir_left"] / name), ir_l, png_params)
        if ir_r is not None:
            ok_r = cv2.imwrite(str(folders["ir_right"] / name), ir_r, png_params)

        if not (ok_rgb and ok_d and ok_l and ok_r):
            raise RuntimeError("cv2.imwrite failed (disk permission? out of space?)")

        # 3) After we confirm frames are acquired & saved, fetch pose
        x, y, yaw = pose_client.get_pose_sync(timeout_s=1.5)

        # 4) Write CSV row ("previous" common format: t_sec,x,y,yaw_rad)
        with poses_csv.open("a", encoding="utf-8", buffering=1) as f:
            f.write(
                f"{idx},{t_capture:.6f},{x:.6f},{y:.6f},{yaw:.9f},{rgb_rel},{depth_rel},{ir_l_rel},{ir_r_rel}\n"
            )

        # 5) Optional meta
        if args.save_meta:
            meta = {
                "idx": idx,
                "t_capture": t_capture,
                "realsense": frames["meta"],
                "files": {
                    "rgb": rgb_rel,
                    "depth_aligned_mm": depth_rel,
                    "ir_left": ir_l_rel,
                    "ir_right": ir_r_rel,
                },
                "kachaka_pose": {"x": x, "y": y, "yaw_rad": yaw},
            }
            (folders["meta"] / f"{idx:06d}.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

        print(f"[DONE] idx={idx:06d} saved. pose=({x:.3f},{y:.3f},{yaw:.3f})\a")
        idx += 1

    try:
        if args.no_gui or os.environ.get("DISPLAY", "") == "":
            print("[CLI] Input 'c' + Enter to capture, 'q' + Enter to quit")
            while True:
                s = input().strip().lower()
                if s == "q":
                    break
                if s == "c" or s == "":
                    try:
                        capture_once()
                    except Exception as e:
                        print(f"[ERROR] capture failed: {e}")
        else:
            win = "RealSense Preview (press 'c' capture, 'q' quit)"
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)

            max_mm = int(float(args.preview_max_depth_m) * 1000.0)

            while True:
                # preview grab (do not align repeatedly on preview if you want max FPS)
                try:
                    frames = rs_collector.grab_aligned(timeout_ms=2000)
                except Exception as e:
                    print(f"[WARN] preview grab failed: {e}")
                    continue

                color = frames["color_bgr"]
                depth_mm = depth_z16_to_mm(frames["depth_aligned_z16"], frames["meta"]["depth_scale_m"])
                dgray = depth_preview_u8(depth_mm)
                dgray_bgr = cv2.cvtColor(dgray, cv2.COLOR_GRAY2BGR)

                # Make a simple mosaic: [RGB | DepthGray | IR-left]
                panes = [color, dgray_bgr]

                if frames["ir_left_u8"] is not None:
                    irl = frames["ir_left_u8"]
                    panes.append(cv2.cvtColor(irl, cv2.COLOR_GRAY2BGR))
                elif frames["ir_right_u8"] is not None:
                    irr = frames["ir_right_u8"]
                    panes.append(cv2.cvtColor(irr, cv2.COLOR_GRAY2BGR))

                # Resize panes to same height
                h = min(p.shape[0] for p in panes)
                panes2 = []
                for p in panes:
                    if p.shape[0] != h:
                        w = int(p.shape[1] * (h / p.shape[0]))
                        p = cv2.resize(p, (w, h), interpolation=cv2.INTER_AREA)
                    panes2.append(p)

                view = np.concatenate(panes2, axis=1)
                cv2.imshow(win, view)

                k = cv2.waitKey(1) & 0xFF
                if k == ord('q'):
                    break
                if k == ord('c'):
                    try:
                        capture_once()
                    except Exception as e:
                        print(f"[ERROR] capture failed: {e}")

            cv2.destroyAllWindows()

    finally:
        try:
            rs_collector.stop()
        except Exception:
            pass
        try:
            pose_client.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
