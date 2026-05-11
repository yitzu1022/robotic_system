import argparse
import contextlib
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2  # type: ignore[import-not-found]
import numpy as np

try:
    import asyncio
    import kachaka_api  # type: ignore[import-not-found]
except ModuleNotFoundError:
    asyncio = None
    kachaka_api = None

try:
    import pyrealsense2 as rs  # type: ignore[import-not-found]
except ModuleNotFoundError:
    rs = None


MAX_POSE_TIME_DIFF = 0.05
DEFAULT_CAPTURE_FPS = 15
DEFAULT_POSE_HZ = 30

pose_buffer = deque(maxlen=5000)
pose_lock = threading.Lock()
stop_event = threading.Event()


class KachakaPoseClient:
    def __init__(self, endpoint: str):
        if kachaka_api is None or asyncio is None:
            raise RuntimeError("kachaka_api not available. Install kachaka-api first.")

        self.endpoint = endpoint
        self._loop: Optional[Any] = None
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
                self._client = kachaka_api.aio.KachakaApiClient()

        loop.run_until_complete(init_client())
        self._ready.set()

        async def keep_alive():
            while not self._stop.is_set():
                await asyncio.sleep(0.05)

        try:
            loop.run_until_complete(keep_alive())
        finally:
            with contextlib.suppress(Exception):
                loop.stop()
            with contextlib.suppress(Exception):
                loop.close()

    def close(self):
        self._stop.set()
        with contextlib.suppress(Exception):
            self._thread.join(timeout=2.0)

    def get_pose_sync(self, timeout_s: float = 1.5) -> Tuple[float, float, float]:
        if self._loop is None or self._client is None:
            raise RuntimeError("KachakaPoseClient not ready")

        async def _get():
            return await self._client.get_robot_pose()

        fut = asyncio.run_coroutine_threadsafe(_get(), self._loop)
        pose = fut.result(timeout=timeout_s)
        return float(pose.x), float(pose.y), float(pose.theta)


class RealSenseCollector:
    def __init__(self, serial: Optional[str] = None):
        if rs is None:
            raise RuntimeError("pyrealsense2 not available. Install librealsense + python bindings")

        self.serial = serial
        self.pipeline = rs.pipeline()
        self.align_to_color = rs.align(rs.stream.color)
        self.profile = None
        self.depth_scale = 0.001

    def start(self, width: int, height: int, fps: int, warmup_frames: int = 15):
        candidates = [
            (width, height, fps),
            (640, 480, fps),
            (848, 480, fps),
            (640, 480, 15),
            (848, 480, 15),
        ]

        last_err = None
        for cw, ch, cfps in candidates:
            try:
                self.pipeline = rs.pipeline()
                cfg = rs.config()
                if self.serial:
                    cfg.enable_device(self.serial)

                cfg.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, cfps)
                cfg.enable_stream(rs.stream.depth, cw, ch, rs.format.z16, cfps)

                profile = self.pipeline.start(cfg)
                self.profile = profile

                depth_sensor = profile.get_device().first_depth_sensor()
                self.depth_scale = float(depth_sensor.get_depth_scale())

                for _ in range(max(0, int(warmup_frames))):
                    self.pipeline.wait_for_frames()

                print(f"[RealSense] started: color={cw}x{ch}@{cfps}, depth={cw}x{ch}@{cfps}")
                return cw, ch, cfps
            except (RuntimeError, TypeError, ValueError, OSError) as e:
                last_err = e
                print(f"[RealSense] start failed: color={cw}x{ch}@{cfps} -> {e}")

        raise RuntimeError(f"RealSense start failed for all candidates. Last error: {last_err}")

    def stop(self):
        with contextlib.suppress(Exception):
            self.pipeline.stop()

    def grab_aligned(self, timeout_ms: int = 2000) -> Dict[str, Any]:
        frames = self.pipeline.wait_for_frames(timeout_ms)
        aligned_frames = self.align_to_color.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            raise RuntimeError("Missing color/depth frame")

        color_bgr = np.asanyarray(color_frame.get_data())
        depth_z16 = np.asanyarray(depth_frame.get_data()).astype(np.uint16)

        return {
            "color_bgr": color_bgr,
            "depth_aligned_z16": depth_z16,
            "meta": {
                "t_system": time.time(),
                "depth_scale_m": self.depth_scale,
                "rs_timestamp_ms": {
                    "color": float(color_frame.get_timestamp()),
                    "depth_aligned": float(depth_frame.get_timestamp()),
                },
                "rs_frame_number": {
                    "color": int(color_frame.get_frame_number()),
                    "depth_aligned": int(depth_frame.get_frame_number()),
                },
            },
        }


def mkdirs(out_dir: Path) -> Dict[str, Path]:
    folders = {
        "poses": out_dir / "poses",
        "rgb": out_dir / "rgb",
        "depth": out_dir / "depth",
    }
    for path in folders.values():
        path.mkdir(parents=True, exist_ok=True)
    return folders


def depth_z16_to_mm(depth_z16: np.ndarray, depth_scale_m: float) -> np.ndarray:
    mm = depth_z16.astype(np.float32) * float(depth_scale_m) * 1000.0
    return np.clip(mm, 0.0, 65535.0).astype(np.uint16)


def pose_logger(client: KachakaPoseClient, pose_hz: int):
    period = 1.0 / float(pose_hz)

    while not stop_event.is_set():
        t0 = time.time()

        try:
            x, y, yaw = client.get_pose_sync(timeout_s=1.5)
            item = {
                "t_system": t0,
                "x": x,
                "y": y,
                "yaw": yaw,
            }
            with pose_lock:
                pose_buffer.append(item)
        except (RuntimeError, TimeoutError, TypeError, OSError) as e:
            print(f"[Kachaka pose error] {e}")

        elapsed = time.time() - t0
        time.sleep(max(0.0, period - elapsed))


def find_nearest_pose(t_query: float):
    with pose_lock:
        if len(pose_buffer) == 0:
            return None

        nearest = min(pose_buffer, key=lambda p: abs(p["t_system"] - t_query))

    dt = abs(nearest["t_system"] - t_query)
    if dt > MAX_POSE_TIME_DIFF:
        return None

    return nearest


def timestamp_tag(t_sec: float) -> str:
    return f"{int(round(t_sec * 1000.0)):013d}"


def write_pose_file(path: Path, t_sec: float, pose: Dict[str, float]):
    path.write_text(
        f"{t_sec:.6f},{pose['x']:.6f},{pose['y']:.6f},{pose['yaw']:.9f}\n",
        encoding="utf-8",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=str, default="dataset_continue")
    parser.add_argument("--fps", type=int, default=DEFAULT_CAPTURE_FPS)
    parser.add_argument("--pose-hz", type=int, default=DEFAULT_POSE_HZ)
    parser.add_argument("--serial", type=str, default=None)
    parser.add_argument("--kachaka-endpoint", type=str, default="192.168.1.19:26400")
    args = parser.parse_args()

    try:
        cv2.setNumThreads(0)
    except (TypeError, ValueError):
        pass

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    folders = mkdirs(out_dir)

    pose_client = KachakaPoseClient(args.kachaka_endpoint)
    pose_thread = threading.Thread(target=pose_logger, args=(pose_client, args.pose_hz), daemon=True)
    pose_thread.start()

    rs_collector = RealSenseCollector(serial=args.serial)
    width, height, fps = rs_collector.start(width=640, height=480, fps=max(1, int(args.fps)))
    print(f"[INFO] collecting at {fps} FPS with frame size {width}x{height}")

    png_params = [cv2.IMWRITE_PNG_COMPRESSION, 1]
    idx = 1
    period = 1.0 / float(max(1, int(args.fps)))
    next_capture = time.monotonic()

    try:
        print("[INFO] auto capture started. Press Ctrl-C to stop.")
        while True:
            now = time.monotonic()
            if now < next_capture:
                time.sleep(next_capture - now)
                continue

            t_capture = time.time()
            capture = rs_collector.grab_aligned(timeout_ms=2000)
            pose = find_nearest_pose(t_capture)

            if pose is None:
                print("[Skip] no synchronized Kachaka pose")
                next_capture += period
                continue

            tag = timestamp_tag(t_capture)
            prefix = f"{idx:04d}_{tag}"

            rgb_path = folders["rgb"] / f"{prefix}_rgb.png"
            depth_path = folders["depth"] / f"{prefix}_depth.png"
            pose_path = folders["poses"] / f"{prefix}_pose.txt"

            depth_mm = depth_z16_to_mm(capture["depth_aligned_z16"], capture["meta"]["depth_scale_m"])

            ok_rgb = cv2.imwrite(str(rgb_path), capture["color_bgr"], png_params)
            ok_depth = cv2.imwrite(str(depth_path), depth_mm, png_params)
            if not ok_rgb or not ok_depth:
                raise RuntimeError("cv2.imwrite failed (disk permission or out of space)")

            write_pose_file(pose_path, t_capture, pose)

            pose_dt = abs(t_capture - pose["t_system"])
            print(f"[Saved] {prefix} pose_dt={pose_dt:.4f}s")
            idx += 1
            next_capture += period

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        stop_event.set()
        with contextlib.suppress(Exception):
            pose_thread.join(timeout=1.0)
        with contextlib.suppress(Exception):
            rs_collector.stop()
        with contextlib.suppress(Exception):
            pose_client.close()


if __name__ == "__main__":
    main()