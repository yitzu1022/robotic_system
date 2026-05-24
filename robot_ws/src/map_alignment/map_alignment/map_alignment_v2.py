#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import yaml
import numpy as np

from pathlib import Path
from typing import List, Dict, Tuple, Optional

# SciPy
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

# ROS2
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


# =============================================================================
# 基礎數學工具
# =============================================================================

def normalize_angle(rad: float) -> float:
    """把角度正規化到 (-pi, pi]"""
    return float(np.arctan2(np.sin(rad), np.cos(rad)))


def extract_yaw(T: np.ndarray) -> float:
    """從 4x4 取 yaw（假設 z 軸朝上）"""
    return float(np.arctan2(T[1, 0], T[0, 0]))


def quaternion_to_rotation_matrix(q: np.ndarray, quat_format: str = "xyzw") -> np.ndarray:
    """
    四元數轉旋轉矩陣
    Args:
        q: 長度 4
        quat_format: "xyzw" 或 "wxyz"
    """
    q = np.asarray(q).reshape(4,)
    if quat_format.lower() == "xyzw":
        x, y, z, w = q
    elif quat_format.lower() == "wxyz":
        w, x, y, z = q
    else:
        raise ValueError(f"quat_format 必須是 'xyzw' 或 'wxyz'，但得到: {quat_format}")
    return Rotation.from_quat([x, y, z, w]).as_matrix()


def rotation_matrix_to_quaternion(Rm: np.ndarray) -> np.ndarray:
    """旋轉矩陣轉四元數 [x,y,z,w]"""
    return Rotation.from_matrix(Rm).as_quat()



def matrix_to_transform(T: np.ndarray, parent_frame: str, child_frame: str, stamp_msg) -> TransformStamped:
    """4x4 (T_parent_child) -> ROS TransformStamped"""
    trans = TransformStamped()
    trans.header.stamp = stamp_msg
    trans.header.frame_id = parent_frame
    trans.child_frame_id = child_frame

    trans.transform.translation.x = float(T[0, 3])
    trans.transform.translation.y = float(T[1, 3])
    trans.transform.translation.z = float(T[2, 3])

    q = rotation_matrix_to_quaternion(T[:3, :3])  # xyzw
    trans.transform.rotation.x = float(q[0])
    trans.transform.rotation.y = float(q[1])
    trans.transform.rotation.z = float(q[2])
    trans.transform.rotation.w = float(q[3])
    return trans


def make_T_se2(tx: float, ty: float, yaw: float, tz: float = 0.0) -> np.ndarray:
    """建立 SE(2) (放進 4x4)，z 平移可選但不做 roll/pitch"""
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    T[0, 3] = tx
    T[1, 3] = ty
    T[2, 3] = tz
    return T


def apply_similarity_2d(R2: np.ndarray, t2: np.ndarray, s: float, p: np.ndarray) -> np.ndarray:
    """
    p: (...,2)
    回傳: (...,2)  = s * (R2 @ p.T).T + t2
    """
    return (s * (R2 @ p.T).T) + t2.reshape(1, 2)


# =============================================================================
# 資料載入
# =============================================================================

class DataLoader:
    """
    pose 檔支援格式：
    A) timestamp tx ty tz qx qy qz qw
    B) tx ty tz qx qy qz qw  (無 timestamp，會用 index)
    """

    @staticmethod
    def load_pose_file(
        file_path: str,
        quat_format: str = "xyzw",
        pose_convention: str = "c2w",
    ) -> Tuple[np.ndarray, List[np.ndarray]]:
        """
        讀 pose 檔並回傳：
            timestamps: (N,)
            poses: List[4x4]，統一成 T_world_camera (T_parent_child) 的形式

        pose_convention:
            - "c2w": 檔案提供的是 camera->world，也就是 T_world_camera (常見 c2w)
            - "w2c": 檔案提供的是 world->camera，也就是 T_camera_world，需要 invert 才變 T_world_camera
        """
        data = np.loadtxt(file_path)
        if data.ndim == 1:
            data = data.reshape(1, -1)

        if data.shape[1] == 8:
            ts = data[:, 0].astype(float)
            pose_data = data[:, 1:]
        elif data.shape[1] == 7:
            ts = np.arange(len(data), dtype=float)
            pose_data = data
        else:
            raise ValueError(f"不支援的 pose 檔格式：每列欄位數={data.shape[1]}（需 7 或 8）")

        poses: List[np.ndarray] = []
        for row in pose_data:
            tx, ty, tz = row[0:3]
            q = row[3:7]
            T = np.eye(4)
            T[:3, 3] = np.array([tx, ty, tz], dtype=float)
            T[:3, :3] = quaternion_to_rotation_matrix(q, quat_format=quat_format)

            if pose_convention.lower() == "c2w":
                # 檔案就是 T_world_camera
                Twc = T
            elif pose_convention.lower() == "w2c":
                # 檔案是 T_camera_world -> invert
                Twc = np.linalg.inv(T)
            else:
                raise ValueError("pose_convention 必須是 'c2w' 或 'w2c'")

            poses.append(Twc)

        return ts, poses

    @staticmethod
    def load_calibration(file_path: str, quat_format: str = "xyzw") -> np.ndarray:
        """
        讀 base->camera 外參
        支援：
            - YAML/JSON: {translation:[x,y,z], rotation:[...4...]}
            - TXT: 4x4 matrix
        rotation 的 quat_format 由參數指定（不要自動猜）
        """
        path = Path(file_path)
        if path.suffix in [".yaml", ".yml"]:
            with open(file_path, "r") as f:
                data = yaml.safe_load(f)
            T = np.eye(4)
            T[:3, 3] = np.array(data["translation"], dtype=float)
            T[:3, :3] = quaternion_to_rotation_matrix(np.array(data["rotation"], dtype=float), quat_format=quat_format)
            return T

        if path.suffix == ".json":
            with open(file_path, "r") as f:
                data = json.load(f)
            T = np.eye(4)
            T[:3, 3] = np.array(data["translation"], dtype=float)
            T[:3, :3] = quaternion_to_rotation_matrix(np.array(data["rotation"], dtype=float), quat_format=quat_format)
            return T

        if path.suffix == ".txt":
            T = np.loadtxt(file_path)
            if T.shape != (4, 4):
                raise ValueError("TXT 外參必須是 4x4 矩陣")
            return T

        raise ValueError(f"不支援的外參格式: {path.suffix}")

# =============================================================================
# 時間同步（含 offset 搜尋、scale、起點歸一化）
# =============================================================================

class TrajectorySync:

    @staticmethod
    def normalize_time(t: np.ndarray, do_normalize: bool = True) -> np.ndarray:
        if not do_normalize:
            return t.astype(float)
        t = t.astype(float)
        return t - t[0]

    @staticmethod
    def synchronize_nearest(
        t2: np.ndarray,
        poses2: List[np.ndarray],
        t3: np.ndarray,
        poses3: List[np.ndarray],
        max_time_diff: float,
    ) -> List[Dict]:
        """
        nearest-neighbor sync
        假設 t2, t3 已經在同 timebase（至少相對時間）且已排序
        """
        synced: List[Dict] = []
        t3 = np.asarray(t3, dtype=float)

        for i, (ti, p2) in enumerate(zip(t2, poses2)):
            j = int(np.argmin(np.abs(t3 - ti)))
            dt = float(abs(t3[j] - ti))
            if dt <= max_time_diff:
                synced.append({
                    "index_2d": i,
                    "index_3d": j,
                    "time_2d": float(ti),
                    "time_3d": float(t3[j]),
                    "time_diff": dt,
                    "pose_2d": p2,
                    "pose_3d": poses3[j],
                })
        return synced

    @staticmethod
    def estimate_time_offset_grid(
        t2: np.ndarray,
        poses2: List[np.ndarray],
        t3: np.ndarray,
        poses3: List[np.ndarray],
        max_time_diff: float,
        search_range: float = 2.0,
        coarse_step: float = 0.05,
        fine_step: float = 0.005,
    ) -> float:
        """
        以「配對數最大」為目標搜尋 offset Δ，使得 (t3 + Δ) 與 t2 更匹配
        回傳最佳 Δ（加到 3D 時間上）
        """
        def score_for(delta: float) -> Tuple[int, float]:
            synced = TrajectorySync.synchronize_nearest(t2, poses2, t3 + delta, poses3, max_time_diff)
            if not synced:
                return (0, 1e9)
            dts = [p["time_diff"] for p in synced]
            return (len(synced), float(np.mean(dts)))

        # coarse
        deltas = np.arange(-search_range, search_range + 1e-9, coarse_step)
        best = (-1, 1e9)
        best_delta = 0.0
        for d in deltas:
            sc = score_for(float(d))
            if (sc[0] > best[0]) or (sc[0] == best[0] and sc[1] < best[1]):
                best, best_delta = sc, float(d)

        # fine around coarse best
        deltas_f = np.arange(best_delta - coarse_step, best_delta + coarse_step + 1e-9, fine_step)
        best2 = (-1, 1e9)
        best_delta2 = best_delta
        for d in deltas_f:
            sc = score_for(float(d))
            if (sc[0] > best2[0]) or (sc[0] == best2[0] and sc[1] < best2[1]):
                best2, best_delta2 = sc, float(d)

        return float(best_delta2)

    @staticmethod
    def report_sync_quality(synced_pairs: List[Dict]) -> str:
        if not synced_pairs:
            return "⚠️ 同步品質報告：沒有任何成功配對（請檢查 time scale / time offset / max_time_diff）"

        time_diffs = np.array([p["time_diff"] for p in synced_pairs], dtype=float)
        return (
            "同步品質報告:\n"
            "============\n"
            f"總配對數: {len(synced_pairs)}\n"
            f"平均時間差: {np.mean(time_diffs)*1000:.2f} ms\n"
            f"最大時間差: {np.max(time_diffs)*1000:.2f} ms\n"
            f"標準差: {np.std(time_diffs)*1000:.2f} ms\n"
        )


# =============================================================================
# 座標轉換（camera pose -> base pose）
# =============================================================================

class CoordinateTransform:

    @staticmethod
    def camera_pose_to_base_pose(T_world_camera: np.ndarray, T_base_camera: np.ndarray) -> np.ndarray:
        """
        統一用 TF 風格：T_parent_child
        已知：
            T_world_camera (world->camera)
            T_base_camera (base->camera)
        則：
            T_world_base = T_world_camera * inv(T_base_camera)
        """
        return T_world_camera @ np.linalg.inv(T_base_camera)

    @staticmethod
    def transform_trajectory_camera_to_base(poses_camera: List[np.ndarray], T_base_camera: np.ndarray) -> List[np.ndarray]:
        return [CoordinateTransform.camera_pose_to_base_pose(Twc, T_base_camera) for Twc in poses_camera]


# =============================================================================
# 對齊：SE(2) + (可選) scale
# =============================================================================

class MapAlignment:

    @staticmethod
    def estimate_se2(
        poses_2d: List[np.ndarray],
        poses_3d: List[np.ndarray],
        estimate_scale: bool = False,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        從 xy 點估計 R(2), t(2)，可選 scale（Umeyama-like similarity in 2D）
        回傳:
            R2: 2x2
            t2: (2,)
            s: float
        """
        P = np.array([[T[0, 3], T[1, 3]] for T in poses_3d], dtype=float)  # source (3D map projected to xy)
        Q = np.array([[T[0, 3], T[1, 3]] for T in poses_2d], dtype=float)  # target (2D map xy)

        muP = P.mean(axis=0)
        muQ = Q.mean(axis=0)
        X = P - muP
        Y = Q - muQ

        H = X.T @ Y
        U, S, Vt = np.linalg.svd(H)
        R2 = Vt.T @ U.T
        if np.linalg.det(R2) < 0:
            Vt[-1, :] *= -1
            R2 = Vt.T @ U.T

        if estimate_scale:
            # s = trace(S) / sum(||X||^2)
            varP = np.sum(X**2)
            s = float(np.sum(S) / (varP + 1e-12))
        else:
            s = 1.0

        t2 = muQ - s * (R2 @ muP)
        return R2, t2, s

    @staticmethod
    def ransac_se2(
        poses_2d: List[np.ndarray],
        poses_3d: List[np.ndarray],
        ransac_iterations: int = 1000,
        inlier_threshold_xy: float = 0.10,
        estimate_scale: bool = False,
        sample_size: int = 6,
    ) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray]:
        """
        RANSAC 估 SE2 (可選 scale)，只用 xy 誤差判斷內點
        """
        n = len(poses_2d)
        if n == 0:
            return np.eye(2), np.zeros(2), 1.0, np.zeros(0, dtype=bool)

        best_inliers = np.zeros(n, dtype=bool)
        best_score = -1
        best_R2, best_t2, best_s = np.eye(2), np.zeros(2), 1.0

        ss = min(sample_size, n)

        for _ in range(int(ransac_iterations)):
            idx = np.random.choice(n, ss, replace=False) if n > ss else np.arange(n)
            sample_2d = [poses_2d[i] for i in idx]
            sample_3d = [poses_3d[i] for i in idx]

            try:
                R2, t2, s = MapAlignment.estimate_se2(sample_2d, sample_3d, estimate_scale=estimate_scale)
            except Exception:
                continue

            # evaluate all
            P = np.array([[T[0, 3], T[1, 3]] for T in poses_3d], dtype=float)
            Q = np.array([[T[0, 3], T[1, 3]] for T in poses_2d], dtype=float)
            P_pred = apply_similarity_2d(R2, t2, s, P)
            err = np.linalg.norm(Q - P_pred, axis=1)
            inliers = err <= inlier_threshold_xy
            score = int(inliers.sum())

            if score > best_score:
                best_score = score
                best_inliers = inliers
                best_R2, best_t2, best_s = R2, t2, s

        # re-fit with all inliers
        if best_score <= 0:
            return best_R2, best_t2, best_s, best_inliers

        in2 = [poses_2d[i] for i in np.where(best_inliers)[0]]
        in3 = [poses_3d[i] for i in np.where(best_inliers)[0]]
        R2, t2, s = MapAlignment.estimate_se2(in2, in3, estimate_scale=estimate_scale)
        return R2, t2, s, best_inliers

    @staticmethod
    def refine_se2(
        poses_2d: List[np.ndarray],
        poses_3d: List[np.ndarray],
        R2_init: np.ndarray,
        t2_init: np.ndarray,
        s_init: float,
        estimate_scale: bool = False,
        use_yaw: bool = True,
        loss: str = "huber",
        w_xy: float = 1.0,
        w_yaw: float = 0.3,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        非線性精化：參數 (tx, ty, yaw, [log_s])
        只用 xy + (可選) yaw residual
        """
        yaw0 = float(np.arctan2(R2_init[1, 0], R2_init[0, 0]))
        if estimate_scale:
            x0 = np.array([t2_init[0], t2_init[1], yaw0, np.log(max(s_init, 1e-9))], dtype=float)
        else:
            x0 = np.array([t2_init[0], t2_init[1], yaw0], dtype=float)

        def build_params(x: np.ndarray) -> Tuple[float, float, float, float]:
            tx, ty, yaw = float(x[0]), float(x[1]), float(x[2])
            if estimate_scale:
                s = float(np.exp(x[3]))
            else:
                s = 1.0
            return tx, ty, yaw, s

        def residual(x: np.ndarray) -> np.ndarray:
            tx, ty, yaw, s = build_params(x)
            c, s_y = np.cos(yaw), np.sin(yaw)
            R2 = np.array([[c, -s_y],
                           [s_y,  c]], dtype=float)
            t2 = np.array([tx, ty], dtype=float)

            res = []
            for p2, p3 in zip(poses_2d, poses_3d):
                p3_xy = np.array([p3[0, 3], p3[1, 3]], dtype=float)
                p2_xy = np.array([p2[0, 3], p2[1, 3]], dtype=float)
                pred_xy = s * (R2 @ p3_xy) + t2
                e_xy = (p2_xy - pred_xy) * w_xy
                res.extend([e_xy[0], e_xy[1]])

                if use_yaw:
                    # 預測的 yaw： yaw_pred = yaw_align + yaw_3d
                    yaw2 = extract_yaw(p2)
                    yaw3 = extract_yaw(p3)
                    yaw_pred = normalize_angle(yaw + yaw3)
                    e_yaw = normalize_angle(yaw2 - yaw_pred) * w_yaw
                    res.append(e_yaw)

            return np.array(res, dtype=float)

        # robust loss 需要 trf/dogbox；這裡用 trf
        result = least_squares(
            residual,
            x0,
            method="trf",
            loss=loss,
            f_scale=1.0,
            verbose=0,
        )

        tx, ty, yaw, s_final = build_params(result.x)
        R2 = np.array([[np.cos(yaw), -np.sin(yaw)],
                       [np.sin(yaw),  np.cos(yaw)]], dtype=float)
        t2 = np.array([tx, ty], dtype=float)
        return R2, t2, s_final


# =============================================================================
# 品質與誤差評估（只看 xy + yaw）
# =============================================================================

class TrajectoryQualityChecker:

    @staticmethod
    def check_rotation_richness(poses: List[np.ndarray]) -> Dict:
        if len(poses) < 2:
            return {"total_rotation_deg": 0.0, "sufficient": False}

        yaws = np.array([extract_yaw(T) for T in poses], dtype=float)
        dy = np.array([normalize_angle(yaws[i] - yaws[i-1]) for i in range(1, len(yaws))], dtype=float)
        total = float(np.sum(np.abs(dy)))
        return {
            "total_rotation_deg": float(np.degrees(total)),
            "avg_rotation_deg": float(np.degrees(np.mean(np.abs(dy)))),
            "max_rotation_deg": float(np.degrees(np.max(np.abs(dy)))),
            "sufficient": total >= np.radians(90.0),
        }

    @staticmethod
    def check_coverage(poses: List[np.ndarray]) -> Dict:
        if not poses:
            return {"x_range": 0.0, "y_range": 0.0, "area": 0.0, "sufficient": False}
        xy = np.array([[T[0, 3], T[1, 3]] for T in poses], dtype=float)
        xr = float(xy[:, 0].max() - xy[:, 0].min())
        yr = float(xy[:, 1].max() - xy[:, 1].min())
        return {
            "x_range": xr,
            "y_range": yr,
            "area": xr * yr,
            "sufficient": (xr >= 2.0 and yr >= 2.0),
        }

    @staticmethod
    def generate_report(poses_2d: List[np.ndarray], poses_3d: List[np.ndarray]) -> str:
        rot2 = TrajectoryQualityChecker.check_rotation_richness(poses_2d)
        rot3 = TrajectoryQualityChecker.check_rotation_richness(poses_3d)
        cov2 = TrajectoryQualityChecker.check_coverage(poses_2d)
        cov3 = TrajectoryQualityChecker.check_coverage(poses_3d)

        def ok(rot, cov) -> bool:
            return bool(rot["sufficient"] and cov["sufficient"])

        report = (
            "軌跡品質報告:\n"
            "============\n"
            f"2D poses: N={len(poses_2d)} | rotation={rot2['total_rotation_deg']:.1f}° | "
            f"coverage={cov2['x_range']:.2f}m x {cov2['y_range']:.2f}m | "
            f"{'✓ 足夠' if ok(rot2, cov2) else '⚠️ 不足'}\n"
            f"3D poses: N={len(poses_3d)} | rotation={rot3['total_rotation_deg']:.1f}° | "
            f"coverage={cov3['x_range']:.2f}m x {cov3['y_range']:.2f}m | "
            f"{'✓ 足夠' if ok(rot3, cov3) else '⚠️ 不足'}\n"
        )
        if not rot2["sufficient"]:
            report += "建議：2D 軌跡轉彎不足，yaw 估計易退化\n"
        if not rot3["sufficient"]:
            report += "建議：3D 軌跡轉彎不足，yaw 對齊易不穩\n"
        if not cov2["sufficient"]:
            report += "建議：2D 覆蓋範圍不足，平移估計會弱\n"
        if not cov3["sufficient"]:
            report += "建議：3D 覆蓋範圍不足，平移估計會弱\n"
        return report


class AlignmentError:

    @staticmethod
    def compute(
        R2: np.ndarray,
        t2: np.ndarray,
        s: float,
        poses_2d: List[np.ndarray],
        poses_3d: List[np.ndarray],
        use_yaw: bool = True,
    ) -> Dict:
        P = np.array([[T[0, 3], T[1, 3]] for T in poses_3d], dtype=float)
        Q = np.array([[T[0, 3], T[1, 3]] for T in poses_2d], dtype=float)
        Pp = apply_similarity_2d(R2, t2, s, P)
        e_xy = np.linalg.norm(Q - Pp, axis=1)

        out = {
            "position_mean_m": float(np.mean(e_xy)) if len(e_xy) else 0.0,
            "position_std_m": float(np.std(e_xy)) if len(e_xy) else 0.0,
            "position_max_m": float(np.max(e_xy)) if len(e_xy) else 0.0,
            "n": int(len(e_xy)),
        }

        if use_yaw and poses_2d:
            yaw2 = np.array([extract_yaw(T) for T in poses_2d], dtype=float)
            yaw3 = np.array([extract_yaw(T) for T in poses_3d], dtype=float)
            yaw_align = float(np.arctan2(R2[1, 0], R2[0, 0]))
            yaw_pred = np.array([normalize_angle(yaw_align + y) for y in yaw3], dtype=float)
            ey = np.array([normalize_angle(a - b) for a, b in zip(yaw2, yaw_pred)], dtype=float)
            out.update({
                "yaw_mean_deg": float(np.degrees(np.mean(np.abs(ey)))),
                "yaw_std_deg": float(np.degrees(np.std(np.abs(ey)))),
                "yaw_max_deg": float(np.degrees(np.max(np.abs(ey)))),
            })
        return out

    @staticmethod
    def report(err: Dict) -> str:
        s = (
            "對齊誤差報告:\n"
            "============\n"
            f"XY 平均誤差: {err['position_mean_m']*100:.2f} cm\n"
            f"XY 標準差:  {err['position_std_m']*100:.2f} cm\n"
            f"XY 最大誤差: {err['position_max_m']*100:.2f} cm\n"
        )
        if "yaw_mean_deg" in err:
            s += (
                f"Yaw 平均誤差: {err['yaw_mean_deg']:.2f}°\n"
                f"Yaw 標準差:  {err['yaw_std_deg']:.2f}°\n"
                f"Yaw 最大誤差: {err['yaw_max_deg']:.2f}°\n"
            )
        pm = err["position_mean_m"]
        if pm < 0.05:
            s += "評估：✓ 對齊品質優秀\n"
        elif pm < 0.15:
            s += "評估：⚠️ 對齊品質一般\n"
        else:
            s += "評估：❌ 對齊品質較差（建議檢查 timebase / pose 方向 / 外參）\n"
        return s


# =============================================================================
# ROS2 Node（讀檔對齊 + 發布靜態 TF + 存檔）
# =============================================================================

class MapAlignmentNode(Node):

    def __init__(self):
        super().__init__("map_alignment_node_offline")

        self.get_logger().info("Map Alignment Node (offline inputs) 啟動")

        self.declare_parameters()

        self.static_broadcaster = StaticTransformBroadcaster(self)

        # run once
        self.run_alignment_pipeline()

    def declare_parameters(self):
        # files
        self.declare_parameter("poses_2d_file", "poses_2d.txt")
        self.declare_parameter("poses_3d_file", "poses_3d.txt")  # Uni3R camera poses
        self.declare_parameter("base_camera_calib_file", "base_camera.yaml")
        
        self.declare_parameter("output_file", "map_alignment.yaml")

        # frames
        self.declare_parameter("map_2d_frame", "map_2d")
        self.declare_parameter("map_3d_frame", "map_3d")

        # pose parsing
        self.declare_parameter("poses_2d_quat_format", "xyzw")   # xyzw / wxyz
        self.declare_parameter("poses_3d_quat_format", "xyzw")
        self.declare_parameter("poses_2d_pose_convention", "c2w")  # c2w / w2c
        self.declare_parameter("poses_3d_pose_convention", "c2w")  # c2w / w2c
        self.declare_parameter("calib_quat_format", "xyzw")

        # time handling
        self.declare_parameter("normalize_time", True)
        self.declare_parameter("time_scale_2d", 1.0)   # 若 2D ts 是 index，可設 1/fps
        self.declare_parameter("time_scale_3d", 1.0)
        self.declare_parameter("max_time_diff", 0.05)

        self.declare_parameter("enable_time_offset_search", True)
        self.declare_parameter("time_offset_search_range", 2.0)
        self.declare_parameter("time_offset_coarse_step", 0.05)
        self.declare_parameter("time_offset_fine_step", 0.005)

        # alignment
        self.declare_parameter("min_synced_pairs", 30)
        self.declare_parameter("ransac_iterations", 1500)
        self.declare_parameter("inlier_threshold_xy", 0.10)
        self.declare_parameter("estimate_scale", False)
        self.declare_parameter("use_yaw_residual", True)

        # output behavior
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("estimate_z_offset", True)  # 用 median(z2 - z3) 放到 TF 的 z

    def run_alignment_pipeline(self):
        log = self.get_logger()

        poses_2d_file = self.get_parameter("poses_2d_file").value
        poses_3d_file = self.get_parameter("poses_3d_file").value
        calib_file = self.get_parameter("base_camera_calib_file").value

        if not os.path.exists(poses_2d_file):
            raise FileNotFoundError(f"找不到 poses_2d_file: {poses_2d_file}")
        if not os.path.exists(poses_3d_file):
            raise FileNotFoundError(f"找不到 poses_3d_file: {poses_3d_file}")
        if not os.path.exists(calib_file):
            raise FileNotFoundError(f"找不到 base_camera_calib_file: {calib_file}")

        # ---- load poses ----
        log.info(f"讀取 2D poses: {poses_2d_file}")
        t2, poses2 = DataLoader.load_pose_file(
            poses_2d_file,
            quat_format=self.get_parameter("poses_2d_quat_format").value,
            pose_convention=self.get_parameter("poses_2d_pose_convention").value,
        )
        log.info(f"✓ 2D poses: N={len(poses2)}")

        log.info(f"讀取 3D poses (camera): {poses_3d_file}")
        t3, poses3_cam = DataLoader.load_pose_file(
            poses_3d_file,
            quat_format=self.get_parameter("poses_3d_quat_format").value,
            pose_convention=self.get_parameter("poses_3d_pose_convention").value,
        )
        log.info(f"✓ 3D camera poses: N={len(poses3_cam)}")

        log.info(f"讀取外參 base->camera: {calib_file}")
        T_base_camera = DataLoader.load_calibration(
            calib_file,
            quat_format=self.get_parameter("calib_quat_format").value,
        )
        log.info(f"✓ 外參平移: {T_base_camera[:3, 3].tolist()}")

        # camera -> base
        poses3_base = CoordinateTransform.transform_trajectory_camera_to_base(poses3_cam, T_base_camera)

        # ---- time handling ----
        normalize_time = bool(self.get_parameter("normalize_time").value)
        s2 = float(self.get_parameter("time_scale_2d").value)
        s3 = float(self.get_parameter("time_scale_3d").value)
        max_dt = float(self.get_parameter("max_time_diff").value)

        t2a = TrajectorySync.normalize_time(t2 * s2, do_normalize=normalize_time)
        t3a = TrajectorySync.normalize_time(t3 * s3, do_normalize=normalize_time)

        # optional offset search
        if bool(self.get_parameter("enable_time_offset_search").value):
            rng = float(self.get_parameter("time_offset_search_range").value)
            cst = float(self.get_parameter("time_offset_coarse_step").value)
            fst = float(self.get_parameter("time_offset_fine_step").value)
            delta = TrajectorySync.estimate_time_offset_grid(
                t2a, poses2, t3a, poses3_base,
                max_time_diff=max_dt,
                search_range=rng,
                coarse_step=cst,
                fine_step=fst,
            )
            log.info(f"估計 time offset Δ（加到 3D 時間）= {delta:+.4f} s")
            t3a = t3a + delta

        synced = TrajectorySync.synchronize_nearest(t2a, poses2, t3a, poses3_base, max_time_diff=max_dt)
        log.info(TrajectorySync.report_sync_quality(synced))

        if len(synced) < int(self.get_parameter("min_synced_pairs").value):
            log.error("同步配對數不足，停止。請檢查 time_scale / offset / max_time_diff")
            return

        poses2_s = [p["pose_2d"] for p in synced]
        poses3_s = [p["pose_3d"] for p in synced]

        # ---- quality report ----
        log.info(TrajectoryQualityChecker.generate_report(poses2_s, poses3_s))

        # ---- RANSAC align (SE2) ----
        estimate_scale = bool(self.get_parameter("estimate_scale").value)
        ransac_iters = int(self.get_parameter("ransac_iterations").value)
        thr = float(self.get_parameter("inlier_threshold_xy").value)

        R2, t2v, s_est, inliers = MapAlignment.ransac_se2(
            poses2_s, poses3_s,
            ransac_iterations=ransac_iters,
            inlier_threshold_xy=thr,
            estimate_scale=estimate_scale,
            sample_size=6,
        )

        in_ratio = float(inliers.sum() / max(len(inliers), 1))
        log.info(f"RANSAC 內點比例: {in_ratio*100:.1f}% ({int(inliers.sum())}/{len(inliers)})")
        if in_ratio < 0.5:
            log.warn("內點比例偏低：很可能 timebase 或 pose_convention / 外參方向有問題")

        # ---- refine ----
        use_yaw = bool(self.get_parameter("use_yaw_residual").value)
        in2 = [poses2_s[i] for i in np.where(inliers)[0]]
        in3 = [poses3_s[i] for i in np.where(inliers)[0]]

        R2r, t2r, s_ref = MapAlignment.refine_se2(
            in2, in3,
            R2_init=R2, t2_init=t2v, s_init=s_est,
            estimate_scale=estimate_scale,
            use_yaw=use_yaw,
            loss="huber",
            w_xy=1.0,
            w_yaw=0.3,
        )

        # ---- build final 4x4 T(map2d -> map3d) ----
        # 我們估到的是： Q ≈ s * R2 * P + t2
        # 這對應 map3d(xy) -> map2d(xy)
        # 但 TF 需要 map_2d -> map_3d，方向相反
        # 所以先組 T_2d_from_3d，再 invert
        yaw_align = float(np.arctan2(R2r[1, 0], R2r[0, 0]))
        T_2d_from_3d = make_T_se2(t2r[0], t2r[1], yaw_align, tz=0.0)

        # 若估 scale，這其實是 similarity，不是純 SE(2) 剛體
        # TF 無法表達 scale，因此：
        # - 若 estimate_scale=True，建議你先修正 3D 端尺度到 metric 再用 TF
        # - 這裡仍會輸出「忽略 scale 的剛體 TF」，並在 report 中提示
        if estimate_scale and abs(s_ref - 1.0) > 1e-3:
            log.warn(f"estimate_scale=True 且估到 scale={s_ref:.4f}；TF 不支援 scale，輸出的 TF 會忽略 scale。")

        # z offset：用 inliers 的 median(z2 - z3) 作為 map 間高度差
        if bool(self.get_parameter("estimate_z_offset").value):
            z2 = np.array([T[2, 3] for T in in2], dtype=float) if in2 else np.array([0.0])
            z3 = np.array([T[2, 3] for T in in3], dtype=float) if in3 else np.array([0.0])
            z_off = float(np.median(z2 - z3))
            T_2d_from_3d[2, 3] = z_off
        else:
            z_off = 0.0

        T_3d_from_2d = np.linalg.inv(T_2d_from_3d)  # map_2d -> map_3d

        # ---- error report (use rigid part) ----
        err = AlignmentError.compute(R2r, t2r, 1.0, in2, in3, use_yaw=use_yaw)
        log.info(AlignmentError.report(err))

        # ---- publish TF ----
        if bool(self.get_parameter("publish_tf").value):
            map2 = self.get_parameter("map_2d_frame").value
            map3 = self.get_parameter("map_3d_frame").value
            msg = matrix_to_transform(T_3d_from_2d, map2, map3, self.get_clock().now().to_msg())
            self.static_broadcaster.sendTransform(msg)
            log.info(f"已發布 /tf_static: {map2} -> {map3}")

        # ---- save result ----
        self.save_result(T_3d_from_2d, err, in_ratio, s_ref, z_off)

        log.info("完成。你可以直接在 TF tree 中使用 map_2d -> map_3d 來對齊。")

    def save_result(self, T_3d_from_2d: np.ndarray, err: Dict, inlier_ratio: float, scale_est: float, z_off: float):
        output_file = self.get_parameter("output_file").value
        map2 = self.get_parameter("map_2d_frame").value
        map3 = self.get_parameter("map_3d_frame").value

        q = rotation_matrix_to_quaternion(T_3d_from_2d[:3, :3])  # xyzw

        result = {
            "map_2d_frame": map2,
            "map_3d_frame": map3,
            "transformation": {
                "matrix": T_3d_from_2d.tolist(),
                "translation": T_3d_from_2d[:3, 3].tolist(),
                "rotation_quaternion_xyzw": q.tolist(),
                "z_offset_estimated": float(z_off),
            },
            "statistics": {
                "inlier_ratio": float(inlier_ratio),
                "error": err,
                "scale_estimated_if_enabled": float(scale_est),
            },
            "params": {
                "max_time_diff": float(self.get_parameter("max_time_diff").value),
                "time_scale_2d": float(self.get_parameter("time_scale_2d").value),
                "time_scale_3d": float(self.get_parameter("time_scale_3d").value),
                "enable_time_offset_search": bool(self.get_parameter("enable_time_offset_search").value),
                "estimate_scale": bool(self.get_parameter("estimate_scale").value),
                "use_yaw_residual": bool(self.get_parameter("use_yaw_residual").value),
                "inlier_threshold_xy": float(self.get_parameter("inlier_threshold_xy").value),
                "ransac_iterations": int(self.get_parameter("ransac_iterations").value),
            }
        }

        with open(output_file, "w") as f:
            yaml.safe_dump(result, f, sort_keys=False)
        self.get_logger().info(f"結果已儲存: {output_file}")

        # 同時生成 static_transform_publisher 指令
        cmd_file = output_file.replace(".yaml", "_command.sh")
        cmd = f"""#!/bin/bash
# Auto generated static TF command
ros2 run tf2_ros static_transform_publisher \\
  {T_3d_from_2d[0,3]:.6f} {T_3d_from_2d[1,3]:.6f} {T_3d_from_2d[2,3]:.6f} \\
  {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} \\
  {result['map_2d_frame']} \\
  {result['map_3d_frame']}
"""
        with open(cmd_file, "w") as f:
            f.write(cmd)
        os.chmod(cmd_file, 0o755)
        self.get_logger().info(f"TF 指令已儲存: {cmd_file}")


# =============================================================================
# main
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MapAlignmentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()