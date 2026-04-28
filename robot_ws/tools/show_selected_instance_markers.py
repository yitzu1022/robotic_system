#!/usr/bin/python3
from __future__ import annotations

import argparse
import json
import math
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import DefaultDict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


Color = Tuple[float, float, float, float]
Selection = Tuple[str, int]
Instance = Tuple[str, Tuple[float, float, float]]


@dataclass(frozen=True)
class AlignmentTransform:
    mu: np.ndarray
    e1: np.ndarray
    e2: np.ndarray
    normal: np.ndarray
    scale: float
    rot: np.ndarray
    trans: np.ndarray


@dataclass(frozen=True)
class GoalTarget:
    name: str
    index: int
    instance_id: str
    raw_xyz: Tuple[float, float, float]
    map_xy: Tuple[float, float]
    map_z: float


@dataclass(frozen=True)
class RuntimeGoal:
    event: str
    label: str
    action: str
    map_x: float
    map_y: float
    map_z: float
    for_grasp: bool


@dataclass(frozen=True)
class CandidateOption:
    index: int
    instance_id: str
    map_x: float
    map_y: float
    map_z: float


@dataclass(frozen=True)
class CandidateSelection:
    name: str
    options: List[CandidateOption]


def load_instances(instance_path: Path) -> DefaultDict[str, List[Instance]]:
    if not instance_path.exists():
        raise FileNotFoundError(f"Instance JSON not found: {instance_path}")

    with instance_path.open("r", encoding="utf-8") as file_obj:
        data = json.load(file_obj)

    instances = data.get("instances", {})
    if not isinstance(instances, dict) or not instances:
        raise ValueError(f'No valid "instances" section found in {instance_path}')

    objects: DefaultDict[str, List[Instance]] = defaultdict(list)
    for inst_id, inst in instances.items():
        name = str(inst.get("semantic_name", "")).strip().lower()
        centroid = inst.get("centroid")
        if not name or not isinstance(centroid, list) or len(centroid) < 3:
            continue
        objects[name].append((inst_id, (float(centroid[0]), float(centroid[1]), float(centroid[2]))))
    return objects


def resolve_selections(
    objects: DefaultDict[str, List[Instance]], selections: Sequence[Selection]
) -> List[Tuple[str, int, str, Tuple[float, float, float]]]:
    resolved: List[Tuple[str, int, str, Tuple[float, float, float]]] = []
    for name, index in selections:
        instances = objects.get(name, [])
        if not instances:
            available = ", ".join(sorted(objects.keys()))
            raise ValueError(f'Object "{name}" not found. Available labels include: {available}')
        if index < 0 or index >= len(instances):
            raise IndexError(
                f'Index {index} is out of range for "{name}". '
                f"Valid range: 0..{len(instances) - 1}"
            )
        inst_id, position = instances[index]
        resolved.append((name, index, inst_id, position))
    return resolved


def load_map3d_to_map2d(yaml_path: Path) -> AlignmentTransform:
    with yaml_path.open("r", encoding="utf-8") as file_obj:
        data = yaml.safe_load(file_obj)

    return AlignmentTransform(
        mu=np.array(
            [
                data["plane_fit"]["mu"]["x"],
                data["plane_fit"]["mu"]["y"],
                data["plane_fit"]["mu"]["z"],
            ],
            dtype=float,
        ),
        e1=np.array(data["plane_fit"]["basis_e1"], dtype=float),
        e2=np.array(data["plane_fit"]["basis_e2"], dtype=float),
        normal=np.array(data["plane_fit"]["normal_n"], dtype=float),
        scale=float(data["sim2"]["s"]),
        rot=np.array(data["sim2"]["R"], dtype=float),
        trans=np.array([data["sim2"]["t"]["x"], data["sim2"]["t"]["y"]], dtype=float),
    )


def map3d_point_to_map_frame(
    p_xyz: Tuple[float, float, float],
    alignment: AlignmentTransform,
) -> Tuple[float, float, float]:
    point = np.array(p_xyz, dtype=float)
    delta = point - alignment.mu
    uv = np.array([delta @ alignment.e1, delta @ alignment.e2], dtype=float)
    height = float(delta @ alignment.normal)
    xy = alignment.scale * (alignment.rot @ uv) + alignment.trans
    z = alignment.scale * height
    return float(xy[0]), float(xy[1]), float(z)


def map3d_point_to_map2d_xy(
    p_xyz: Tuple[float, float, float],
    alignment: AlignmentTransform,
) -> Tuple[float, float]:
    x, y, _ = map3d_point_to_map_frame(p_xyz, alignment)
    return x, y


def build_goal_targets(
    resolved: Sequence[Tuple[str, int, str, Tuple[float, float, float]]],
    alignment: AlignmentTransform,
) -> List[GoalTarget]:
    goals: List[GoalTarget] = []
    for name, index, inst_id, raw_xyz in resolved:
        map_x, map_y, map_z = map3d_point_to_map_frame(raw_xyz, alignment)
        goals.append(
            GoalTarget(
                name=name,
                index=index,
                instance_id=inst_id,
                raw_xyz=raw_xyz,
                map_xy=(map_x, map_y),
                map_z=map_z,
            )
        )
    return goals


def load_point_cloud_npz(pointcloud_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    if not pointcloud_path.exists():
        raise FileNotFoundError(f"Point cloud NPZ not found: {pointcloud_path}")

    data = np.load(pointcloud_path)

    points = None
    for key in ("points", "pts", "means3D", "xyz"):
        if key in data:
            points = np.asarray(data[key], dtype=np.float32)
            break
    if points is None or points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"Could not find Nx3 point data in {pointcloud_path}")

    colors = None
    for key in ("colors", "rgb", "color", "rgb_colors"):
        if key in data:
            raw_colors = np.asarray(data[key])
            if raw_colors.ndim == 2 and raw_colors.shape[1] >= 3:
                colors = raw_colors[:, :3]
                break
    if colors is None:
        colors = np.full((points.shape[0], 3), 180, dtype=np.uint8)
    elif colors.dtype != np.uint8:
        if colors.max() <= 1.0:
            colors = np.clip(colors * 255.0, 0.0, 255.0).astype(np.uint8)
        else:
            colors = np.clip(colors, 0.0, 255.0).astype(np.uint8)

    if colors.shape[0] != points.shape[0]:
        raise ValueError(
            f"Point/color count mismatch in {pointcloud_path}: "
            f"{points.shape[0]} points vs {colors.shape[0]} colors"
        )
    return points[:, :3], colors[:, :3]


def transform_points_to_map_frame(
    points_xyz: np.ndarray,
    alignment: AlignmentTransform,
    z_offset: float = 0.0,
) -> np.ndarray:
    delta = points_xyz.astype(np.float64) - alignment.mu[None, :]
    u = delta @ alignment.e1
    v = delta @ alignment.e2
    h = delta @ alignment.normal
    uv = np.stack((u, v), axis=1)
    xy = alignment.scale * (uv @ alignment.rot.T) + alignment.trans[None, :]
    z = (alignment.scale * h) + z_offset
    return np.column_stack((xy, z)).astype(np.float32)


def filter_pointcloud_by_z(
    points_xyz: np.ndarray,
    colors_rgb: np.ndarray,
    min_z: Optional[float],
    max_z: Optional[float],
) -> Tuple[np.ndarray, np.ndarray]:
    mask = np.ones(points_xyz.shape[0], dtype=bool)
    if min_z is not None:
        mask &= points_xyz[:, 2] >= min_z
    if max_z is not None:
        mask &= points_xyz[:, 2] <= max_z
    return points_xyz[mask], colors_rgb[mask]


def build_pointcloud2_message(
    frame_id: str,
    points_xyz: np.ndarray,
    colors_rgb: np.ndarray,
    stamp,
) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.height = 1
    msg.width = int(points_xyz.shape[0])
    msg.is_bigendian = False
    msg.is_dense = True
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width

    packed = np.empty((points_xyz.shape[0], 4), dtype=np.float32)
    packed[:, :3] = points_xyz
    rgb_uint32 = (
        (colors_rgb[:, 0].astype(np.uint32) << 16)
        | (colors_rgb[:, 1].astype(np.uint32) << 8)
        | colors_rgb[:, 2].astype(np.uint32)
    )
    packed[:, 3] = rgb_uint32.view(np.float32)
    msg.data = packed.tobytes()
    return msg


def parse_selection(raw: str) -> Selection:
    if ":" not in raw:
        raise argparse.ArgumentTypeError(f'Invalid selection "{raw}". Use object:index, e.g. table:2')
    name, index_str = raw.split(":", 1)
    name = name.strip().lower()
    if not name:
        raise argparse.ArgumentTypeError(f'Invalid selection "{raw}": missing object name')
    try:
        index = int(index_str)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f'Invalid selection "{raw}": index must be an integer'
        ) from exc
    return name, index


def clone_pose(msg: PoseStamped, frame_id: str) -> PoseStamped:
    pose = PoseStamped()
    pose.header.stamp = msg.header.stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = msg.pose.position.x
    pose.pose.position.y = msg.pose.position.y
    pose.pose.position.z = msg.pose.position.z
    pose.pose.orientation.x = msg.pose.orientation.x
    pose.pose.orientation.y = msg.pose.orientation.y
    pose.pose.orientation.z = msg.pose.orientation.z
    pose.pose.orientation.w = msg.pose.orientation.w
    return pose


def xy_distance(pose: PoseStamped, target_xy: Tuple[float, float]) -> float:
    return math.hypot(pose.pose.position.x - target_xy[0], pose.pose.position.y - target_xy[1])


class SelectedInstanceMarkerPublisher(Node):
    def __init__(
        self,
        instance_path: Path,
        alignment_path: Path,
        pointcloud_path: Path,
        selections: Sequence[Selection],
        marker_topic: str,
        path_topic: str,
        pointcloud_topic: str,
        runtime_goal_topic: str,
        candidate_topic: str,
        frame_id: str,
        pose_topic: str,
        tf_base_frame: str,
        rate_hz: float,
        goal_tolerance: float,
        advance_distance: float,
        path_min_step: float,
        pose_timeout_sec: float,
        max_path_points: int,
        pointcloud_stride: int,
        pointcloud_refresh_sec: float,
        pointcloud_z_offset: float,
        pointcloud_min_z: Optional[float],
        pointcloud_max_z: Optional[float],
        disable_tf_fallback: bool,
    ) -> None:
        super().__init__("selected_instance_marker_publisher")
        self._frame_id = frame_id
        self._goal_tolerance = goal_tolerance
        self._advance_distance = advance_distance
        self._path_min_step = path_min_step
        self._pose_timeout_sec = pose_timeout_sec
        self._max_path_points = max_path_points
        self._disable_tf_fallback = disable_tf_fallback
        self._tf_base_frame = tf_base_frame
        self._pose_topic = pose_topic
        self._runtime_goal_topic = runtime_goal_topic
        self._candidate_topic = candidate_topic
        self._pointcloud_msg: Optional[PointCloud2] = None
        self._runtime_goal: Optional[RuntimeGoal] = None
        self._candidate_selection: Optional[CandidateSelection] = None

        alignment = load_map3d_to_map2d(alignment_path)
        if selections:
            objects = load_instances(instance_path)
            resolved = resolve_selections(objects, list(selections))
            self._goals = build_goal_targets(resolved, alignment)
        else:
            self._goals = []

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, qos)
        self._path_pub = self.create_publisher(NavPath, path_topic, qos)
        self._pointcloud_pub = self.create_publisher(PointCloud2, pointcloud_topic, qos)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose_msg, 10)
        if runtime_goal_topic:
            self.create_subscription(String, runtime_goal_topic, self._on_runtime_goal_msg, 10)
        if candidate_topic:
            self.create_subscription(String, candidate_topic, self._on_candidate_msg, 10)
        self.create_timer(1.0 / rate_hz, self._on_timer)
        self._pointcloud_timer = None

        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        if not disable_tf_fallback:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        self._path_msg = NavPath()
        self._path_msg.header.frame_id = frame_id
        self._robot_pose: Optional[PoseStamped] = None
        self._last_pose_wall_time = 0.0
        self._warned_pose_frame = False
        self._active_goal_idx = 0
        self._current_goal_reached = False

        cloud_points, cloud_colors = load_point_cloud_npz(pointcloud_path)
        if pointcloud_stride > 1:
            cloud_points = cloud_points[::pointcloud_stride]
            cloud_colors = cloud_colors[::pointcloud_stride]
        transformed_points = transform_points_to_map_frame(
            cloud_points,
            alignment,
            z_offset=pointcloud_z_offset,
        )
        original_point_count = transformed_points.shape[0]
        transformed_points, cloud_colors = filter_pointcloud_by_z(
            transformed_points,
            cloud_colors,
            min_z=pointcloud_min_z,
            max_z=pointcloud_max_z,
        )
        self._pointcloud_msg = build_pointcloud2_message(
            frame_id,
            transformed_points,
            cloud_colors,
            self.get_clock().now().to_msg(),
        )
        if pointcloud_refresh_sec > 0.0:
            self._pointcloud_timer = self.create_timer(pointcloud_refresh_sec, self.publish_pointcloud)

        for goal in self._goals:
            raw_x, raw_y, raw_z = goal.raw_xyz
            map_x, map_y = goal.map_xy
            map_z = goal.map_z
            self.get_logger().info(
                f"{goal.name}:{goal.index} (instance_id={goal.instance_id}) "
                f"raw=({raw_x:.3f}, {raw_y:.3f}, {raw_z:.3f}) "
                f"-> map=({map_x:.3f}, {map_y:.3f}, {map_z:.3f})"
            )

        self.get_logger().info(
            f"Publishing RViz markers to {marker_topic}, path to {path_topic}, "
            f"pose source topic={pose_topic}, runtime goal topic={runtime_goal_topic or 'off'}, "
            f"candidate topic={candidate_topic or 'off'}, "
            f"tf fallback={'off' if disable_tf_fallback else tf_base_frame}"
        )
        self.get_logger().info(
            f"Publishing transformed point cloud to {pointcloud_topic} "
            f"({transformed_points.shape[0]} points after stride={pointcloud_stride}, "
            f"z filter min={pointcloud_min_z}, max={pointcloud_max_z}; "
            f"source points={original_point_count})"
        )
        if transformed_points.shape[0] == 0:
            self.get_logger().warn("Point cloud Z filter removed all points. Relax --pointcloud-min-z/--pointcloud-max-z.")
        self.publish_pointcloud()

    def _on_pose_msg(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self._frame_id and not self._warned_pose_frame:
            self._warned_pose_frame = True
            self.get_logger().warn(
                f"Pose topic frame is '{msg.header.frame_id}', but marker frame is '{self._frame_id}'. "
                "Overlay assumes both are already in the same 2D map frame."
            )
        pose = clone_pose(msg, self._frame_id)
        self._update_robot_pose(pose)

    def _on_runtime_goal_msg(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse runtime goal JSON: {exc}")
            return

        event = str(payload.get("event", "")).strip().lower()
        if event == "reset_path":
            self._reset_path_history()
            return
        if event == "clear":
            self._runtime_goal = None
            return

        try:
            self._runtime_goal = RuntimeGoal(
                event=event or "active",
                label=str(payload.get("label", "nav_goal")),
                action=str(payload.get("action", "")),
                map_x=float(payload["x"]),
                map_y=float(payload["y"]),
                map_z=float(payload.get("z", 0.0)),
                for_grasp=bool(payload.get("for_grasp", False)),
            )
        except Exception as exc:
            self.get_logger().warn(f"Invalid runtime goal payload: {exc}")

    def _on_candidate_msg(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse candidate JSON: {exc}")
            return

        event = str(payload.get("event", "")).strip().lower()
        if event == "clear":
            self._candidate_selection = None
            return

        raw_options = payload.get("options", [])
        if not isinstance(raw_options, list):
            self.get_logger().warn("Invalid candidate payload: options must be a list")
            return

        try:
            options = [
                CandidateOption(
                    index=int(item["index"]),
                    instance_id=str(item.get("instance_id", "")),
                    map_x=float(item["map_x"]),
                    map_y=float(item["map_y"]),
                    map_z=float(item.get("map_z", 0.0)),
                )
                for item in raw_options
            ]
        except Exception as exc:
            self.get_logger().warn(f"Invalid candidate option payload: {exc}")
            return

        self._candidate_selection = CandidateSelection(
            name=str(payload.get("name", "object")),
            options=options,
        )

    def _update_robot_pose(self, pose: PoseStamped) -> None:
        self._robot_pose = pose
        self._last_pose_wall_time = time.monotonic()
        self._append_path_pose(pose)
        self._update_goal_progress()

    def _reset_path_history(self) -> None:
        self._path_msg = NavPath()
        self._path_msg.header.frame_id = self._frame_id
        if self._robot_pose is not None:
            self._path_msg.header.stamp = self._robot_pose.header.stamp
            self._path_msg.poses.append(clone_pose(self._robot_pose, self._frame_id))

    def _append_path_pose(self, pose: PoseStamped) -> None:
        if self._path_msg.poses:
            last = self._path_msg.poses[-1]
            dist = math.hypot(
                pose.pose.position.x - last.pose.position.x,
                pose.pose.position.y - last.pose.position.y,
            )
            if dist < self._path_min_step:
                return

        self._path_msg.header.stamp = pose.header.stamp
        self._path_msg.poses.append(clone_pose(pose, self._frame_id))
        if len(self._path_msg.poses) > self._max_path_points:
            self._path_msg.poses.pop(0)

    def _update_goal_progress(self) -> None:
        if self._runtime_goal is not None:
            return
        if self._robot_pose is None or not self._goals:
            return

        goal = self._goals[self._active_goal_idx]
        distance = xy_distance(self._robot_pose, goal.map_xy)

        if not self._current_goal_reached and distance <= self._goal_tolerance:
            self._current_goal_reached = True
            self.get_logger().info(
                f"Reached goal {goal.name}:{goal.index} within {distance:.2f} m. "
                "Holding this goal until the robot leaves the area."
            )
            return

        has_next_goal = self._active_goal_idx + 1 < len(self._goals)
        if self._current_goal_reached and has_next_goal and distance >= self._advance_distance:
            self._active_goal_idx += 1
            self._current_goal_reached = False
            next_goal = self._goals[self._active_goal_idx]
            self.get_logger().info(
                f"Robot moved away from previous goal. Advancing active goal -> "
                f"{next_goal.name}:{next_goal.index}"
            )

    def _maybe_refresh_pose_from_tf(self) -> None:
        if self._disable_tf_fallback or self._tf_buffer is None:
            return

        if self._robot_pose is not None and (time.monotonic() - self._last_pose_wall_time) < self._pose_timeout_sec:
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame_id,
                self._tf_base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self._update_robot_pose(pose)

    def _make_goal_marker(
        self,
        marker_id: int,
        now,
        goal: GoalTarget,
        color: Color,
        label_prefix: str,
        scale: float,
    ) -> List[Marker]:
        goal_x, goal_y = goal.map_xy

        sphere = Marker()
        sphere.header.frame_id = self._frame_id
        sphere.header.stamp = now
        sphere.ns = "goal_points"
        sphere.id = marker_id
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = goal_x
        sphere.pose.position.y = goal_y
        sphere.pose.position.z = max(goal.map_z, 0.02)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = scale
        sphere.scale.y = scale
        sphere.scale.z = scale
        sphere.color.r = color[0]
        sphere.color.g = color[1]
        sphere.color.b = color[2]
        sphere.color.a = color[3]
        sphere.lifetime.sec = 0

        text = Marker()
        text.header.frame_id = self._frame_id
        text.header.stamp = now
        text.ns = "goal_labels"
        text.id = marker_id + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = goal_x
        text.pose.position.y = goal_y
        text.pose.position.z = max(goal.map_z + 0.30, 0.35)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.22
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"{label_prefix} {goal.name} {goal.index}"
        text.lifetime.sec = 0
        return [sphere, text]

    def publish_markers(self) -> None:
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        delete_marker = Marker()
        delete_marker.header.frame_id = self._frame_id
        delete_marker.header.stamp = now
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        marker_id = 0
        done_color: Color = (0.15, 0.80, 0.35, 0.95)
        active_color: Color = (0.95, 0.30, 0.20, 1.0)
        waiting_color: Color = (0.95, 0.70, 0.20, 1.0)
        future_color: Color = (0.45, 0.60, 0.90, 0.70)
        ref_color: Color = (0.70, 0.70, 0.70, 0.35)
        display_goal_xy: Optional[Tuple[float, float]] = None
        display_goal_z = 0.05

        for idx, goal in enumerate(self._goals):
            if self._runtime_goal is not None:
                label = "REF"
                color = ref_color
                scale = 0.20
            else:
                if idx < self._active_goal_idx:
                    label = "DONE"
                    color = done_color
                    scale = 0.24
                elif idx == self._active_goal_idx and self._current_goal_reached:
                    label = "WAIT"
                    color = waiting_color
                    scale = 0.34
                    display_goal_xy = goal.map_xy
                    display_goal_z = goal.map_z
                elif idx == self._active_goal_idx:
                    label = "GOAL"
                    color = active_color
                    scale = 0.36
                    display_goal_xy = goal.map_xy
                    display_goal_z = goal.map_z
                else:
                    label = "NEXT"
                    color = future_color
                    scale = 0.24

            markers = self._make_goal_marker(marker_id, now, goal, color, label, scale)
            marker_array.markers.extend(markers)
            marker_id += 2

        if self._runtime_goal is not None:
            if self._runtime_goal.event == "arrived":
                runtime_color: Color = waiting_color
                runtime_prefix = ""
                runtime_scale = 0.36
            else:
                runtime_color = active_color
                runtime_prefix = ""
                runtime_scale = 0.38

            runtime_sphere = Marker()
            runtime_sphere.header.frame_id = self._frame_id
            runtime_sphere.header.stamp = now
            runtime_sphere.ns = "runtime_goal_points"
            runtime_sphere.id = marker_id
            marker_id += 1
            runtime_sphere.type = Marker.SPHERE
            runtime_sphere.action = Marker.ADD
            runtime_sphere.pose.position.x = self._runtime_goal.map_x
            runtime_sphere.pose.position.y = self._runtime_goal.map_y
            runtime_sphere.pose.position.z = max(self._runtime_goal.map_z, 0.02)
            runtime_sphere.pose.orientation.w = 1.0
            runtime_sphere.scale.x = runtime_scale
            runtime_sphere.scale.y = runtime_scale
            runtime_sphere.scale.z = runtime_scale
            runtime_sphere.color.r = runtime_color[0]
            runtime_sphere.color.g = runtime_color[1]
            runtime_sphere.color.b = runtime_color[2]
            runtime_sphere.color.a = runtime_color[3]
            runtime_sphere.lifetime.sec = 0
            marker_array.markers.append(runtime_sphere)

            runtime_text = Marker()
            runtime_text.header.frame_id = self._frame_id
            runtime_text.header.stamp = now
            runtime_text.ns = "runtime_goal_labels"
            runtime_text.id = marker_id
            marker_id += 1
            runtime_text.type = Marker.TEXT_VIEW_FACING
            runtime_text.action = Marker.ADD
            runtime_text.pose.position.x = self._runtime_goal.map_x
            runtime_text.pose.position.y = self._runtime_goal.map_y
            runtime_text.pose.position.z = max(self._runtime_goal.map_z + 0.35, 0.40)
            runtime_text.pose.orientation.w = 1.0
            runtime_text.scale.z = 0.24
            runtime_text.color.r = 1.0
            runtime_text.color.g = 1.0
            runtime_text.color.b = 1.0
            runtime_text.color.a = 1.0
            runtime_text.text = f"{runtime_prefix} {self._runtime_goal.label}"
            runtime_text.lifetime.sec = 0
            marker_array.markers.append(runtime_text)

            display_goal_xy = (self._runtime_goal.map_x, self._runtime_goal.map_y)
            display_goal_z = self._runtime_goal.map_z

        if self._candidate_selection is not None:
            candidate_color: Color = (0.85, 0.15, 0.95, 0.95)
            for option in self._candidate_selection.options:
                candidate_sphere = Marker()
                candidate_sphere.header.frame_id = self._frame_id
                candidate_sphere.header.stamp = now
                candidate_sphere.ns = "candidate_points"
                candidate_sphere.id = marker_id
                marker_id += 1
                candidate_sphere.type = Marker.SPHERE
                candidate_sphere.action = Marker.ADD
                candidate_sphere.pose.position.x = option.map_x
                candidate_sphere.pose.position.y = option.map_y
                candidate_sphere.pose.position.z = max(option.map_z, 0.02)
                candidate_sphere.pose.orientation.w = 1.0
                candidate_sphere.scale.x = 0.26
                candidate_sphere.scale.y = 0.26
                candidate_sphere.scale.z = 0.26
                candidate_sphere.color.r = candidate_color[0]
                candidate_sphere.color.g = candidate_color[1]
                candidate_sphere.color.b = candidate_color[2]
                candidate_sphere.color.a = candidate_color[3]
                candidate_sphere.lifetime.sec = 0
                marker_array.markers.append(candidate_sphere)

                candidate_text = Marker()
                candidate_text.header.frame_id = self._frame_id
                candidate_text.header.stamp = now
                candidate_text.ns = "candidate_labels"
                candidate_text.id = marker_id
                marker_id += 1
                candidate_text.type = Marker.TEXT_VIEW_FACING
                candidate_text.action = Marker.ADD
                candidate_text.pose.position.x = option.map_x
                candidate_text.pose.position.y = option.map_y
                candidate_text.pose.position.z = max(option.map_z + 0.35, 0.40)
                candidate_text.pose.orientation.w = 1.0
                candidate_text.scale.z = 0.22
                candidate_text.color.r = 1.0
                candidate_text.color.g = 1.0
                candidate_text.color.b = 1.0
                candidate_text.color.a = 1.0
                candidate_text.text = f"{self._candidate_selection.name} {option.index}"
                candidate_text.lifetime.sec = 0
                marker_array.markers.append(candidate_text)

        if self._path_msg.poses:
            trail = Marker()
            trail.header.frame_id = self._frame_id
            trail.header.stamp = now
            trail.ns = "robot_trail"
            trail.id = marker_id
            marker_id += 1
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.pose.orientation.w = 1.0
            trail.scale.x = 0.07
            trail.color.r = 0.05
            trail.color.g = 0.95
            trail.color.b = 0.95
            trail.color.a = 0.95
            trail.lifetime.sec = 0
            for pose in self._path_msg.poses:
                trail.points.append(
                    Point(
                        x=pose.pose.position.x,
                        y=pose.pose.position.y,
                        z=0.03,
                    )
                )
            marker_array.markers.append(trail)

        if self._robot_pose is not None:
            robot_marker = Marker()
            robot_marker.header.frame_id = self._frame_id
            robot_marker.header.stamp = now
            robot_marker.ns = "robot_pose"
            robot_marker.id = marker_id
            marker_id += 1
            robot_marker.type = Marker.ARROW
            robot_marker.action = Marker.ADD
            robot_marker.pose = self._robot_pose.pose
            robot_marker.pose.position.z = 0.12
            robot_marker.scale.x = 0.65
            robot_marker.scale.y = 0.18
            robot_marker.scale.z = 0.18
            robot_marker.color.r = 0.10
            robot_marker.color.g = 0.35
            robot_marker.color.b = 0.95
            robot_marker.color.a = 1.0
            robot_marker.lifetime.sec = 0
            marker_array.markers.append(robot_marker)

        self._marker_pub.publish(marker_array)
        self._path_msg.header.stamp = now
        self._path_pub.publish(self._path_msg)

    def publish_pointcloud(self) -> None:
        if self._pointcloud_msg is None:
            return
        self._pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        self._pointcloud_pub.publish(self._pointcloud_msg)

    def _on_timer(self) -> None:
        self._maybe_refresh_pose_from_tf()
        self.publish_markers()


def build_arg_parser() -> argparse.ArgumentParser:
    workspace_root = Path(__file__).resolve().parents[1]
    default_instance_path = workspace_root / "data/lab/accumulated_gaussians_instance_semantic_info.json"
    default_alignment_path = workspace_root / "data/Util/alignment.yaml"
    default_pointcloud_path = workspace_root / "data/lab/accumulated_gaussians.npz"

    parser = argparse.ArgumentParser(
        description=(
            "Publish aligned semantic point cloud plus Kachaka pose, heading, current goal, and trajectory "
            "for RViz during execution."
        )
    )
    parser.add_argument(
        "selections",
        nargs="*",
        type=parse_selection,
        help='Optional reference selections in object:index form, e.g. "table:2" "chair:0"',
    )
    parser.add_argument(
        "--instance-path",
        default=str(default_instance_path),
        help="Path to accumulated_gaussians_instance_semantic_info.json",
    )
    parser.add_argument(
        "--alignment-path",
        default=str(default_alignment_path),
        help="Path to the 3D->2D alignment.yaml used to convert semantic centroids into Kachaka map coordinates",
    )
    parser.add_argument(
        "--topic",
        default="/semantic_map_markers_preview",
        help="MarkerArray topic for RViz",
    )
    parser.add_argument(
        "--path-topic",
        default="/semantic_map_preview_path",
        help="Path topic for RViz",
    )
    parser.add_argument(
        "--pointcloud-path",
        default=str(default_pointcloud_path),
        help="Path to the source 3D NPZ point cloud that will be transformed into the Kachaka map frame",
    )
    parser.add_argument(
        "--pointcloud-topic",
        default="/aligned_map_pointcloud_preview",
        help="PointCloud2 topic for the transformed map point cloud",
    )
    parser.add_argument(
        "--frame-id",
        default="map",
        help="Frame id used by the output markers and path",
    )
    parser.add_argument(
        "--pose-topic",
        default="/kachaka_pose",
        help="PoseStamped topic for the robot pose in the same 2D map frame",
    )
    parser.add_argument(
        "--runtime-goal-topic",
        default="/semantic_preview/current_goal",
        help="JSON String topic published by decision_maker for the currently active navigation goal",
    )
    parser.add_argument(
        "--candidate-topic",
        default="/semantic_preview/object_candidates",
        help="JSON String topic published by object_query_server for candidate instance options during selection",
    )
    parser.add_argument(
        "--tf-base-frame",
        default="base_link",
        help="TF child frame used when pose-topic is unavailable or stale",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="Publish rate in Hz",
    )
    parser.add_argument(
        "--goal-tolerance",
        type=float,
        default=0.60,
        help="Distance threshold in meters to consider the current goal reached",
    )
    parser.add_argument(
        "--advance-distance",
        type=float,
        default=1.00,
        help="After reaching a goal, keep it active until the robot is this far away, then advance to the next goal",
    )
    parser.add_argument(
        "--path-min-step",
        type=float,
        default=0.03,
        help="Minimum XY movement in meters before appending a new path point",
    )
    parser.add_argument(
        "--pose-timeout-sec",
        type=float,
        default=1.0,
        help="If no pose-topic update arrives within this time, try TF fallback",
    )
    parser.add_argument(
        "--max-path-points",
        type=int,
        default=6000,
        help="Maximum number of path points kept in memory",
    )
    parser.add_argument(
        "--pointcloud-stride",
        type=int,
        default=2,
        help="Publish every Nth point from the source cloud. Increase if RViz becomes heavy",
    )
    parser.add_argument(
        "--pointcloud-refresh-sec",
        type=float,
        default=3.0,
        help="Re-publish interval for the static transformed point cloud. Set 0 to publish only once",
    )
    parser.add_argument(
        "--pointcloud-z-offset",
        type=float,
        default=0.0,
        help="Optional extra Z offset applied after transforming the point cloud into map frame",
    )
    parser.add_argument(
        "--pointcloud-min-z",
        type=float,
        default=None,
        help="Keep only transformed point cloud points with z >= this value in map frame",
    )
    parser.add_argument(
        "--pointcloud-max-z",
        type=float,
        default=1.8,
        help="Keep only transformed point cloud points with z <= this value in map frame",
    )
    parser.add_argument(
        "--disable-tf-fallback",
        action="store_true",
        help="Disable TF fallback and rely only on --pose-topic",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print raw 3D centroids and transformed 2D map coordinates without creating a ROS node",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    instance_path = Path(args.instance_path)
    alignment_path = Path(args.alignment_path)
    pointcloud_path = Path(args.pointcloud_path)

    if args.rate <= 0.0:
        raise SystemExit("--rate must be greater than 0")
    if args.goal_tolerance <= 0.0:
        raise SystemExit("--goal-tolerance must be greater than 0")
    if args.advance_distance < args.goal_tolerance:
        raise SystemExit("--advance-distance must be >= --goal-tolerance")
    if args.path_min_step < 0.0:
        raise SystemExit("--path-min-step must be >= 0")
    if args.max_path_points <= 0:
        raise SystemExit("--max-path-points must be > 0")
    if args.pointcloud_stride <= 0:
        raise SystemExit("--pointcloud-stride must be > 0")
    if args.pointcloud_refresh_sec < 0.0:
        raise SystemExit("--pointcloud-refresh-sec must be >= 0")
    if (
        args.pointcloud_min_z is not None
        and args.pointcloud_max_z is not None
        and args.pointcloud_min_z > args.pointcloud_max_z
    ):
        raise SystemExit("--pointcloud-min-z must be <= --pointcloud-max-z")

    if args.selections:
        objects = load_instances(instance_path)
        resolved = resolve_selections(objects, args.selections)
        goals = build_goal_targets(resolved, load_map3d_to_map2d(alignment_path))
    else:
        goals = []

    if args.dry_run:
        if not goals:
            print("No selections provided for --dry-run.")
            return
        for goal in goals:
            raw_x, raw_y, raw_z = goal.raw_xyz
            map_x, map_y = goal.map_xy
            map_z = goal.map_z
            print(
                f"{goal.name}:{goal.index} instance_id={goal.instance_id} "
                f"raw=({raw_x:.6f}, {raw_y:.6f}, {raw_z:.6f}) "
                f"map=({map_x:.6f}, {map_y:.6f}, {map_z:.6f})"
            )
        return

    rclpy.init()
    node = None
    try:
        node = SelectedInstanceMarkerPublisher(
            instance_path=instance_path,
            alignment_path=alignment_path,
            pointcloud_path=pointcloud_path,
            selections=args.selections,
            marker_topic=args.topic,
            path_topic=args.path_topic,
            pointcloud_topic=args.pointcloud_topic,
            runtime_goal_topic=args.runtime_goal_topic,
            candidate_topic=args.candidate_topic,
            frame_id=args.frame_id,
            pose_topic=args.pose_topic,
            tf_base_frame=args.tf_base_frame,
            rate_hz=args.rate,
            goal_tolerance=args.goal_tolerance,
            advance_distance=args.advance_distance,
            path_min_step=args.path_min_step,
            pose_timeout_sec=args.pose_timeout_sec,
            max_path_points=args.max_path_points,
            pointcloud_stride=args.pointcloud_stride,
            pointcloud_refresh_sec=args.pointcloud_refresh_sec,
            pointcloud_z_offset=args.pointcloud_z_offset,
            pointcloud_min_z=args.pointcloud_min_z,
            pointcloud_max_z=args.pointcloud_max_z,
            disable_tf_fallback=args.disable_tf_fallback,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
