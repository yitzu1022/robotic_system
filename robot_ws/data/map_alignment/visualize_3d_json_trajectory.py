#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple


Vec3 = Tuple[float, float, float]
Mat3 = List[List[float]]


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> Mat3:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def load_trajectory(path: Path) -> List[Dict[str, Any]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict):
        for key in ("trajectory", "poses", "frames", "camera_trajectory"):
            if isinstance(data.get(key), list):
                data = data[key]
                break
    if not isinstance(data, list):
        raise ValueError("JSON must be a list, or a dict containing trajectory/poses/frames/camera_trajectory")
    return data


def pose_from_item(item: Dict[str, Any]) -> Tuple[Vec3, Mat3]:
    if "transform_matrix" in item:
        T = item["transform_matrix"]
        if len(T) != 4 or any(len(row) != 4 for row in T):
            raise ValueError("transform_matrix must be 4x4")
        position = (float(T[0][3]), float(T[1][3]), float(T[2][3]))
        rotation = [[float(T[r][c]) for c in range(3)] for r in range(3)]
        return position, rotation

    required = ("tx", "ty", "tz", "qx", "qy", "qz", "qw")
    missing = [k for k in required if k not in item]
    if missing:
        raise ValueError(f"trajectory item missing fields: {missing}")
    position = (float(item["tx"]), float(item["ty"]), float(item["tz"]))
    rotation = quat_to_rot(float(item["qx"]), float(item["qy"]), float(item["qz"]), float(item["qw"]))
    return position, rotation


def mat_col(R: Mat3, col: int) -> Vec3:
    return (R[0][col], R[1][col], R[2][col])


def scale_vec(v: Vec3, scale: float) -> Vec3:
    return (v[0] * scale, v[1] * scale, v[2] * scale)


def build_view_data(items: List[Dict[str, Any]], stride: int, axis_length: float) -> Dict[str, Any]:
    poses = [pose_from_item(item) for item in items]
    positions = [p for p, _ in poses]
    axes = []
    for idx in range(0, len(poses), max(1, stride)):
        p, R = poses[idx]
        axes.append(
            {
                "p": p,
                "x": scale_vec(mat_col(R, 0), axis_length),
                "y": scale_vec(mat_col(R, 1), axis_length),
                "z": scale_vec(mat_col(R, 2), axis_length),
            }
        )
    return {"positions": positions, "axes": axes, "count": len(positions)}


def html_template(data: Dict[str, Any]) -> str:
    payload = json.dumps(data, ensure_ascii=False)
    return f"""<!doctype html>
<html lang=\"en\">
<head>
<meta charset=\"utf-8\" />
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
<title>3D Trajectory Viewer</title>
<style>
  html, body {{ margin: 0; height: 100%; background: #101214; color: #eef2f5; font-family: Arial, sans-serif; }}
  #wrap {{ position: fixed; inset: 0; display: grid; grid-template-rows: auto 1fr; }}
  #bar {{ padding: 10px 14px; background: #181c20; border-bottom: 1px solid #2b3138; display: flex; gap: 18px; align-items: center; flex-wrap: wrap; }}
  #bar span {{ color: #b9c2ca; }}
  canvas {{ width: 100%; height: 100%; display: block; cursor: grab; }}
  canvas:active {{ cursor: grabbing; }}
  .key {{ display: inline-flex; align-items: center; gap: 6px; }}
  .swatch {{ width: 10px; height: 10px; border-radius: 50%; display: inline-block; }}
</style>
</head>
<body>
<div id=\"wrap\">
  <div id=\"bar\">
    <strong>3D Camera Trajectory</strong>
    <span id=\"info\"></span>
    <span>drag: rotate</span>
    <span>wheel: zoom</span>
    <span class=\"key\"><i class=\"swatch\" style=\"background:#4ade80\"></i>start</span>
    <span class=\"key\"><i class=\"swatch\" style=\"background:#fb7185\"></i>end</span>
    <span class=\"key\"><i class=\"swatch\" style=\"background:#ef4444\"></i>cam x</span>
    <span class=\"key\"><i class=\"swatch\" style=\"background:#22c55e\"></i>cam y</span>
    <span class=\"key\"><i class=\"swatch\" style=\"background:#3b82f6\"></i>cam z</span>
  </div>
  <canvas id=\"view\"></canvas>
</div>
<script>
const DATA = {payload};
const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
document.getElementById('info').textContent = `${{DATA.count}} poses`;
let yaw = -0.85;
let pitch = 0.45;
let zoom = 1.0;
let dragging = false;
let lastX = 0;
let lastY = 0;

const pts = DATA.positions.map(p => [Number(p[0]), Number(p[1]), Number(p[2])]);
const center = [0, 0, 0];
const minv = [Infinity, Infinity, Infinity];
const maxv = [-Infinity, -Infinity, -Infinity];
for (const p of pts) {{
  for (let i = 0; i < 3; i++) {{ minv[i] = Math.min(minv[i], p[i]); maxv[i] = Math.max(maxv[i], p[i]); }}
}}
for (let i = 0; i < 3; i++) center[i] = (minv[i] + maxv[i]) / 2;
const extent = Math.max(maxv[0] - minv[0], maxv[1] - minv[1], maxv[2] - minv[2], 1e-6);

function resize() {{
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(canvas.clientWidth * dpr);
  canvas.height = Math.floor(canvas.clientHeight * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}}

function rotatePoint(p) {{
  const x0 = p[0] - center[0];
  const y0 = p[1] - center[1];
  const z0 = p[2] - center[2];
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const x1 = cy * x0 + sy * z0;
  const z1 = -sy * x0 + cy * z0;
  const y1 = cp * y0 - sp * z1;
  const z2 = sp * y0 + cp * z1;
  return [x1, y1, z2];
}}

function project(p) {{
  const r = rotatePoint(p);
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  const scale = Math.min(w, h) * 0.72 * zoom / extent;
  const depth = 4.0 * extent;
  const persp = depth / (depth + r[2]);
  return [w / 2 + r[0] * scale * persp, h / 2 - r[1] * scale * persp, r[2]];
}}

function drawLine(a, b, color, width = 1) {{
  const pa = project(a);
  const pb = project(b);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(pa[0], pa[1]);
  ctx.lineTo(pb[0], pb[1]);
  ctx.stroke();
}}

function drawDot(p, radius, color) {{
  const pp = project(p);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(pp[0], pp[1], radius, 0, Math.PI * 2);
  ctx.fill();
}}

function add(p, v) {{ return [p[0] + v[0], p[1] + v[1], p[2] + v[2]]; }}

function drawGrid() {{
  const z = minv[2];
  ctx.globalAlpha = 0.35;
  for (let i = 0; i <= 10; i++) {{
    const t = i / 10;
    const x = minv[0] + (maxv[0] - minv[0]) * t;
    const y = minv[1] + (maxv[1] - minv[1]) * t;
    drawLine([x, minv[1], z], [x, maxv[1], z], '#47515d', 0.7);
    drawLine([minv[0], y, z], [maxv[0], y, z], '#47515d', 0.7);
  }}
  ctx.globalAlpha = 1;
}}

function draw() {{
  ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  ctx.fillStyle = '#101214';
  ctx.fillRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  drawGrid();

  for (let i = 1; i < pts.length; i++) drawLine(pts[i - 1], pts[i], '#e5e7eb', 1.4);
  for (let i = 0; i < pts.length; i++) drawDot(pts[i], 2.2, `hsl(${{210 + 120 * i / Math.max(1, pts.length - 1)}}, 80%, 58%)`);

  for (const axis of DATA.axes) {{
    const p = axis.p;
    drawLine(p, add(p, axis.x), '#ef4444', 1.6);
    drawLine(p, add(p, axis.y), '#22c55e', 1.6);
    drawLine(p, add(p, axis.z), '#3b82f6', 1.6);
  }}

  drawDot(pts[0], 6, '#4ade80');
  drawDot(pts[pts.length - 1], 6, '#fb7185');
}}

canvas.addEventListener('mousedown', e => {{ dragging = true; lastX = e.clientX; lastY = e.clientY; }});
window.addEventListener('mouseup', () => {{ dragging = false; }});
window.addEventListener('mousemove', e => {{
  if (!dragging) return;
  yaw += (e.clientX - lastX) * 0.008;
  pitch += (e.clientY - lastY) * 0.008;
  pitch = Math.max(-1.45, Math.min(1.45, pitch));
  lastX = e.clientX;
  lastY = e.clientY;
  draw();
}});
canvas.addEventListener('wheel', e => {{
  e.preventDefault();
  zoom *= Math.exp(-e.deltaY * 0.001);
  zoom = Math.max(0.2, Math.min(10, zoom));
  draw();
}}, {{ passive: false }});
window.addEventListener('resize', resize);
resize();
</script>
</body>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a self-contained interactive 3D trajectory HTML from JSON.")
    parser.add_argument("--input", "-i", default="robot_deploy_slam_example0_camera_trajectory.json")
    parser.add_argument("--output", "-o", default="alignment_out/trajectory_3d.html")
    parser.add_argument("--stride", type=int, default=20, help="draw one camera orientation every N poses")
    parser.add_argument("--axis-length", type=float, default=0.08, help="length of camera orientation axes")
    args = parser.parse_args()

    items = load_trajectory(Path(args.input))
    if not items:
        raise ValueError("trajectory JSON is empty")

    data = build_view_data(items, stride=max(1, args.stride), axis_length=float(args.axis_length))
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html_template(data), encoding="utf-8")

    print(f"Loaded: {args.input}")
    print(f"Poses: {data['count']}")
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
