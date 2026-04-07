import json
import argparse
from typing import Dict, Any, List

import pyrealsense2 as rs


def intrinsics_to_dict(intr) -> Dict[str, Any]:
    return {
        "width": intr.width,
        "height": intr.height,
        "fx": intr.fx,
        "fy": intr.fy,
        "ppx": intr.ppx,
        "ppy": intr.ppy,
        "model": int(intr.model),
        "coeffs": list(intr.coeffs),
    }


def extrinsics_to_dict(extr) -> Dict[str, Any]:
    return {
        "rotation": list(extr.rotation),      # row-major 3x3, length=9
        "translation": list(extr.translation) # [tx, ty, tz] in meters
    }


def reshape_rotation(r9: List[float]) -> List[List[float]]:
    return [
        [r9[0], r9[1], r9[2]],
        [r9[3], r9[4], r9[5]],
        [r9[6], r9[7], r9[8]],
    ]


def matmul3x3(a: List[float], b: List[float]) -> List[float]:
    out = [0.0] * 9
    for i in range(3):
        for j in range(3):
            out[i * 3 + j] = (
                a[i * 3 + 0] * b[0 * 3 + j] +
                a[i * 3 + 1] * b[1 * 3 + j] +
                a[i * 3 + 2] * b[2 * 3 + j]
            )
    return out


def matvec3(a: List[float], v: List[float]) -> List[float]:
    return [
        a[0] * v[0] + a[1] * v[1] + a[2] * v[2],
        a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
        a[6] * v[0] + a[7] * v[1] + a[8] * v[2],
    ]


def transpose3x3(a: List[float]) -> List[float]:
    return [
        a[0], a[3], a[6],
        a[1], a[4], a[7],
        a[2], a[5], a[8],
    ]


def compose_extrinsics(extr_ab: Dict[str, Any], extr_bc: Dict[str, Any]) -> Dict[str, Any]:
    """
    Compose A->B and B->C to get A->C

    p_B = R_ab * p_A + t_ab
    p_C = R_bc * p_B + t_bc

    => p_C = (R_bc * R_ab) p_A + (R_bc * t_ab + t_bc)
    """
    R_ab = extr_ab["rotation"]
    t_ab = extr_ab["translation"]
    R_bc = extr_bc["rotation"]
    t_bc = extr_bc["translation"]

    R_ac = matmul3x3(R_bc, R_ab)
    Rt = matvec3(R_bc, t_ab)
    t_ac = [Rt[0] + t_bc[0], Rt[1] + t_bc[1], Rt[2] + t_bc[2]]

    return {
        "rotation": R_ac,
        "translation": t_ac
    }


def invert_extrinsics(extr_ab: Dict[str, Any]) -> Dict[str, Any]:
    """
    Invert A->B to get B->A

    p_B = R p_A + t
    => p_A = R^T p_B - R^T t
    """
    R = extr_ab["rotation"]
    t = extr_ab["translation"]

    R_inv = transpose3x3(R)
    Rt = matvec3(R_inv, t)
    t_inv = [-Rt[0], -Rt[1], -Rt[2]]

    return {
        "rotation": R_inv,
        "translation": t_inv
    }


def pretty_extr(extr: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "rotation_row_major": extr["rotation"],
        "rotation_matrix": reshape_rotation(extr["rotation"]),
        "translation_m": extr["translation"]
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--output", type=str, default="realsense_ir2rgb_params.json")
    args = parser.parse_args()

    pipeline = rs.pipeline()
    config = rs.config()

    # Enable the streams we need
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.infrared, 1, args.width, args.height, rs.format.y8, args.fps)
    config.enable_stream(rs.stream.infrared, 2, args.width, args.height, rs.format.y8, args.fps)

    profile = pipeline.start(config)

    try:
        # Wait a few frames so the pipeline is stable
        for _ in range(5):
            pipeline.wait_for_frames()

        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        ir_left_profile = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        ir_right_profile = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()

        depth_intr = depth_profile.get_intrinsics()
        color_intr = color_profile.get_intrinsics()
        ir_left_intr = ir_left_profile.get_intrinsics()
        ir_right_intr = ir_right_profile.get_intrinsics()

        # Direct extrinsics from SDK
        extr_depth_to_color = depth_profile.get_extrinsics_to(color_profile)
        extr_depth_to_ir_left = depth_profile.get_extrinsics_to(ir_left_profile)
        extr_depth_to_ir_right = depth_profile.get_extrinsics_to(ir_right_profile)

        extr_ir_left_to_color = ir_left_profile.get_extrinsics_to(color_profile)
        extr_ir_right_to_color = ir_right_profile.get_extrinsics_to(color_profile)

        # Convert to plain dicts
        d2c = extrinsics_to_dict(extr_depth_to_color)
        d2l = extrinsics_to_dict(extr_depth_to_ir_left)
        d2r = extrinsics_to_dict(extr_depth_to_ir_right)
        l2c_direct = extrinsics_to_dict(extr_ir_left_to_color)
        r2c_direct = extrinsics_to_dict(extr_ir_right_to_color)

        # Also derive ir_right -> color from:
        # ir_right -> depth, then depth -> color
        r2d = invert_extrinsics(d2r)
        r2c_derived = compose_extrinsics(r2d, d2c)

        # And derive ir_left(depth optical frame) -> color
        l2d = invert_extrinsics(d2l)
        l2c_derived = compose_extrinsics(l2d, d2c)

        result = {
            "meta": {
                "width": args.width,
                "height": args.height,
                "fps": args.fps,
                "depth_scale": depth_scale
            },
            "intrinsics": {
                "depth_intr": intrinsics_to_dict(depth_intr),
                "ir_left_intr": intrinsics_to_dict(ir_left_intr),
                "ir_right_intr": intrinsics_to_dict(ir_right_intr),
                "color_intr": intrinsics_to_dict(color_intr)
            },
            "extrinsics_direct_from_sdk": {
                "depth_to_color": pretty_extr(d2c),
                "depth_to_ir_left": pretty_extr(d2l),
                "depth_to_ir_right": pretty_extr(d2r),
                "ir_left_to_color": pretty_extr(l2c_direct),
                "ir_right_to_color": pretty_extr(r2c_direct)
            },
            "extrinsics_derived": {
                "ir_right_to_depth": pretty_extr(r2d),
                "ir_right_to_color": pretty_extr(r2c_derived),
                "ir_left_to_depth": pretty_extr(l2d),
                "ir_left_to_color": pretty_extr(l2c_derived)
            }
        }

        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)

        print("=" * 80)
        print("Saved parameters to:", args.output)
        print("=" * 80)

        print("\n[IR-right -> RGB] direct from SDK")
        print(json.dumps(pretty_extr(r2c_direct), indent=2))

        print("\n[IR-right -> RGB] derived from (IR-right -> depth) + (depth -> color)")
        print(json.dumps(pretty_extr(r2c_derived), indent=2))

        print("\n[IR-left -> RGB] direct from SDK")
        print(json.dumps(pretty_extr(l2c_direct), indent=2))

        print("\n[IR-left -> RGB] derived from (IR-left -> depth) + (depth -> color)")
        print(json.dumps(pretty_extr(l2c_derived), indent=2))

    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()