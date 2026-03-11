#!/usr/bin/env python3
"""
Convert PLY (either per-vertex RGB or spherical-harmonic coeffs) to NPZ

This version supports PLYs that store `red`,`green`,`blue` (u1) and an
optional `label` (u2) field. It will by default map colors (RGB normalized
to [0,1]) to categories using the provided `actual_color_mapping.json`.

Usage example:
  python3 convert_sem_gs_rgb_ply.py --input_ply /path/to/file.ply \
      --output_npz /path/to/out.npz --mapping /path/to/actual_color_mapping.json

"""

import argparse
import json
import sys
import numpy as np
from plyfile import PlyData


def convert_with_rgb_or_sh(ply_path, output_path, mapping_path,
                           use_ply_label=False, label_map_path=None,
                           tol=0.001, batch_size=100000):
    print("=" * 70)
    print(f"📂 讀取 PLY: {ply_path}")
    print("=" * 70)

    ply = PlyData.read(ply_path)
    vertex = ply['vertex'].data
    num_points = len(vertex)
    print(f"✅ 讀取頂點數: {num_points:,}")

    # XYZ
    pts = np.vstack((vertex['x'], vertex['y'], vertex['z'])).T.astype(np.float32)
    print(f"✅ 提取 XYZ: {pts.shape}")

    names = vertex.dtype.names

    # 支援兩種來源：直接 RGB (u1) 或球諧係數 f_dc_0/1/2
    if {'red', 'green', 'blue'}.issubset(names):
        rgb = np.vstack((vertex['red'], vertex['green'], vertex['blue'])).T.astype(np.float32) / 255.0
        print(f"✅ 使用頂點 RGB (uint8)，shape={rgb.shape}")
    elif {'f_dc_0', 'f_dc_1', 'f_dc_2'}.issubset(names):
        sh_dc = np.vstack((vertex['f_dc_0'], vertex['f_dc_1'], vertex['f_dc_2'])).T.astype(np.float32)
        C0 = 0.28209479177387814
        rgb = np.clip(sh_dc * C0 + 0.5, 0.0, 1.0)
        print(f"✅ 使用球諧係數轉 RGB，shape={rgb.shape}")
    else:
        raise RuntimeError('PLY 中缺少 RGB 或 f_dc_* 欄位，無法處理')

    # 讀取顏色映射
    print(f"\n📂 讀取顏色映射: {mapping_path}")
    with open(mapping_path, 'r') as f:
        mapping_data = json.load(f)

    color_mappings = mapping_data.get('color_mapping', [])
    if len(color_mappings) == 0:
        raise RuntimeError('mapping file 中找不到 color_mapping')

    mapping_rgbs = np.array([m['rgb'] for m in color_mappings], dtype=np.float32)
    id_to_name = {i: m['category'] for i, m in enumerate(color_mappings)}

    # 優先使用 PLY 提供的 label（若使用者要求並且欄位存在）
    if use_ply_label and 'label' in names:
        labels = vertex['label'].astype(np.int32)
        unique_labels = np.unique(labels)
        label_to_idx = {lab: i for i, lab in enumerate(unique_labels)}
        semantic_ids = np.vectorize(label_to_idx.get)(labels)
        # 建立 id->name，如果提供 label_map 則使用之
        if label_map_path:
            with open(label_map_path, 'r') as f:
                label_map = json.load(f)
            # label_map 預期為 {"1": "wall", "2": "floor", ...}
            id_to_name = {i: label_map.get(str(lab), f'label_{lab}') for lab, i in label_to_idx.items()}
        else:
            id_to_name = {i: f'label_{lab}' for lab, i in label_to_idx.items()}
        print(f"✅ 使用 PLY `label` 欄位：mapped {len(unique_labels)} 類別")
        matched_count = int((semantic_ids >= 0).sum())

    else:
        # 使用最近鄰顏色匹配
        semantic_ids = np.zeros(num_points, dtype=np.int32)
        matched_count = 0
        print(f"\n🎨 開始顏色匹配（容差 {tol}），分批大小 {batch_size} ...")

        for bs in range(0, num_points, batch_size):
            be = min(bs + batch_size, num_points)
            batch_rgb = rgb[bs:be]
            # distances: (batch, n_mappings)
            diff = batch_rgb[:, None, :] - mapping_rgbs[None, :, :]
            distances = np.sqrt(np.sum(diff * diff, axis=2))
            closest = np.argmin(distances, axis=1)
            min_dist = np.min(distances, axis=1)
            semantic_ids[bs:be] = closest
            matched_count += np.sum(min_dist < tol)
            if be % (max(1, batch_size)) == 0 or be == num_points:
                print(f"   處理: {be}/{num_points} ({100.0 * be / num_points:.1f}%)")

        print(f"   ✅ 完成匹配，精確匹配數: {matched_count:,} / {num_points:,}")

    # 類別分佈
    id_counts = np.bincount(semantic_ids)
    print(f"\n✅ 語義分類統計:")
    for idx, cnt in enumerate(id_counts):
        if cnt > 0:
            name = id_to_name.get(idx, 'unknown')
            print(f"   ID {idx:3d} ({name:15s}): {cnt:,} pts ({100.0 * cnt / num_points:.2f}%)")

    # 儲存 NPZ
    save_dict = {
        'means3D': pts,
        'pts': pts,
        'pan': semantic_ids,
        'semantic_ids': semantic_ids,
        'rgb': rgb
    }

    np.savez_compressed(output_path, **save_dict)
    print(f"\n✅ 已保存 NPZ: {output_path}")

    # 儲存元資料
    json_output = output_path.replace('.npz', '_meta.json')
    segments_info = []
    for idx, name in id_to_name.items():
        count = int(id_counts[idx]) if idx < len(id_counts) else 0
        if count > 0:
            segments_info.append({
                'id': int(idx),
                'category_name': name,
                'class': name,
                'point_count': int(count)
            })

    with open(json_output, 'w') as f:
        json.dump({'segments_info': segments_info}, f, indent=2)

    print(f"✅ 已保存語義元資料: {json_output}")
    print("\n" + "=" * 70)
    print("✅ 轉換完成")
    print("=" * 70)
    return True


def main():
    parser = argparse.ArgumentParser(description='Convert PLY (RGB/label or SH) to NPZ using exact color mapping')
    parser.add_argument('--input_ply', '-i', required=True, help='輸入 PLY 文件')
    parser.add_argument('--output_npz', '-o', required=True, help='輸出 NPZ 文件')
    parser.add_argument('--mapping', '-m', default='actual_color_mapping.json', help='顏色映射 JSON')
    parser.add_argument('--use_ply_label', action='store_true', help='若 PLY 含 label 欄位，直接使用 label 作為語義 ID')
    parser.add_argument('--label_map', help='可選的 label -> 名稱 映射 JSON（例如 {"1":"wall"}）')
    parser.add_argument('--tol', type=float, default=0.001, help='匹配容差（歐式距離）')
    parser.add_argument('--batch', type=int, default=100000, help='批次大小')
    args = parser.parse_args()

    ok = convert_with_rgb_or_sh(args.input_ply, args.output_npz, args.mapping,
                                use_ply_label=args.use_ply_label, label_map_path=args.label_map,
                                tol=args.tol, batch_size=args.batch)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
