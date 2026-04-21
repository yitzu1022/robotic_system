#!/usr/bin/env python3
import json
import sys
import shutil
from pathlib import Path

FILEPATH = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/home/weichen/robotic-project/robot_ws/data/lab/accumulated_gaussians_instance_semantic_info.json")
BACKUP = FILEPATH.with_suffix(FILEPATH.suffix + ".order_based.bak")

# Keep positions are 1-based indices of occurrences within `instances` for each semantic class
KEEP_POSITIONS = {
    "cabinet": [1, 3, 10, 11, 19],
    "shelf": [4],
    "table": [1, 10, 15, 29],
    "chair": [4, 5, 17, 19],
    "sofa": [5, 7, 8, 31, 32, 37],
    "refrigerator": [5, 8]
}

TARGET_CLASSES = set(KEEP_POSITIONS.keys())


def load(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def save(path: Path, data):
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def main():
    data = load(FILEPATH)
    instances = data.get("instances", {})
    classes = data.get("classes", {})

    # Gather occurrences per class in the order they appear in the instances dict
    occurrences = {c: [] for c in TARGET_CLASSES}
    for key, inst in instances.items():
        if not isinstance(inst, dict):
            continue
        cname = inst.get("semantic_name")
        iid = inst.get("instance_id")
        if cname in TARGET_CLASSES and isinstance(iid, int):
            occurrences[cname].append((key, iid))

    # Determine kept instance ids according to requested 1-based positions
    kept_by_class = {}
    kept_ids = set()
    for cname, pos_list in KEEP_POSITIONS.items():
        occ = occurrences.get(cname, [])
        kept = []
        for p in pos_list:
            if 1 <= p <= len(occ):
                kept.append(occ[p - 1][1])
            else:
                print(f"Warning: {cname} requested position {p} out of range (1..{len(occ)})")
        kept = sorted(set(kept))
        kept_by_class[cname] = kept
        for _id in kept:
            kept_ids.add(_id)

    # Backup original file
    shutil.copy2(FILEPATH, BACKUP)

    before = len(instances)
    removed = []

    # Remove instances for target classes that are NOT in kept_ids
    for key in list(instances.keys()):
        inst = instances.get(key) or {}
        cname = inst.get("semantic_name")
        iid = inst.get("instance_id")
        if cname in TARGET_CLASSES:
            if not isinstance(iid, int) or iid not in kept_ids:
                removed.append((key, iid, cname))
                del instances[key]

    data["instances"] = instances

    # Update classes' local/global mappings and histograms to keep only kept_ids
    for cls_key, cls in classes.items():
        cname = cls.get("class_name")
        if cname not in TARGET_CLASSES:
            continue

        old_local_hist = cls.get("cluster_histogram_local", {})
        old_l2g = cls.get("local_to_global_instance_id", {})
        old_global = cls.get("cluster_histogram_global", {})

        new_l2g = {}
        new_local_hist = {}
        # Keep '-1' (noise) if present
        if "-1" in old_l2g:
            new_l2g["-1"] = old_l2g["-1"]
            if "-1" in old_local_hist:
                new_local_hist["-1"] = old_local_hist["-1"]

        for lk, gv in old_l2g.items():
            if lk == "-1":
                continue
            try:
                gid = int(gv)
            except Exception:
                continue
            if gid in kept_ids:
                new_l2g[lk] = gv
                if lk in old_local_hist:
                    new_local_hist[lk] = old_local_hist[lk]

        new_global = {}
        for gk, cnt in old_global.items():
            try:
                gid = int(gk)
            except Exception:
                continue
            if gid in kept_ids:
                new_global[str(gid)] = cnt

        cls["local_to_global_instance_id"] = new_l2g
        cls["cluster_histogram_local"] = new_local_hist
        cls["cluster_histogram_global"] = new_global

    # Save updated JSON
    save(FILEPATH, data)

    after = len(data.get("instances", {}))

    # Print summary
    print(f"Instances before: {before}, after: {after}, removed: {len(removed)}")
    if removed:
        by_class = {}
        for k, iid, cname in removed:
            by_class.setdefault(cname, []).append(iid)
        for cname, lst in by_class.items():
            print(f"- {cname}: removed {len(lst)} instances -> ids: {sorted([x for x in lst if x is not None])}")

    print("Kept instance ids per target class:")
    for cname in sorted(TARGET_CLASSES):
        ids = kept_by_class.get(cname, [])
        print(f"- {cname}: kept {len(ids)} -> {ids}")


if __name__ == '__main__':
    main()
