#!/usr/bin/env python3
"""
Compute exploration coverage % by comparing a run's saved map against a
ground-truth (fully explored) reference map.

Usage:
    python3 compute_coverage.py ground_truth.yaml run_map.yaml

Requires: numpy, Pillow, PyYAML
    pip install numpy Pillow PyYAML --break-system-packages
"""

import sys
import yaml
import numpy as np
from PIL import Image

FREE = 254
OCCUPIED = 0
UNKNOWN = 205


def load_map(yaml_path):
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    pgm_path = meta["image"]
    # image path in yaml is often relative to the yaml file's folder
    import os
    base_dir = os.path.dirname(os.path.abspath(yaml_path))
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(base_dir, pgm_path)

    img = Image.open(pgm_path)
    arr = np.array(img)

    resolution = meta["resolution"]  # meters/pixel
    origin = meta["origin"]          # [x, y, yaw]

    return arr, resolution, origin, meta


def align_and_crop(arr_ref, origin_ref, res_ref, arr_run, origin_run, res_run):
    """
    Align two occupancy grids using their map-frame origins so we compare
    the same physical area. Assumes resolution matches (should, since both
    SLAM sessions used the same `resolution: 0.05` setting) and no rotation
    (yaw ~ 0 for both, typical for slam_toolbox with default start pose).
    """
    if abs(res_ref - res_run) > 1e-6:
        raise ValueError(
            f"Resolution mismatch: ref={res_ref}, run={res_run}. "
            "Both maps must be generated with the same resolution."
        )
    res = res_ref

    # pixel (0,0) is top-left, but map (0,0) in meters maps to bottom-left
    # of the image at 'origin'. Convert origin to a pixel offset.
    h_ref, w_ref = arr_ref.shape
    h_run, w_run = arr_run.shape

    # world-space bounding box (in meters) for each map
    ref_x0, ref_y0 = origin_ref[0], origin_ref[1]
    ref_x1, ref_y1 = ref_x0 + w_ref * res, ref_y0 + h_ref * res

    run_x0, run_y0 = origin_run[0], origin_run[1]
    run_x1, run_y1 = run_x0 + w_run * res, run_y0 + h_run * res

    # common overlapping world-space bounding box
    x0, y0 = max(ref_x0, run_x0), max(ref_y0, run_y0)
    x1, y1 = min(ref_x1, run_x1), min(ref_y1, run_y1)

    if x1 <= x0 or y1 <= y0:
        raise ValueError("Maps do not overlap in world space — check origins/resolution.")

    def crop(arr, origin, h):
        # convert world box to pixel indices for this map
        px0 = int(round((x0 - origin[0]) / res))
        px1 = int(round((x1 - origin[0]) / res))
        # image row 0 = top = highest y in map frame (PGM convention flips y)
        py0 = int(round(h - (y1 - origin[1]) / res))
        py1 = int(round(h - (y0 - origin[1]) / res))
        return arr[py0:py1, px0:px1]

    ref_crop = crop(arr_ref, origin_ref, h_ref)
    run_crop = crop(arr_run, origin_run, h_run)

    # trim to identical shape in case of rounding differences
    h = min(ref_crop.shape[0], run_crop.shape[0])
    w = min(ref_crop.shape[1], run_crop.shape[1])
    return ref_crop[:h, :w], run_crop[:h, :w], res


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 compute_coverage.py ground_truth.yaml run_map.yaml")
        sys.exit(1)

    ref_yaml, run_yaml = sys.argv[1], sys.argv[2]

    ref_arr, ref_res, ref_origin, _ = load_map(ref_yaml)
    run_arr, run_res, run_origin, _ = load_map(run_yaml)

    ref_crop, run_crop, res = align_and_crop(
        ref_arr, ref_origin, ref_res, run_arr, run_origin, run_res
    )

    # "known" ground truth free space = the area we actually want to explore
    ref_free_mask = ref_crop == FREE
    ref_known_mask = (ref_crop == FREE) | (ref_crop == OCCUPIED)

    run_free_mask = run_crop == FREE
    run_known_mask = (run_crop == FREE) | (run_crop == OCCUPIED)

    total_ref_known_px = np.count_nonzero(ref_known_mask)
    total_ref_free_px = np.count_nonzero(ref_free_mask)

    overlap_known_px = np.count_nonzero(ref_known_mask & run_known_mask)
    overlap_free_px = np.count_nonzero(ref_free_mask & run_free_mask)

    cell_area = res * res  # m^2 per pixel

    print(f"Resolution:                 {res} m/pixel")
    print(f"Ground truth known area:    {total_ref_known_px * cell_area:.2f} m^2 "
          f"({total_ref_known_px} px)")
    print(f"Ground truth free area:     {total_ref_free_px * cell_area:.2f} m^2 "
          f"({total_ref_free_px} px)")
    print()
    print(f"Run known-area overlap:     {overlap_known_px * cell_area:.2f} m^2 "
          f"({overlap_known_px} px)")
    print(f"Run free-area overlap:      {overlap_free_px * cell_area:.2f} m^2 "
          f"({overlap_free_px} px)")
    print()
    print(f"Coverage % (known cells):   {100 * overlap_known_px / total_ref_known_px:.1f}%")
    print(f"Coverage % (free space):    {100 * overlap_free_px / total_ref_free_px:.1f}%")


if __name__ == "__main__":
    main()
