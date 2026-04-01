"""clean_map.py — Remove anti-aliasing / gray-pixel artifacts from an f110_gym map PNG.

Converts every pixel to strictly 0 (wall) or 255 (free) using a binary threshold,
then saves the cleaned image alongside the original.

Usage:
    python clean_map.py                          # processes icra2022.png in-place backup
    python clean_map.py --map icra2022.png       # explicit path
    python clean_map.py --thresh 200             # custom threshold (default: 230)
    python clean_map.py --inplace                # overwrite original (backup saved first)
"""

import argparse
import os
import shutil

import cv2
import numpy as np

# ── CLI ────────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Binarize f110_gym map PNG")
parser.add_argument("--map",     default="icra2022.png",  help="Map PNG path")
parser.add_argument("--thresh",  default=230, type=int,   help="Binary threshold 0-255 (default 230)")
parser.add_argument("--inplace", action="store_true",     help="Overwrite original (backup created)")
args = parser.parse_args()

map_path = args.map
if not os.path.exists(map_path):
    # try relative to this script's directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_path   = os.path.join(script_dir, args.map)

if not os.path.exists(map_path):
    raise FileNotFoundError(f"Map not found: {args.map}")

# ── Load ───────────────────────────────────────────────────────────────────────
gray = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
if gray is None:
    raise RuntimeError(f"cv2 could not read: {map_path}")

H, W = gray.shape
print(f"[INFO] Loaded: {map_path}  ({W}×{H} px)")

# ── Diagnostics — before ───────────────────────────────────────────────────────
unique_before = len(np.unique(gray))
gray_pixels   = int(np.sum((gray > 0) & (gray < 255)))
print(f"[INFO] Before: {unique_before} unique intensity values, "
      f"{gray_pixels} gray (non-binary) pixels "
      f"({100.0 * gray_pixels / (H * W):.2f}% of image)")

# ── Binarize ───────────────────────────────────────────────────────────────────
# Pixels >= thresh → 255 (free space)
# Pixels <  thresh → 0   (wall / obstacle)
_, binary = cv2.threshold(gray, args.thresh, 255, cv2.THRESH_BINARY)

# ── Diagnostics — after ────────────────────────────────────────────────────────
unique_after   = len(np.unique(binary))
gray_after     = int(np.sum((binary > 0) & (binary < 255)))
free_px        = int(np.sum(binary == 255))
wall_px        = int(np.sum(binary == 0))
print(f"[INFO] After:  {unique_after} unique values, "
      f"{gray_after} gray pixels remaining")
print(f"[INFO] Free (white): {free_px} px ({100.0*free_px/(H*W):.1f}%)  "
      f"Wall (black): {wall_px} px ({100.0*wall_px/(H*W):.1f}%)")

# ── Save ───────────────────────────────────────────────────────────────────────
base, ext = os.path.splitext(map_path)
if args.inplace:
    backup_path = base + "_original" + ext
    shutil.copy2(map_path, backup_path)
    print(f"[INFO] Backup saved: {backup_path}")
    out_path = map_path
else:
    out_path = base + "_clean" + ext

cv2.imwrite(out_path, binary)
print(f"[INFO] Cleaned map saved: {out_path}")
print(f"[HINT] To use in f110_gym, update your YAML 'image:' field to point to the new file.")
