"""Orthographic, fixed-bounds Agg contact sheet of measured skinned NPZ meshes.

Use --case LABEL=PATH once per row. Inputs are immutable replay_body_skin.py
outputs. The renderer does not infer animation or skinning quality.
"""
from __future__ import annotations
import argparse
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "Tools" / "cache" / "matplotlib"))
sys.path.insert(0, str(ROOT / "Tools" / "pylibs"))
import numpy as np
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def load_mesh(path):
    triangles = []
    with np.load(path, allow_pickle=False) as arrays:
        prefixes = sorted(k[:-10] for k in arrays.files if k.startswith("primitive_") and k.endswith("_positions"))
        if not prefixes:
            raise ValueError(f"no mesh primitives: {path}")
        for prefix in prefixes:
            xyz = arrays[prefix + "_positions"].astype(float)
            indices = arrays[prefix + "_indices"].reshape(-1, 3)
            if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all():
                raise ValueError(f"invalid vertices: {path}/{prefix}")
            if indices.min() < 0 or indices.max() >= len(xyz):
                raise ValueError(f"invalid triangle indices: {path}/{prefix}")
            # GLTF(X,Y,Z) -> display/CP(X,-Z,Y); determinant +1.
            display = xyz[:, [0, 2, 1]].copy()
            display[:, 1] *= -1
            triangles.append(display[indices])
    return np.concatenate(triangles)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", action="append", required=True, help="LABEL=NPZ path, ordered rows")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--z-min", type=float, help="optional shared lower display-Z crop (metres)")
    args = parser.parse_args()
    cases = []
    for case in args.case:
        label, path = case.split("=", 1)
        cases.append((label, Path(path), load_mesh(Path(path))))
    all_points = np.concatenate([mesh.reshape(-1, 3) for _, _, mesh in cases])
    lo, hi = all_points.min(axis=0), all_points.max(axis=0)
    if args.z_min is not None:
        if not np.isfinite(args.z_min) or args.z_min >= hi[2]:
            parser.error("z-min must be finite and below the mesh maximum Z")
        lo[2] = args.z_min
    extent = np.maximum(hi - lo, 0.001)
    lo -= extent * 0.04
    hi += extent * 0.04
    fig = plt.figure(figsize=(12, 4 * len(cases)), dpi=160, facecolor="#ecebea")
    views = [("Front (+Y)", 90), ("Back (-Y)", -90), ("Side (+X)", 0)]
    for row, (label, path, triangles) in enumerate(cases):
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        for col, (view, azimuth) in enumerate(views):
            ax = fig.add_subplot(len(cases), 3, row * 3 + col + 1, projection="3d")
            ax.set_facecolor("#ecebea")
            light = np.array([np.cos(np.deg2rad(azimuth)), np.sin(np.deg2rad(azimuth)), 0.8])
            light /= np.linalg.norm(light)
            shade = 0.42 + 0.58 * np.maximum(0, normals @ light)
            colors = np.column_stack((shade * 0.73, shade * 0.66, shade * 0.58, np.ones(len(shade))))
            ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, edgecolors="none", linewidths=0))
            ax.set_proj_type("ortho")
            ax.view_init(elev=0, azim=azimuth)
            ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
            ax.set_box_aspect(hi - lo)
            ax.set_axis_off()
            ax.set_title(f"{label} | {view}", fontsize=10)
    fig.suptitle("Offline body skin comparison | shared bounds, orthographic views\nInputs may include experimental poses; no retail validation", fontsize=11)
    fig.subplots_adjust(left=0, right=1, bottom=0.01, top=0.86, wspace=0, hspace=0.08)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"{args.out}: {len(cases)} rows x3 views; shared bounds {lo.tolist()} .. {hi.tolist()}")


if __name__ == "__main__":
    main()
