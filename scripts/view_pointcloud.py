#!/usr/bin/env python3
"""PCD (ascii) point cloud viewer using matplotlib.

Usage:
    python3 view_pointcloud.py <cloud.pcd> [--max-points N] [--save out.png]
"""

import argparse
import sys

import numpy as np


def load_pcd(path):
    """Load an ascii PCD file -> (N,4) array [x y z intensity]."""
    with open(path, "r") as f:
        for i, line in enumerate(f):
            if line.strip().lower() == "data ascii":
                data_start = i + 1
                break
        else:
            raise ValueError("no 'DATA ascii' header found")
    return np.loadtxt(path, skiprows=data_start)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd")
    ap.add_argument("--max-points", type=int, default=120000,
                    help="subsample to at most this many points for display")
    ap.add_argument("--save", default=None, help="save figure to PNG instead of showing")
    ap.add_argument("--color", default="z", choices=["z", "intensity"],
                    help="color points by height or reflectivity")
    args = ap.parse_args()

    pts = load_pcd(args.pcd)
    if pts.ndim == 1:
        pts = pts.reshape(1, -1)
    print(f"loaded {len(pts)} points from {args.pcd}")
    print(f"  x: [{pts[:,0].min():.2f}, {pts[:,0].max():.2f}] m")
    print(f"  y: [{pts[:,1].min():.2f}, {pts[:,1].max():.2f}] m")
    print(f"  z: [{pts[:,2].min():.2f}, {pts[:,2].max():.2f}] m")

    if len(pts) > args.max_points:
        idx = np.random.default_rng(0).choice(len(pts), args.max_points,
                                              replace=False)
        pts = pts[idx]
        print(f"  (displaying {args.max_points} subsampled points)")

    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 9))
    ax = fig.add_subplot(111, projection="3d")
    c = pts[:, 2] if args.color == "z" else pts[:, 3]
    sc = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=c, s=0.3,
                    cmap="viridis" if args.color == "z" else "gray",
                    linewidths=0)
    fig.colorbar(sc, ax=ax, label="z [m]" if args.color == "z" else "intensity",
                 shrink=0.6)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title(args.pcd)

    # equal-ish aspect: use the largest xy range
    xr = pts[:, 0].max() - pts[:, 0].min()
    yr = pts[:, 1].max() - pts[:, 1].min()
    r = max(xr, yr, 1e-3)
    ax.set_xlim(pts[:, 0].mean() - r / 2, pts[:, 0].mean() + r / 2)
    ax.set_ylim(pts[:, 1].mean() - r / 2, pts[:, 1].mean() + r / 2)

    if args.save:
        fig.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"saved -> {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    sys.exit(main())
