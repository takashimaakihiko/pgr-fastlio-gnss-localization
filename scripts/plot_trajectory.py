#!/usr/bin/env python3
"""Plot 2D trajectory from trajectory.csv produced by lio_node.

Usage:
    python3 plot_trajectory.py [csv_path]

    csv_path defaults to ../data/trajectory/trajectory.csv relative to cwd.
"""

import csv
import math
import sys

import matplotlib.pyplot as plt


def main() -> None:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "../data/trajectory/trajectory.csv"

    xs, ys, thetas, stamps = [], [], [], []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            stamps.append(float(row["timestamp"]))
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            thetas.append(float(row["yaw"]))

    if not xs:
        print(f"no data in {csv_path}")
        sys.exit(1)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(xs, ys, "-", linewidth=1.5, label="trajectory")
    ax.plot(xs[0], ys[0], "go", markersize=10, label="start")
    ax.plot(xs[-1], ys[-1], "rs", markersize=10, label="end")

    # heading arrows every ~50 samples
    step = max(1, len(xs) // 50)
    ax.quiver(xs[::step], ys[::step],
              [0.3 * math.cos(t) for t in thetas[::step]],
              [0.3 * math.sin(t) for t in thetas[::step]],
              angles="xy", scale_units="xy", scale=1, width=0.004, alpha=0.5)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Trajectory ({len(xs)} poses, {stamps[-1] - stamps[0]:.1f} s)")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
