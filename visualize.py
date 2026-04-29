#!/usr/bin/env python3
"""Visualize swarm trajectories from a CSV file using cubic spline interpolation."""

import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from bezier import CubicBezierSpline

TRAJECTORY_CSV = "balette_var1_I2V_Wan_FP8_step_distillation_video_seed6514_0.22_simulated_trajectories_final.csv"
SEGMENT_DURATION = 0.1   # seconds per waypoint interval
POINTS_PER_SEGMENT = 10  # resolution of the plotted curve


def interpolate_trajectory(
    waypoints_x: np.ndarray,
    waypoints_y: np.ndarray,
    waypoints_z: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fit Bezier spline through waypoints and sample a smooth curve."""
    waypoints = np.column_stack((waypoints_x, waypoints_y, waypoints_z))
    spline = CubicBezierSpline.from_waypoints(waypoints)

    n_segments = len(spline.beziers)
    t_fine = np.linspace(0, n_segments, n_segments * POINTS_PER_SEGMENT)
    points = np.array([spline.evaluate(t) for t in t_fine])
    return points[:, 0], points[:, 1], points[:, 2]


def main() -> None:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else TRAJECTORY_CSV
    df = pd.read_csv(csv_path)
    print(f"Loaded {csv_path}: {len(df)} waypoints")
    print(
        f"Drone 0 start: x={df['x0'].iloc[0]:.3f}, y={df['y0'].iloc[0]:.3f}, z={df['z0'].iloc[0]:.3f}"
    )

    # Detect number of drones from column names (x0, x1, ...)
    n_drones = sum(1 for col in df.columns if col.startswith("x"))

    colors = plt.cm.tab10.colors

    # Interpolate all drones
    all_xi, all_yi, all_zi = [], [], []
    for n in range(n_drones):
        xi, yi, zi = interpolate_trajectory(
            df[f"x{n}"].values, df[f"y{n}"].values, df[f"z{n}"].values
        )
        all_xi.append(xi)
        all_yi.append(yi)
        all_zi.append(zi)

    n_frames = len(all_xi[0])

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")

    # Static ghost paths
    for n in range(n_drones):
        ax.plot(all_xi[n], all_yi[n], all_zi[n],
                color=colors[n % len(colors)], linewidth=0.8, alpha=0.25)

    # Animated: trail lines and drone markers
    trails = [
        ax.plot([], [], [], color=colors[n % len(colors)], linewidth=1.5)[0]
        for n in range(n_drones)
    ]
    markers = [
        ax.plot([], [], [], "o", color=colors[n % len(colors)],
                markersize=8, label=f"Drone {n}")[0]
        for n in range(n_drones)
    ]

    # Fixed axis limits
    all_x = np.concatenate(all_xi)
    all_y = np.concatenate(all_yi)
    all_z = np.concatenate(all_zi)
    pad = 0.2
    ax.set_xlim(all_x.min() - pad, all_x.max() + pad)
    ax.set_ylim(all_y.min() - pad, all_y.max() + pad)
    ax.set_zlim(all_z.min() - pad, all_z.max() + pad)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(f"Swarm trajectories – {n_drones} drone(s)")
    ax.legend(ncols=2, loc="upper left", bbox_to_anchor=(1.05, 1), borderaxespad=0)

    TRAIL_LEN = POINTS_PER_SEGMENT * 3  # frames of visible trail

    

    def update(frame: int):
        for n in range(n_drones):
            start = max(0, frame - TRAIL_LEN)
            trails[n].set_data_3d(
                all_xi[n][start:frame + 1],
                all_yi[n][start:frame + 1],
                all_zi[n][start:frame + 1],
            )
            markers[n].set_data_3d(
                [all_xi[n][frame]],
                [all_yi[n][frame]],
                [all_zi[n][frame]],
            )
        return [*trails, *markers]

    from matplotlib.animation import FuncAnimation
    total_time_ms = n_frames * SEGMENT_DURATION * 1000 / POINTS_PER_SEGMENT
    interval_ms = total_time_ms / n_frames  # real-time playback
    anim = FuncAnimation(fig, update, frames=n_frames, interval=interval_ms, blit=False)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
