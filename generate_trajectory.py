import pandas as pd
import numpy as np
from scipy.interpolate import make_interp_spline, CubicSpline

import matplotlib.pyplot as plt

if __name__ == "__main__":
    trajectory = np.zeros((100, 3))

    for i in range(len(trajectory)):
        trajectory[i, 0] = np.sin(2 * i * 0.1 * 2 * np.pi)
        trajectory[i, 1] = np.cos(i * 0.1 * 2 * np.pi)
        trajectory[i, 2] = 1.0 + i * 0.01

    df = pd.DataFrame(trajectory, columns=["x", "y", "z"])
    df.to_csv("trajectory.csv", index=False)

    df = pd.read_csv("trajectory.csv")

    t = np.arange(len(df))
    t_fine = np.linspace(t[0], t[-1], 1000)

    spline_x = CubicSpline(t, df["x"].values)
    spline_y = CubicSpline(t, df["y"].values)
    spline_z = CubicSpline(t, df["z"].values)

    x_interp = spline_x(t_fine)
    y_interp = spline_y(t_fine)
    z_interp = spline_z(t_fine)

    print(spline_x.c[:, 1])
    # print(spline_x.k)

    df_interp = pd.DataFrame({"x": x_interp, "y": y_interp, "z": z_interp})
    df_interp.to_csv("trajectory_interpolated.csv", index=False)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(df["x"], df["y"], df["z"], 'o', label="Original", markersize=3)
    ax.plot(x_interp, y_interp, z_interp, label="Interpolated")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.show()

