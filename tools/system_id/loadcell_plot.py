import numpy as np
import csv

import matplotlib.pyplot as plt

data = {"t": [], "readings": [], "readings_raw": []}

with open("loadcell_data.csv", "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        data["t"].append(float(row["t"]))
        # readings: [fx, fy, fz, tx, ty, tz]
        readings = [
            float(row["fx"]),
            float(row["fy"]),
            float(row["fz"]),
            float(row["tx"]),
            float(row["ty"]),
            float(row["tz"]),
        ]
        # readings_raw: [fx_raw, fy_raw, fz_raw, tx_raw, ty_raw, tz_raw]
        readings_raw = [
            float(row["fx_raw"]),
            float(row["fy_raw"]),
            float(row["fz_raw"]),
            float(row["tx_raw"]),
            float(row["ty_raw"]),
            float(row["tz_raw"]),
        ]
        data["readings"].append(readings)
        data["readings_raw"].append(readings_raw)

data["t"] = np.array(data["t"])
data["readings"] = np.array(data["readings"])
data["readings_raw"] = np.array(data["readings_raw"])

fig, axs = plt.subplots(3, 2, figsize=(18, 8))
labels = ["X", "Y", "Z"]

# Compute noise level (standard deviation of raw force and torque signals)
force_noise = np.std(data["readings_raw"][:, 0])
torque_noise = np.std(data["readings_raw"][:, 3])

# Force and Raw Force (readings 0:3 and readings_raw 0:3)
for i in range(3):
    ax = axs[i, 0]
    ax.plot(data["t"], data["readings"][:, i], label=f"Force {labels[i]}")
    ax.set_title(f"Force {labels[i]}")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Force Value")
    ax2 = ax.twinx()
    ax2.plot(
        data["t"],
        data["readings_raw"][:, i],
        ":",
        label=f"Raw Force {labels[i]}",
        color="tab:orange",
    )
    ax2.set_ylabel("Raw Force Value")
    ax.legend(loc="upper left")
    ax2.legend(loc="upper right")

# Torque and Raw Torque (readings 3:6 and readings_raw 3:6)
for i in range(3):
    ax = axs[i, 1]
    ax.plot(data["t"], data["readings"][:, i + 3], label=f"Torque {labels[i]}")
    ax.set_title(f"Torque {labels[i]}")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Torque Value")
    ax2 = ax.twinx()
    ax2.plot(
        data["t"],
        data["readings_raw"][:, i + 3],
        ":",
        label=f"Raw Torque {labels[i]}",
        color="tab:orange",
    )
    ax2.set_ylabel("Raw Torque Value")
    ax.legend(loc="upper left")
    ax2.legend(loc="upper right")

fig.suptitle(
    f"Loadcell Data\nStd of noise on raw readings: Fx={force_noise:.4f}, Tx={torque_noise:.4f}",
    fontsize=16,
)
plt.tight_layout()  # rect=[0, 0.03, 1, 0.95]
plt.show()
