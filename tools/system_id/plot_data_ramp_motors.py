import numpy as np
import matplotlib.pyplot as plt
import argparse
from utils import loadFile

parser = argparse.ArgumentParser()
parser.add_argument("--file", default="data_ramp_motors_O250_00.csv", help="csv file")
args = parser.parse_args()

data = loadFile(args.file)

fig, ax = plt.subplots(3)

### Plotting PWM to RPM
ax[0].plot(data["pwm"], data["rpm_avg"], label="data")
ax[0].set_xlabel("PWM")
ax[0].set_ylabel("RPM")

# Fit linear function y = ax + b; don't use PWM < 15000 for fit
indices = np.where(data["pwm"] > 20000)[0]
first_index = indices[0]
last_index = indices[-1]
z = np.polyfit(
    data["pwm"][first_index:last_index], data["rpm_avg"][first_index:last_index], 1
)
print(f"Fitted parameters linear: {z}")
pwm2rpm = np.poly1d(z)
xp = np.linspace(np.min(data["pwm"]), np.max(data["pwm"]), 100)
ax[0].plot(xp, pwm2rpm(xp), "--", label="fit $ax+b$")
ax[0].legend()

### Plotting RPM to Thrust
ax[1].plot(data["rpm_avg"], data["thrust"], label="data")
ax[1].set_xlabel("RPM")
ax[1].set_ylabel("Thrust [N]")

z = np.polyfit(data["rpm_avg"], data["thrust"], 2)
print(f"Fitted parameters quadratic: {z}")
rpm2thrust = np.poly1d(z)

xp = np.linspace(np.min(data["rpm_avg"]), np.max(data["rpm_avg"]), 100)

ax[1].plot(xp, rpm2thrust(xp), "--", label="fit")
ax[1].legend()

### Plotting PWM to Thrust
ax[2].plot(data["pwm"], data["thrust"], label="data")
ax[2].plot(
    data["pwm"],
    rpm2thrust(pwm2rpm(data["pwm"])),
    "--",
    label="fit",
)
ax[2].set_xlabel("PWM")
ax[2].set_ylabel("Thrust [N]")
ax[2].legend()

# plt.show()

fig, ax = plt.subplots(2)
### Plotting deviation in RPM
ax[0].plot(data["rpm2"] - data["rpm1"], label="M2-M1")
ax[0].plot(data["rpm3"] - data["rpm1"], label="M3-M1")
ax[0].plot(data["rpm4"] - data["rpm1"], label="M4-M1")
ax[0].set_xlabel("Time")
ax[0].set_ylabel("Relative RPM")
ax[0].legend()

### Plotting Thrust to Power
ax[1].plot(data["thrust"], data["p"], label="data")
ax[1].set_xlabel("Thrust [N]")
ax[1].set_ylabel("Power [W]")
ax[1].legend()
plt.show()
