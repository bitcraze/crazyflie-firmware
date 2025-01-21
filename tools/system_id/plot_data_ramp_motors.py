import numpy as np
import matplotlib.pyplot as plt
import argparse
from plot_helpers import loadFile

parser = argparse.ArgumentParser()
parser.add_argument("--file", default="data_ramp_motors_-B350_0.csv", help="csv file")
args = parser.parse_args()

data = loadFile(args.file)

fig,ax = plt.subplots(3)

### Plotting PWM to RPM
ax[0].plot(data["pwm"], data["rpm_avg"], label='data')
ax[0].set_xlabel("PWM")
ax[0].set_ylabel("RPM")

# Fit exponential function y = ae^(-x) + bx + c
# X = np.vstack([np.exp(-data["pwm"]*1.5e-4), data["pwm"], np.ones_like(data["pwm"])]).T
X = np.vstack([-np.exp(-data["pwm"]*1.5e-4) + 1, data["pwm"]]).T

coeffs, residuals, rank, s = np.linalg.lstsq(X, data["rpm_avg"], rcond=None)
# a, b, c = coeffs
a, b = coeffs
pwm2rpm = lambda pwm: a*-np.exp(-pwm*1.5e-4) + b*pwm + a  # noqa: E731
print(f"Fitted parameters exponential: {coeffs}")
ax[0].plot(data["pwm"], X @ coeffs, "--", label="fit $ae^{-x*1.5*e-4}+bx+c$")
ax[0].set_title(f"RPM [1/min] = {a:.3e} * e^(-PWM*1.5*e-4) + {b:.3e} * PWM + {a:.3e}")

# Fit linear function y = ax + b; don't use PWM < 15000 for fit
indices = np.where(data["pwm"] > 20000)[0]
first_index = indices[0]
last_index = indices[-1]
z = np.polyfit(data["pwm"][first_index:last_index], data["rpm_avg"][first_index:last_index], 1)
print(f"Fitted parameters linear: {z}")
p = np.poly1d(z)
xp = np.linspace(np.min(data["pwm"]), np.max(data["pwm"]), 100)
ax[0].plot(xp, p(xp), '--', label='fit $ax+b$')
ax[0].legend()

### Plotting RPM to Thrust
ax[1].plot(data["rpm_avg"], data["thrust"], label='data')
ax[1].set_xlabel('RPM')
ax[1].set_ylabel('Thrust [N]')

z = np.polyfit(data["rpm_avg"], data["thrust"], 2)
print(f"Fitted parameters quadratic: {z}")
rpm2thrust = np.poly1d(z)

xp = np.linspace(np.min(data["rpm_avg"]), np.max(data["rpm_avg"]), 100)

ax[1].plot(xp, rpm2thrust(xp), '--', label='fit')
ax[1].set_title(f"thrust [N] = {z[0]:.3e} * rpm^2 + {z[1]:.3e} * rpm + {z[2]:.3e}")
ax[1].legend()

### Plotting PWM to Thrust
ax[2].plot(data["pwm"], data["thrust"], label='data')
ax[2].plot(data["pwm"], rpm2thrust(pwm2rpm(data["pwm"])), '--', label='fit')
# From firmware: // thrust = a * pwm^2 + b * pwm // where PWM is normalized (range 0...1) // thrust is in Newtons (per rotor)
# static float pwmToThrustA = 0.091492681f;
# static float pwmToThrustB = 0.067673604f;
ax[2].plot(data["pwm"], (0.091492681*(data["pwm"]/65535)**2+0.067673604*(data["pwm"]/65535))/(9.81/1000), '--', label='firmware (old)')
ax[2].set_xlabel('PWM')
ax[2].set_ylabel('Thrust [N]')
ax[2].legend()

# plt.show()

fig, ax = plt.subplots(2)
### Plotting deviation in RPM
ax[0].plot(data["rpm2"] - data["rpm1"], label='M2-M1')
ax[0].plot(data["rpm3"] - data["rpm1"], label='M3-M1')
ax[0].plot(data["rpm4"] - data["rpm1"], label='M4-M1')
ax[0].set_xlabel('Time')
ax[0].set_ylabel('Relative RPM')
ax[0].legend()

### Plotting Thrust to Power
# print(data[0, 7]/vbat[0])

# ax.plot(data["thrust"], vbat, label='pm.vbat')
# ax.plot(data["thrust"], data[:, 7]/1.0955852742562724, label='i2c.v_avg')
# ax.plot(rpm, data[:, 8], label='i2c.i_avg')
ax[1].plot(data["thrust"], data["p"], label='data') # TODO fit
ax[1].set_xlabel("Thrust [N]")
ax[1].set_ylabel("Power [W]")
ax[1].legend()
plt.show()
