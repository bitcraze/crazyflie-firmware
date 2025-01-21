import matplotlib.pyplot as plt
import numpy as np
import argparse
from plot_helpers import loadFile


parser = argparse.ArgumentParser()
parser.add_argument("--file", default="data_motor_delay_-B350_0.csv", help="csv file")
args = parser.parse_args()

data = loadFile(args.file)

# default
c00 = 11.093358483549203
c10 = -39.08104165843915
c01 = -9.525647087583181
c20 = 20.573302305476638
c11 = 38.42885066644033

def pwm2force(pwm, v):
	pwm = pwm / 65536
	v = v / 4.2
	return c00 + c10 * pwm + c01 * v + c20 * pwm * pwm + c11 * pwm * v

def pwm2rpm(pwm):
	"""From earlier system identification."""
	return 4.684e3*(1-np.exp(-pwm*1.5e-4)) + 2.904e-1*pwm

def rpmdynamics(rpm, pwm, dt):
	"""Calculates the next RPM value based on the commanded PWM and the current RPM value.
	linear dynamics: rpm_dot = k * (rpm_des - rpm)
	TODO fit the parameters to the data. Right now it's just an estimate."""
	# TODO maybe make dynamics on PWM istead of RPM (time constant rn depends on the size of the jump, which shouldn't be the case)
	k_up = 10
	k_down = 5
	rpm_des = pwm2rpm(pwm)
	delta = rpm_des-rpm
	# print(f"delta={delta}")
	if delta>0:
		rpm_dot = k_up*delta
	else:
		rpm_dot = k_down*delta
	# print(f"dt={dt}")
	return rpm + rpm_dot*dt
	
rpm_pred = [0]
for i, dt in enumerate(np.diff(data["time"])):
	rpm_pred.append(rpmdynamics(rpm_pred[-1], data["pwm"][i], dt))

# u dot = lambda (u_des - u)
l = 16

u_pred = [0]
dts = np.diff(data["time"]) / 1e3
for i, dt in enumerate(dts):
	u = u_pred[-1]
	pwm_des = data["pwm"][i]
	v = data["vbat"][i]
	# convert pwm -> force
	f_des = pwm2force(pwm_des, v) * 4

	u_dot = l * (f_des - u)
	u_pred.append(u + u_dot * dt)
u_pred = np.array(u_pred)

fig, ax1 = plt.subplots()

ax1.plot(data["time"], data["thrust"], 'b', label="Load cell")

# ax2 for PWM and RPM
ax2 = ax1.twinx()
ax2.plot(data["time"], data["pwm"], label="PWM")
ax2.plot(data["time"], data["rpm_avg"], label="RPM")
# ax2.plot(time, pwm2rpm(data_pwm), label="RPM Setpoint")
ax2.plot(data["time"], rpm_pred, label="RPM Prediction")

# ax1.plot(time, u_pred, 'r', label="Prediction")

# compute f desired from pwm
f_des = []
for i in range(len(data["time"])):
	f_des.append(pwm2force(data["pwm"][i], data["vbat"][i]) * 4)
f_des = np.array(f_des)

# ax1.plot(time, f_des, 'g', label="F Desired")

ax1.legend(loc="upper left")
ax2.legend(loc="upper right")

plt.show()
