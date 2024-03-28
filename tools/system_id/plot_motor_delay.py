import matplotlib.pyplot as plt
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("file", help="csv file")
args = parser.parse_args()

data = np.loadtxt(args.file, delimiter=',', skiprows=1)

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


# u dot = lambda (u_des - u)
l = 16

u_pred = [0]
dts = np.diff(data[:,0]) / 1e3
for i, dt in enumerate(dts):
	u = u_pred[-1]
	pwm_des = data[i,2]
	v = data[i,3]
	# convert pwm -> force
	f_des = pwm2force(pwm_des, v) * 4

	u_dot = l * (f_des - u)
	u_pred.append(u + u_dot * dt)
u_pred = np.array(u_pred)

fig, ax1 = plt.subplots()

ax1.plot((data[:,0] - data[0,0])/1e3, data[:,1], 'b', label="Load cell")
ax2 = ax1.twinx()
ax2.plot((data[:,0] - data[0,0])/1e3, data[:,2], 'g', label="PWM")

# ax1.plot((data[:,0] - data[0,0])/1e3, u_pred, 'r', label="Prediction")

# ax1.plot((data[:,0] - data[0,0])/1e3, data[:,1], 'b', label="Load cell")

# # compute f desired from pwm
# f_des = []
# for i in range(0, data.shape[0]):
# 	f_des.append(pwm2force(data[i,2], data[i,3]) * 4)
# f_des = np.array(f_des)

# ax1.plot((data[:,0] - data[0,0])/1e3, f_des, 'g', label="Desired")
# ax1.plot((data[:,0] - data[0,0])/1e3, u_pred, 'r', label="Prediction")
# ax1.plot((data[:, 0] - data[0, 0])/1e3, data[:, 2], 'r', label="PWM")


ax1.legend()

plt.show()
