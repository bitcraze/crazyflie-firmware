import numpy as np
import matplotlib.pyplot as plt

def loadFile(filename):
    fileData = np.loadtxt(filename, delimiter=',', skiprows=1, ndmin=2)
    return fileData

if __name__ == '__main__':
    data = loadFile("data.csv")

    thrust = data[:,0] / 4  # g, per motor
    pwm = data[:,1]         # PWM value
    vbat = data[:,2]        # V, battery voltage, 
    rpm = np.mean(data[:,3:7],axis=1) # average over all motors

    fig,ax = plt.subplots(2)

    ax[0].plot(data[:,4] - data[:,3], label='M2-M1')
    ax[0].plot(data[:,5] - data[:,3], label='M3-M1')
    ax[0].plot(data[:,6] - data[:,3], label='M4-M1')
    ax[0].set_xlabel('Time')
    ax[0].set_ylabel('Relative RPM')
    ax[0].legend()

    ax[1].plot(rpm, thrust)
    ax[1].set_xlabel('RPM')
    ax[1].set_ylabel('Thrust [g]')

    z = np.polyfit(rpm, thrust, 2)
    p = np.poly1d(z)

    xp = np.linspace(np.min(rpm), np.max(rpm), 100)

    ax[1].plot(xp, p(xp), '--')
    ax[1].set_title("thrust [g] = {} * rpm^2 + {} * rpm + {}".format(z[0], z[1], z[2]))

    plt.show()

    fig, ax = plt.subplots()

    print(data[0, 7]/vbat[0])

    # ax.plot(thrust, vbat, label='pm.vbat')
    # ax.plot(thrust, data[:, 7]/1.0955852742562724, label='i2c.v_avg')
    # ax.plot(rpm, data[:, 8], label='i2c.i_avg')
    ax.plot(thrust, data[:, 7] * data[:, 8], label='i2c.p_avg')
    ax.legend()
    plt.show()
