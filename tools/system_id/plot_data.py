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

    fig,ax = plt.subplots()
    ax.plot(rpm, thrust)
    ax.set_xlabel('RPM')
    ax.set_ylabel('Thrust [g]')

    plt.show()
