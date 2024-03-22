import numpy as np
import matplotlib.pyplot as plt
import argparse

def loadFile(filename):
    fileData = np.loadtxt(filename, delimiter=',', skiprows=1, ndmin=2)
    return fileData

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("filename")
    args = parser.parse_args()
    data = loadFile(args.filename)

    thrust = data[:,0] / 4  # g, per motor
    pwm = data[:,1]         # PWM value
    vbat = data[:,2]        # V, battery voltage, 
    rpm = np.mean(data[:,3:7],axis=1) # average over all motors
    vSid =  data[:, 7]      # Volts at system id deck
    amp =  data[:, 8]       # Amps
    pwr = data[:, 9]        # Power in watts
    

    fig, ax = plt.subplots(3)

    print(vSid[0]/vbat[0])

    ax[0].plot(data[:,3], label='M1')
    ax[0].plot(data[:,4], label='M2')
    ax[0].plot(data[:,5], label='M3')
    ax[0].plot(data[:,6], label='M4')
    ax[0].plot(rpm, label='Mean')
    ax[0].set_xlabel('Time')
    ax[0].set_ylabel('RPM')
    ax[0].legend()
    ax[0].grid(True)

    ax2 = ax[0].twinx()
    ax2.plot((pwr), label='P[W]')
    ax2.plot(pwm/6553.5, color='tab:red', label='PWM[0-10]')
    ax2.plot(thrust/((pwr / 4)), label='Efficency [g/W]')
    ax2.set_ylabel('PWM[0-10], Power[W], Efficiency[g/W]', color='tab:red')
    ax2.legend()

    ax[1].plot(rpm, amp, label='I [A]')
    ax[1].plot(rpm, vSid, label='V PCB [V]')
    ax[1].plot(rpm, vbat, label='V Bat [V]')
    ax[1].plot(rpm, thrust/((data[:, 9] / 4)), label='Efficency [g]/Watt')
    ax[1].set_xlabel('RPM')
    ax[1].set_ylabel('Voltage and Current')
    ax[1].legend()
    ax[1].grid(True)

    ax[2].plot(thrust/((data[:, 9] / 4)))
    ax[2].set_xlabel('time')
    ax[2].set_ylabel('Efficency [g/W]')
    ax[2].legend()
    ax[2].grid(True)

    ax3 = ax[2].twinx()
    ax3.plot(thrust*4, color='tab:red')
    ax3.set_ylabel('thrust [g]', color='tab:red')
    ax3.legend()
    
    fig.tight_layout()
    plt.show()