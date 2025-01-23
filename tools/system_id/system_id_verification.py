import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
from helpers import loadFiles, loadYAML

comb = "+B250"
comb_val = "+B250"
PWM_MAX = 65535
THRUST_MAX = loadYAML(comb, "THRUST_MAX")

def system_id(filenames, validations=[]):
    data = loadFiles(filenames)
    # if len(validations) > 0:
    #     data_val = loadFiles(validations)

    fig = plt.figure()
    plt.tight_layout()
    # ax = fig.add_subplot(projection='3d')
    ax = fig.gca(projection='3d')

    ax.scatter(data['cmd'], data['vbat'], data['thrust']/4, label="compensated")
    # ax.scatter(data_val['pwm'], data_val['vbat'], data_val['thrust']/4, label="uncompensated")

    # plane for verification
    X = np.linspace(15000, 65535, 10) # PWM
    Y = np.linspace(3.0, 4.2, 10) # VBat
    X, Y = np.meshgrid(X, Y)
    Z = X/PWM_MAX*THRUST_MAX
    ax.plot_surface(X, Y, Z, alpha=0.2)

    ax.set_xlabel("Thrust command in PWM")
    ax.set_ylabel("Battery voltage [V]")
    ax.set_zlabel("Actual thrust [N]")
    ax.set_title("Thrust Verification")
    ax.legend()
    plt.show()

    points_plane = data['cmd']/PWM_MAX*THRUST_MAX
    errors = points_plane - data['thrust']/4
    print(f"max error = {np.max(np.abs(errors))*1000:.4f}mN")
    print(f"mean error = {np.mean(np.abs(errors))*1000:.4f}mN")

system_id([f"data_verification_{comb}_00.csv", 
        #    f"data_max_thrust_{comb}_01.csv",
        #    f"data_max_thrust_{comb}_02.csv",
        #    f"data_max_thrust_{comb}_03.csv",
        #    f"data_max_thrust_{comb}_04.csv",
        #    f"data_max_thrust_{comb}_05.csv",
        #    f"data_max_thrust_{comb_val}_NEW_00.csv",
           ],
          validations=[f"data_max_thrust_{comb_val}_00.csv"])
