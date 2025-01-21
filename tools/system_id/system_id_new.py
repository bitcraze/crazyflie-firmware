import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
from plot_helpers import loadFiles

PWM_MAX = 65535

def system_id(filenames, validations=[]):
    data = loadFiles(filenames)
    if len(validations) > 0:
        data_val = loadFiles(validations)

    ### v_motors -> thrust, where v_motors = duty_cycle * v_bat
    print("#########################################################")
    print("#### V_motors [V] to Thrust [N] curve")
    print("#########################################################")

    data['vmotors'] = data['vbat'] * data['pwm'] / PWM_MAX
    plt.scatter(data['vmotors'], data['thrust']/4, label="training data")

    if len(validations) > 0:
        data_val['vmotors'] = data_val['vbat'] * data_val['pwm'] / PWM_MAX
        plt.scatter(data_val['vmotors'], data_val['thrust']/4, label="validation data")

    # Fitting
    X = np.vstack((data['vmotors'], data['vmotors']**2)).T
    Y = data['thrust']/4
    reg = LinearRegression().fit(X, Y)
    
    print(f"Thrust = {reg.intercept_:.6f} {reg.coef_[0]:.6f}*V + {reg.coef_[1]:.6f}*V^2")
    print(f"Voltage = ({-reg.coef_[0]} + sqrt({reg.coef_[0]}**2 - 4*{reg.coef_[1]}*({reg.intercept_}-T))) / (2*{reg.coef_[1]})")
    print(f"Thrust_MIN = {np.min(data_val['thrust']/4):.4f}N, Thrust_MAX={np.max(data_val['thrust']/4):.4f}N")
    X = np.linspace(0.9, 3.2, 1000)
    X_fit = np.vstack((X, X**2)).T
    # Y_fit = reg.intercept_ + reg.coef_[0] * X_fit + reg.coef_[1] * X_fit**2
    Y_fit = reg.predict(X_fit)
    plt.plot(X, Y_fit, label="fit", color="green")

    Y_inv = np.linspace(0.01, 0.2, 1000)
    X_inv = (-reg.coef_[0] + np.sqrt(reg.coef_[0]**2 - 4*reg.coef_[1]*(reg.intercept_-Y_inv))) / (2*reg.coef_[1])
    plt.plot(X_inv, Y_inv, label="inv")

    if len(validations) > 0:
        X_val = np.vstack((data_val['vmotors'], data_val['vmotors']**2)).T
        Y_val = data_val['thrust']/4
        Error = reg.predict(X_val) - Y_val
        print(f"validation score = {reg.score(X_val, Y_val):.4f}")
        print(f"validation max error = {np.max(np.abs(Error))*1000:.4f}mN")
        print(f"validation mean error = {np.mean(np.abs(Error))*1000:.4f}mN")

    y_old = np.linspace(0.02, 0.14, 1000)
    # # Thrust = 0.409e-3 PWM^2 + 140.5e-3 PMW - 0.099
    # # kf = 3.16e-10
    # # pwm2rpm_scale = 0.2685 
    # # pwm2rpm_const = 4070.3 

    # # pwm_old = np.linspace(0, 65535, 1000)
    # # y_old = kf * (pwm2rpm_scale * pwm_old + pwm2rpm_const) ** 2
    # # y_old = (0.409e-3 * pwm_old**2 + 140.5e-3 * pwm_old - 0.099)*9.81
    x_old = -0.00062390 * (y_old/9.81*1000*4)**2 + 0.08835522 * y_old/9.81*1000*4 # + 0.06865956
    plt.plot(x_old, y_old, label="old_fit")

    plt.xlabel("V_motors [V]")
    plt.ylabel("Thrust per motor [N]")
    plt.title("Motor voltage to thrust")
    plt.legend()
    plt.show()

    ### power -> thrust (+ efficiency)
    print("#########################################################")
    print("#### Efficiency")
    print("#########################################################")

    plt.scatter(data['p'], data['thrust'], label="training data")
    
    if len(validations) > 0:
        plt.scatter(data_val['p'], data_val['thrust'], label="validation data")

    # fit
    X = np.vstack((data['p']))
    Y = data['thrust']
    reg = LinearRegression().fit(X, Y)
    print(f"Efficiency: {reg.coef_[0]:.3f}N/W or {reg.coef_[0]/9.81*1000:.3f}g/w")

    X = np.linspace(2.0, 14.0, 1000)
    X_fit = np.vstack((X))
    # Y_fit = reg.intercept_ + reg.coef_[0] * X_fit + reg.coef_[1] * X_fit**2
    Y_fit = reg.predict(X_fit)
    plt.plot(X, Y_fit, label="fit", color="green")
    

    plt.xlabel("Total Power [W]")
    plt.ylabel("Total Thrust [N]")
    plt.title("Efficiency")
    plt.legend()
    plt.show()

comb = "-B250"
comb_val = "+B250"
system_id([f"data_max_thrust_{comb}_00.csv", 
           f"data_max_thrust_{comb}_01.csv",
           f"data_max_thrust_{comb}_02.csv",
           f"data_max_thrust_{comb}_03.csv",
           f"data_max_thrust_{comb}_04.csv",
           f"data_max_thrust_{comb}_05.csv",
        #    f"data_max_thrust_{comb_val}_NEW_00.csv",
           ],
          validations=[f"data_max_thrust_{comb_val}_00.csv"])
