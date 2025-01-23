import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
from helpers import loadFiles, poly, storeYAML

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
    X = np.vstack((data['vmotors'], data['vmotors']**2, data['vmotors']**3)).T
    reg = LinearRegression().fit(X, data['thrust']/4)
    p_v_thrust = [reg.intercept_, reg.coef_[0], reg.coef_[1], reg.coef_[2]]
    storeYAML(comb, p_v_thrust, "p_v_thrust")
    
    X = np.linspace(0.9, 3.2, 1000)
    plt.plot(X, poly(X, p_v_thrust, 3), label="fit", color="green")

    inverse = lambda thrust: (-p_v_thrust[1] + np.sqrt(p_v_thrust[1]**2 - 4*p_v_thrust[2]*(p_v_thrust[0]-thrust))) / (2*p_v_thrust[2])
    # inverse = lambda thrust: 
    # Y_inv = np.linspace(0.01, 0.2, 1000)
    # X_inv = inverse(Y_inv)
    # plt.plot(X_inv, Y_inv, label="inv")

    print(f"Thrust = {reg.intercept_:.6f} {reg.coef_[0]:.6f}*V + {reg.coef_[1]:.6f}*V^2")
    print(f"Voltage = ({-reg.coef_[0]} + sqrt({reg.coef_[0]}**2 - 4*{reg.coef_[1]}*({reg.intercept_}-T))) / (2*{reg.coef_[1]})")
    # INFO: We want to achieve ~1.7G for the thrust, meaning 27g * 1.7 / 1000 * 9.81N/kg = 0.45N
    THRUST_MIN = poly(1.0, p_v_thrust, 2)
    THRUST_MAX = 0.45/4
    print(f"Thrust_MIN = {THRUST_MIN:.4f}N, Thrust_MAX={THRUST_MAX:.4f}N")
    storeYAML(comb, THRUST_MIN, "THRUST_MIN")
    storeYAML(comb, THRUST_MAX, "THRUST_MAX")
    # INFO: VoltageBatteryLow is 3.2 and critically low is 3.0 
    print(f"VMotor_MIN = {inverse(THRUST_MIN):.4f}V, VMotor_MAX={inverse(THRUST_MAX):.4f}V")
    storeYAML(comb, inverse(THRUST_MIN), "VMOTOR_MIN")
    storeYAML(comb, inverse(THRUST_MAX), "VMOTOR_MAX")

    if len(validations) > 0:
        X_val = np.vstack((data_val['vmotors'], data_val['vmotors']**2)).T
        Y_val = data_val['thrust']/4
        # Error = reg.predict(X_val) - Y_val
        # print(f"validation score = {reg.score(X_val, Y_val):.4f}")
        # print(f"validation max error = {np.max(np.abs(Error))*1000:.4f}mN")
        # print(f"validation mean error = {np.mean(np.abs(Error))*1000:.4f}mN")

    # y_old = np.linspace(0.02, 0.14, 1000)
    # x_old = -0.00062390 * (y_old/9.81*1000*4)**2 + 0.08835522 * y_old/9.81*1000*4 # + 0.06865956
    # plt.plot(x_old, y_old, label="old_fit")

    plt.xlabel("V_motors [V]")
    plt.ylabel("Thrust per motor [N]")
    plt.title("Motor voltage to thrust")
    plt.legend()
    plt.show()

    ### v_motors -> RPM and RPM -> thrust, where v_motors = duty_cycle * v_bat
    print("#########################################################")
    print("#### V_motors [V] to RPM and RPM to thrust [N] curve")
    print("#########################################################")

    fig, axs = plt.subplots(2,1)

    axs[0].scatter(data['vmotors'], data['rpm_avg'], label="data")
    X = np.vstack((data['vmotors']))
    reg = LinearRegression().fit(X, data['rpm_avg'])
    p_v_rpm = [reg.intercept_, reg.coef_[0]]
    vmotors = np.linspace(1, 3, 1000)
    axs[0].plot(vmotors, poly(vmotors, p_v_rpm, 1), label="fit")
    axs[0].set_xlabel("Motor voltage [V]")
    axs[0].set_ylabel("Motor RPM")
    axs[0].legend()

    axs[1].scatter(data['rpm_avg'], data['thrust'], label="data")
    X = np.vstack((data['rpm_avg']**2))
    reg = LinearRegression().fit(X, data['thrust'])
    p_rpm_thrust = [0, 0, reg.coef_[0]]
    storeYAML(comb, reg.coef_[0], "kf")
    rpms = np.linspace(000, 25000, 1000)
    axs[1].plot(rpms, poly(rpms, p_rpm_thrust, 2), label="fit")
    axs[1].set_xlabel("Motor RPM")
    axs[1].set_ylabel("Thrust [N]")
    axs[1].legend()

    axs[0].title.set_text("Motor voltage to RPM and RPM to thrust")
    plt.show()

    print("Thrust = kf * RPM^2")
    print(f"kf = {reg.coef_[0]}")

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

comb = "+B250"
comb_val = "+B250"
system_id([f"data_max_thrust_{comb}_00.csv", 
        #    f"data_max_thrust_{comb}_01.csv",
        #    f"data_max_thrust_{comb}_02.csv",
        #    f"data_max_thrust_{comb}_03.csv",
        #    f"data_max_thrust_{comb}_04.csv",
        #    f"data_max_thrust_{comb}_05.csv",
        #    f"data_max_thrust_{comb_val}_NEW_00.csv",
           ],
          validations=[f"data_max_thrust_{comb_val}_00.csv"])
