import argparse

import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
from helpers import loadFiles, poly, inversepoly, loadYAML, storeYAML

PWM_MAX = 65535

def system_id_static(filenames, validations=[]):
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
    p_v_thrust = np.array([reg.intercept_, reg.coef_[0], reg.coef_[1], reg.coef_[2]])
    storeYAML(comb, p_v_thrust, "p_v_thrust")
    # INFO: We want to achieve ~1.7G for the thrust, meaning 27g * 1.7 / 1000 * 9.81N/kg = 0.45N
    THRUST_MIN = poly(1.0, p_v_thrust, 3)
    THRUST_MAX = 0.45/4
    storeYAML(comb, THRUST_MIN, "THRUST_MIN")
    storeYAML(comb, THRUST_MAX, "THRUST_MAX")
    
    X = np.linspace(0.9, 3.2, 1000)
    plt.plot(X, poly(X, p_v_thrust, 3), label="fit", color="green")

    # Y_inv = np.linspace(THRUST_MIN, THRUST_MAX, 1000)
    # X_inv = inversepoly(Y_inv, p_v_thrust, 3)
    # plt.plot(X_inv, Y_inv, label="inv")

    print(f"Thrust = {reg.intercept_:.6f} {reg.coef_[0]:.6f}*V + {reg.coef_[1]:.6f}*V^2")
    print(f"Voltage = ({-reg.coef_[0]} + sqrt({reg.coef_[0]}**2 - 4*{reg.coef_[1]}*({reg.intercept_}-T))) / (2*{reg.coef_[1]})")
    print(f"Thrust_MIN = {THRUST_MIN:.4f}N, Thrust_MAX={THRUST_MAX:.4f}N")
    # INFO: VoltageBatteryLow is 3.2 and critically low is 3.0 
    print(f"VMotor_MIN = {inversepoly(THRUST_MIN, p_v_thrust, 3):.4f}V, VMotor_MAX={inversepoly(THRUST_MAX, p_v_thrust, 3):.4f}V")
    storeYAML(comb, inversepoly(THRUST_MIN, p_v_thrust, 3), "VMOTOR_MIN")
    storeYAML(comb, inversepoly(THRUST_MAX, p_v_thrust, 3), "VMOTOR_MAX")

    if len(validations) > 0:
        X_val = np.vstack((data_val['vmotors'], data_val['vmotors']**2)).T
        Y_val = data_val['thrust']/4
        Error = reg.predict(X_val) - Y_val
        print(f"validation score = {reg.score(X_val, Y_val):.4f}")
        print(f"validation max error = {np.max(np.abs(Error))*1000:.4f}mN")
        print(f"validation mean error = {np.mean(np.abs(Error))*1000:.4f}mN")

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

def system_id_verification(filenames, validations=[]):
    data = loadFiles(filenames)
    if len(validations) > 0:
        data_val = loadFiles(validations)

    fig = plt.figure()
    plt.tight_layout()
    # ax = fig.add_subplot(projection='3d')
    ax = fig.gca(projection='3d')

    ax.scatter(data['cmd'], data['vbat'], data['thrust']/4, label="compensated")
    if len(validations) > 0:
        ax.scatter(data_val['pwm'], data_val['vbat'], data_val['thrust']/4, label="uncompensated")

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

def system_id_dynamic(filenames, validations=[]): 
    data = loadFiles(filenames)
    p_v_thrust = loadYAML(comb, "p_v_thrust", 4)

    # default
    c00 = 11.093358483549203
    c10 = -39.08104165843915
    c01 = -9.525647087583181
    c20 = 20.573302305476638
    c11 = 38.42885066644033

    def pwm2force(pwm, v):
        pwm = pwm / 65535
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

    thrustCMD = data["cmd"]/PWM_MAX*THRUST_MAX
    VmotorCMD = np.zeros_like(data["cmd"])
    for i in range(len(data["cmd"])):
        if thrustCMD[i] < THRUST_MIN:
            VmotorCMD[i] = 0
        else:
            VmotorCMD[i] = inversepoly(thrustCMD[i], p_v_thrust, 3)

    Vmotor = data['vbat'] * data["pwm"] / PWM_MAX



    fig, axs = plt.subplots(3,1)

    axs[0].plot(data["time"], data["thrust"], label="Load cell [N]")
    axs[0].plot(data["time"], data["cmd"]/PWM_MAX*THRUST_MAX*4, label="Thrust CMD [N]")

    axs[1].plot(data["time"], data["vbat"], label="Vbat [V]")
    axs[1].plot(data["time"], VmotorCMD, label="Vmotor CMD [V]")
    axs[1].plot(data["time"], Vmotor, label="Vmotor [V]")


    # ax2 for PWM and RPM
    # ax2 = ax1.twinx()
    axs[2].plot(data["time"], data["pwm"], label="PWM")
    axs[2].plot(data["time"], data["cmd"], label="PWM CMD")
    # ax2.plot(data["time"], data["rpm_avg"], label="RPM")
    # ax2.plot(time, pwm2rpm(data_pwm), label="RPM Setpoint")
    # ax2.plot(data["time"], rpm_pred, label="RPM Prediction")

    # ax1.plot(time, u_pred, 'r', label="Prediction")

    # compute f desired from pwm
    f_des = []
    for i in range(len(data["time"])):
        f_des.append(pwm2force(data["pwm"][i], data["vbat"][i]) * 4)
    f_des = np.array(f_des)

    # ax1.plot(time, f_des, 'g', label="F Desired")

    axs[0].legend()
    axs[1].legend()
    axs[2].legend()

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="", help="Type of data collected, for more information see readme")
    parser.add_argument("--comb", default="", help="Combination of propellers, motors, and battery, for more information see readme")
    parser.add_argument("--file", default="", help="Specific csv file used")
    args = parser.parse_args()

    mode = args.mode
    comb = args.comb
    file = args.file

    THRUST_MIN = loadYAML(comb, "THRUST_MIN")
    THRUST_MAX = loadYAML(comb, "THRUST_MAX")

    if args.file != "":
        files = [args.file]
        filesVal = []
    else:
        files = [
            f"data_{mode}_{comb}_00.csv", 
            # f"data_{mode}_{comb}_01.csv",
            # f"data_{mode}_{comb}_02.csv",
            # f"data_{mode}_{comb}_03.csv",
        ]
        filesVal = [
            # f"data_{mode}_{comb}_04.csv", 
        ]

    if args.mode == "ramp_motors":
        raise NotImplementedError("For ramp_motors, please use the plotting script.")
    elif args.mode == "static":
        system_id_static(files, filesVal)
    elif args.mode == "verification": # activate battery compensation to test it
        system_id_verification(files)
    elif args.mode == "dynamic":
        system_id_dynamic(files, filesVal)
    else:
        raise NotImplementedError(f"System Identification Type {args.mode} is not implemented.")

    
    
