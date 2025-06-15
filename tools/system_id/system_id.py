import argparse

import numpy as np
from sklearn.linear_model import LinearRegression
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
from utils import loadFiles, cutData, poly, inversepoly, loadYAML, storeYAML

PWM_MAX = 65535


def system_id_static(filenames, validations=[]):
    data = loadFiles(filenames)
    if len(validations) > 0:
        data_val = loadFiles(validations)

    ### v_motors -> thrust, where v_motors = duty_cycle * v_bat
    print("#########################################################")
    print("#### V_motors [V] to Thrust [N] curve")
    print("#########################################################")

    data["vmotors"] = data["vbat"] * data["pwm"] / PWM_MAX
    plt.scatter(data["vmotors"], data["thrust"] / 4, label="training data")

    if len(validations) > 0:
        data_val["vmotors"] = data_val["vbat"] * data_val["pwm"] / PWM_MAX
        plt.scatter(
            data_val["vmotors"], data_val["thrust"] / 4, label="validation data"
        )

    # Fitting
    X = np.vstack((data["vmotors"], data["vmotors"] ** 2, data["vmotors"] ** 3)).T
    reg = LinearRegression().fit(
        X, data["thrust"] / 4
    )  # positive=True, Ridge, LinearRegression
    p_vmotor2thrust = np.array(
        [reg.intercept_, reg.coef_[0], reg.coef_[1], reg.coef_[2]]
    )
    storeYAML(comb, p_vmotor2thrust, "p_vmotor2thrust")
    # INFO: We want to achieve ~1.7G for the thrust, meaning 35g * 1.7 / 1000 * 9.81N/kg = 0.58N
    # However: From the data, we can see that 0.58N is infeasible for the whole battery range.
    # For the 250mAh battery, we assume 0.45N and for the 350mAh battery 0.65N (with thrust upgrade kit).
    # Even with those values, when the battery is low, the max thrust might not be reachable
    if comb[1:] == "250":
        THRUST_MIN = 0.02
        THRUST_MAX = 0.45 / 4
    elif comb[1:] == "350":
        THRUST_MIN = 0.03
        THRUST_MAX = 0.65 / 4
    else:
        THRUST_MIN = 0.02
        THRUST_MAX = 0.45 / 4
        print(f"WARNING: Comb {comb} not found, setting THRUST_MAX={THRUST_MAX}")

    VMOTOR_MIN = inversepoly(THRUST_MIN, p_vmotor2thrust, 3)
    VMOTOR_MAX = inversepoly(THRUST_MAX, p_vmotor2thrust, 3)

    storeYAML(comb, THRUST_MIN, "THRUST_MIN")
    storeYAML(comb, THRUST_MAX, "THRUST_MAX")
    storeYAML(comb, VMOTOR_MIN, "VMOTOR_MIN")
    storeYAML(comb, VMOTOR_MAX, "VMOTOR_MAX")

    plt.scatter(VMOTOR_MAX, THRUST_MAX, marker="x", label="limit")

    X = np.linspace(0.9, 3.2, 1000)
    Y = poly(X, p_vmotor2thrust, 3)
    plt.plot(X, Y, label="fit", color="tab:green")

    print(
        f"Thrust = {reg.intercept_:.6f} + {reg.coef_[0]:.6f}*V + {reg.coef_[1]:.6f}*V^2 + {reg.coef_[2]:.6f}*V^3"
    )
    print(f"Thrust_MIN = {THRUST_MIN:.4f}N, Thrust_MAX={THRUST_MAX:.4f}N")
    # INFO: VoltageBatteryLow is 3.2 and critically low is 3.0
    print(f"VMotor_MIN = {VMOTOR_MIN:.4f}V, VMotor_MAX={VMOTOR_MAX:.4f}V")

    if len(validations) > 0:
        X_val = np.vstack(
            (data_val["vmotors"], data_val["vmotors"] ** 2, data_val["vmotors"] ** 3)
        ).T
        Y_val = data_val["thrust"] / 4
        Error = reg.predict(X_val) - Y_val
        print(f"validation score = {reg.score(X_val, Y_val):.4f}")
        print(f"validation max error = {np.max(np.abs(Error)) * 1000:.4f}mN")
        print(f"validation mean error = {np.mean(np.abs(Error)) * 1000:.4f}mN")

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

    fig, axs = plt.subplots(2, 1)

    axs[0].scatter(data["vmotors"], data["rpm_avg"], label="data")
    X = np.vstack((data["vmotors"]))
    reg = LinearRegression().fit(X, data["rpm_avg"])
    p_vmotor2rpm = [reg.intercept_, reg.coef_[0]]
    storeYAML(comb, p_vmotor2rpm, "p_vmotor2rpm")
    vmotors = np.linspace(1, 3, 1000)
    axs[0].plot(vmotors, poly(vmotors, p_vmotor2rpm, 1), label="fit", color="tab:green")
    axs[0].set_xlabel("Motor voltage [V]")
    axs[0].set_ylabel("Motor RPM")
    axs[0].legend()

    axs[1].scatter(data["rpm_avg"], data["thrust"], label="data")
    X = np.vstack((data["rpm_avg"], data["rpm_avg"] ** 2)).T
    reg = LinearRegression(fit_intercept=True).fit(X, data["thrust"])
    p_rpm2thrust = [reg.intercept_, reg.coef_[0], reg.coef_[1]]
    storeYAML(comb, p_rpm2thrust, "p_rpm2thrust")
    rpms = np.linspace(000, 30000, 1000)
    axs[1].plot(rpms, poly(rpms, p_rpm2thrust, 2), label="fit", color="tab:green")
    axs[1].set_xlabel("Motor RPM")
    axs[1].set_ylabel("Thrust [N]")
    axs[1].legend()

    axs[0].title.set_text("Motor voltage to RPM and RPM to thrust")
    plt.show()

    print(f"Thrust = {reg.intercept_} + {reg.coef_[0]}*RPM + {reg.coef_[1]}*RPM^2")

    ### power -> thrust (+ efficiency)
    print("#########################################################")
    print("#### Efficiency")
    print("#########################################################")

    plt.scatter(data["p"], data["thrust"], label="training data")

    if len(validations) > 0:
        plt.scatter(data_val["p"], data_val["thrust"], label="validation data")

    # fit
    X = np.vstack((data["p"]))
    Y = data["thrust"]
    reg = LinearRegression().fit(X, Y)
    print(f"Efficiency: {reg.coef_[0]:.3f}N/W or {reg.coef_[0] / 9.81 * 1000:.3f}g/w")

    X = np.linspace(2.0, 15.0, 1000)
    X_fit = np.vstack((X))
    Y_fit = reg.predict(X_fit)
    plt.plot(X, Y_fit, label="fit", color="tab:green")

    plt.xlabel("Total Power [W]")
    plt.ylabel("Total Thrust [N]")
    plt.title("Efficiency")
    plt.legend()
    plt.show()


def system_id_verification(filenames, validations=[]):
    data = loadFiles(filenames)
    if len(validations) > 0:
        data_val = loadFiles(validations)

    ### 3D Plot
    fig = plt.figure()
    plt.tight_layout()
    ax = fig.add_subplot(projection="3d")
    # ax = fig.gca(projection='3d')

    ax.scatter(data["cmd"], data["vbat"], data["thrust"] / 4, label="compensated")
    if len(validations) > 0:
        ax.scatter(
            data_val["cmd"],
            data_val["vbat"],
            data_val["thrust"] / 4,
            label="compensated old",
        )

    # plane for verification
    X = np.linspace(15000, 65535, 10)  # PWM
    Y = np.linspace(3.0, 4.2, 10)  # VBat
    X, Y = np.meshgrid(X, Y)
    Z = X / PWM_MAX * THRUST_MAX
    ax.plot_surface(X, Y, Z, alpha=0.2)  # , label="ideal compensation"

    ax.set_xlabel("Thrust command in PWM")
    ax.set_ylabel("Battery voltage [V]")
    ax.set_zlabel("Actual thrust [N]")
    ax.set_title("Thrust Verification")
    ax.legend()
    plt.show()

    ### 2D Plot
    X = np.linspace(15000, 65535, 10)  # PWM
    Y = X / PWM_MAX * THRUST_MAX
    plt.plot(X, Y, label="ideal compensation", color="tab:green")
    plt.scatter(data["cmd"], data["thrust"] / 4, label="compensated")
    if len(validations) > 0:
        Y_OLD = X / PWM_MAX * (0.06 * 9.81 / 4)
        plt.plot(X, Y_OLD, "--", label="ideal compensation old")
        plt.scatter(data_val["cmd"], data_val["thrust"] / 4, label="compensated old")
    plt.xlabel("Thrust command in PWM")
    plt.ylabel("Thrust per motor [N]")
    plt.title("Thrust Verification")
    plt.legend()
    plt.show()

    points_plane = data["cmd"] / PWM_MAX * THRUST_MAX
    errors = points_plane - data["thrust"] / 4
    print(f"max error = {np.max(np.abs(errors)) * 1000:.4f}mN")
    print(f"mean error = {np.mean(np.abs(errors)) * 1000:.4f}mN")


def system_id_dynamic(filenames, validations=[]):
    data = loadFiles(filenames)
    data = cutData(data, tStart=19, tEnd=34)
    p_vmotor2thrust = loadYAML(comb, "p_vmotor2thrust", 4)
    p_rpm2thrust = loadYAML(comb, "p_rpm2thrust", 3)

    thrustCMD = data["cmd"] / PWM_MAX * THRUST_MAX * 4
    rpmCMD = inversepoly(thrustCMD, p_rpm2thrust, 2)
    VmotorCMD = np.zeros_like(data["cmd"])
    for i in range(len(data["cmd"])):
        if thrustCMD[i] < THRUST_MIN:
            VmotorCMD[i] = 0
        else:
            VmotorCMD[i] = inversepoly(thrustCMD[i], p_vmotor2thrust, 3)

    Vmotor = data["vbat"] * data["pwm"] / PWM_MAX

    ### Fit dynamics
    # There are two possible ways to calculate the thrust dynamics:
    # The first one is to assume the motor speed as a first order
    # system, which is physically correct, if we ignore the drag
    # and DC motor electrical dynamics. The equation is
    # rpm_dot = 1/tau_rpm * (rpm_cmd - rpm)
    # Since we have 4 motors, we average the rpms
    # Alternatively, we can assume the thrust itself to be a first
    # order system: thrust_dot = 1/tau_f * (thrust_cmd - thrust)

    # Smoothing the data for better gradient
    rpm_filtered = savgol_filter(data["rpm_avg"], 10, 4)
    rpm_dot = np.gradient(rpm_filtered, data["time"])
    thrust_filtered = savgol_filter(data["thrust"], 10, 4)
    thrust_dot = np.gradient(thrust_filtered, data["time"])

    A = (rpmCMD - rpm_filtered).reshape(-1, 1)
    b = rpm_dot.reshape(-1, 1)
    k, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    tau_rpm = 1 / k[0, 0]

    A = (thrustCMD - thrust_filtered).reshape(-1, 1)
    b = thrust_dot.reshape(-1, 1)
    k, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    tau_f = 1 / k[0, 0]

    def first_order_system(x, u, tau, dt):
        x_dot = 1 / tau * (u - x)
        return x + x_dot * dt

    dts = np.diff(data["time"])
    rpmPred = [rpmCMD[0]]
    thrustPred = [thrustCMD[0]]
    for i, dt in enumerate(dts):
        rpm = first_order_system(rpmPred[-1], rpmCMD[i], tau_rpm, dt)
        rpmPred.append(rpm)
        thrust = first_order_system(thrustPred[-1], thrustCMD[i], tau_f, dt)
        thrustPred.append(thrust)

    print(f"tau_rpm = {tau_rpm}")
    storeYAML(comb, tau_rpm, "TAU_RPM")
    print(f"tau_f = {tau_f}")
    storeYAML(comb, tau_f, "TAU_F")

    plt.tight_layout()

    plt.plot(data["time"], thrust_filtered, label="Thrust Measurement (filtered)")
    plt.plot(data["time"], thrustCMD, label="Thrust CMD")
    plt.plot(
        data["time"],
        thrustPred,
        "--",
        label="Thrust Prediction from thrust dynamics",
    )
    plt.plot(
        data["time"],
        poly(np.array(rpmPred), p_rpm2thrust, 2),
        "--",
        label="Thrust Prediction from RPM dynamics",
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Total Thrust [N]")
    plt.title("Thrust Delay")
    plt.legend()
    plt.show()

    plt.plot(data["time"], data["vbat"], label="Vbat")
    plt.plot(data["time"], VmotorCMD, label="Vmotor CMD")
    plt.plot(data["time"], Vmotor, label="Vmotor actual")
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [V]")
    plt.title("Battery Compensation")
    plt.legend()
    plt.show()

    plt.plot(data["time"], data["pwm"], label="PWM Motors")
    plt.plot(data["time"], data["cmd"], label="PWM CMD")
    plt.xlabel("Time [s]")
    plt.ylabel("PWM")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        default="",
        help="Type of data collected, for more information see readme",
    )
    parser.add_argument(
        "--comb",
        default="",
        help="Combination of propellers, motors, and battery, for more information see readme",
    )
    parser.add_argument(
        "--extra", default="", help="Additional information for labeling the csv file"
    )
    parser.add_argument("--file", default="", help="Specific csv file used")
    args = parser.parse_args()

    mode = args.mode
    comb = args.comb
    combVal = comb
    extra = args.extra
    file = args.file

    if args.file != "":
        files = [args.file]
        filesVal = []
    else:
        if extra != "":
            comb = f"{comb}_{extra}"
        files = [
            # f"data_{mode}_{comb}_M1_00.csv",
            # f"data_{mode}_{comb}_M1_01.csv",
            # f"data_{mode}_{comb}_M1_02.csv",
            f"data_{mode}_{comb}_M2_00.csv",
            # f"data_{mode}_{comb}_M2_01.csv",
            # f"data_{mode}_{comb}_M2_02.csv",
            # f"data_{mode}_{comb}_M3_00.csv",
            # f"data_{mode}_{comb}_M3_01.csv",
            # f"data_{mode}_{comb}_M3_02.csv",
            # f"data_{mode}_{comb}_M4_00.csv",
            # f"data_{mode}_{comb}_M4_01.csv",
            # f"data_{mode}_{comb}_M4_02.csv",
        ]
        combVal = "P250"
        filesVal = [
            # f"data_{mode}_{combVal}_M1_02.csv",
            # f"data_{mode}_{combVal}_M3_02.csv",
            # f"data_{mode}_{combVal}_M2_03.csv",
            # f"data_{mode}_{combVal}_M4_01.csv",
            # f"data_{mode}_{combVal}_M4_02.csv",
            # f"data_{mode}_{combVal}_M4_old_00.csv",
        ]

    if args.mode == "ramp_motors":
        raise NotImplementedError("For ramp_motors, please use the plotting script.")
    elif args.mode == "static":
        system_id_static(files, filesVal)
    elif args.mode == "static_verification":
        THRUST_MIN = loadYAML(comb, "THRUST_MIN")
        THRUST_MAX = loadYAML(comb, "THRUST_MAX")
        system_id_verification(files, filesVal)
    elif args.mode == "dynamic":
        THRUST_MIN = loadYAML(comb, "THRUST_MIN")
        THRUST_MAX = loadYAML(comb, "THRUST_MAX")
        system_id_dynamic(files, filesVal)
    else:
        raise NotImplementedError(
            f"System Identification Type {args.mode} is not implemented."
        )
