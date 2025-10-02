import argparse

from toml import load, dump
import numpy as np
from sklearn.linear_model import LinearRegression, RANSACRegressor
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
from utils import (
    loadFiles,
    cutData,
    poly,
    inversepoly,
    convert_parameters,
)

FIGSIZE = (10, 10)
PWM_MIN = 7000  # as in firmware
PWM_MAX = 65535  # as in firmware


def system_id_static(filenames, validations=[]):
    data = loadFiles(filenames)
    if len(validations) > 0:
        data_val = loadFiles(validations)

    ### v_motors -> thrust, where v_motors = duty_cycle * v_bat
    print("#########################################################")
    print("#### V_motors [V] to Thrust [N] curve")
    print("#########################################################")
    plt.figure(figsize=FIGSIZE)

    data["vmotors"] = data["vbat"] * data["pwm"] / PWM_MAX
    plt.scatter(data["vmotors"], data["thrust"] / 4, label=f"training data ({comb})")

    if len(validations) > 0:
        data_val["vmotors"] = data_val["vbat"] * data_val["pwm"] / PWM_MAX
        plt.scatter(
            data_val["vmotors"],
            data_val["thrust"] / 4,
            label=f"validation data ({combVal})",
        )

    # Fitting
    X = np.vstack((data["vmotors"], data["vmotors"] ** 2, data["vmotors"] ** 3)).T
    ransac = RANSACRegressor(LinearRegression()).fit(
        X, data["thrust"] / 4
    )  # positive=True, Ridge, LinearRegression
    reg = ransac.estimator_
    vmotor2thrust = [reg.intercept_, reg.coef_[0], reg.coef_[1], reg.coef_[2]]
    parameters["vmotor2thrust"] = vmotor2thrust
    # INFO: We want to achieve the max thrust for all SoC of the battery. Thus,
    # we limit the maximum thrust to a relatively convervative value. For a
    # discussion on that see #1526
    # The min thrust is determined by the min pwm hard coded on the drone by
    # CONFIG_MOTORS_DEFAULT_IDLE_THRUST
    if "L250" in comb or "P250" in comb:
        THRUST_MAX = 0.45 / 4
    elif "T350" in comb:
        THRUST_MAX = 0.65 / 4
    elif "B" in comb:
        THRUST_MAX = 0.8 / 4
    else:
        THRUST_MAX = 0.45 / 4
        print(f"WARNING: Comb {comb} not found, setting THRUST_MAX={THRUST_MAX}")
    THRUST_MIN = PWM_MIN / PWM_MAX * THRUST_MAX

    VMOTOR_MIN = inversepoly(THRUST_MIN, vmotor2thrust, 3)
    VMOTOR_MAX = inversepoly(THRUST_MAX, vmotor2thrust, 3)

    parameters["THRUST_MIN"] = THRUST_MIN
    parameters["THRUST_MAX"] = THRUST_MAX
    parameters["VMOTOR_MIN"] = VMOTOR_MIN
    parameters["VMOTOR_MAX"] = VMOTOR_MAX

    plt.scatter(
        VMOTOR_MAX,
        THRUST_MAX,
        marker="x",
        label="limit",
        c="tab:red",
        linewidths=3,
        s=100,
    )
    plt.scatter(
        VMOTOR_MIN,
        THRUST_MIN,
        marker="x",
        c="tab:red",
        linewidths=3,
        s=100,
    )

    X = np.linspace(0.95 * VMOTOR_MIN, 1.05 * VMOTOR_MAX, 1000)
    Y = poly(X, vmotor2thrust, 3)
    plt.plot(X, Y, label="fit", color="tab:red")

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

    plt.xlabel("V_motors [V]")
    plt.ylabel("Thrust per motor [N]")
    plt.title("Motor voltage to thrust")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    print("\n#########################################################")
    print("#### V_motors [V] to Torque [Nm] curve")
    print("#########################################################")
    plt.figure(figsize=FIGSIZE)
    plt.scatter(data["vmotors"], data["torque_z"] / 4, label="training data")

    if len(validations) > 0:
        plt.scatter(
            data_val["vmotors"], data_val["torque_z"] / 4, label="validation data"
        )

    # Fitting
    X = np.vstack((data["vmotors"], data["vmotors"] ** 2, data["vmotors"] ** 3)).T
    reg = LinearRegression().fit(X, data["torque_z"] / 4)
    vmotor2torque = [reg.intercept_, reg.coef_[0], reg.coef_[1], reg.coef_[2]]
    parameters["vmotor2torque"] = vmotor2torque

    print(
        f"Torque = {reg.intercept_:.6f} + {reg.coef_[0]:.6f}*V + {reg.coef_[1]:.6f}*V^2 + {reg.coef_[2]:.6f}*V^3"
    )

    X = np.linspace(0.95 * VMOTOR_MIN, 1.05 * VMOTOR_MAX, 1000)
    Y = poly(X, vmotor2torque, 3)
    plt.plot(X, Y, label="fit", color="tab:red")

    plt.xlabel("V_motors [V]")
    plt.ylabel("Torque per motor [Nm]")
    plt.title("Motor voltage to torque")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    ### v_motors -> RPM and RPM -> thrust, where v_motors = duty_cycle * v_bat
    print("\n#########################################################")
    print("#### V_motors [V] to RPM and RPM to thrust [N] curve")
    print("#########################################################")
    fig, axs = plt.subplots(2, 1, figsize=FIGSIZE)

    axs[0].scatter(data["vmotors"], data["rpm_avg"], label="training data")
    if len(validations) > 0:
        axs[0].scatter(data_val["vmotors"], data_val["rpm_avg"], label="training data")
    mask = data["vmotors"] > VMOTOR_MIN
    X = np.vstack((data["vmotors"][mask]))
    reg = LinearRegression(fit_intercept=True).fit(X, data["rpm_avg"][mask])
    vmotor2rpm = [reg.intercept_, reg.coef_[0]]
    parameters["vmotor2rpm"] = vmotor2rpm
    vmotors = np.linspace(0.95 * VMOTOR_MIN, 1.05 * VMOTOR_MAX, 1000)
    axs[0].plot(vmotors, poly(vmotors, vmotor2rpm, 1), label="fit", color="tab:red")
    axs[0].set_xlabel("Motor voltage [V]")
    axs[0].set_ylabel("Motor RPM")
    axs[0].legend()
    axs[0].grid()

    axs[1].scatter(data["rpm_avg"], data["thrust"] / 4, label="training data")
    if len(validations) > 0:
        axs[1].scatter(
            data_val["rpm_avg"], data_val["thrust"] / 4, label="validation data"
        )
    X = np.vstack((data["rpm_avg"], data["rpm_avg"] ** 2, data["rpm_avg"] ** 3)).T
    reg = LinearRegression(fit_intercept=False, positive=False).fit(
        X, data["thrust"] / 4
    )
    rpm2thrust = [reg.intercept_, reg.coef_[0], reg.coef_[1], reg.coef_[2]]
    parameters["rpm2thrust"] = rpm2thrust
    rpms = np.linspace(0, 1.05 * np.max(data["rpm_avg"]), 1000)
    axs[1].plot(rpms, poly(rpms, rpm2thrust, 3), label="fit", color="tab:red")
    axs[1].set_xlabel("Motor RPM")
    axs[1].set_ylabel("Thrust per motor [N]")
    axs[1].legend()
    axs[1].grid()

    axs[0].title.set_text("Motor voltage to RPM and RPM to thrust")
    plt.tight_layout()
    plt.show()

    print(
        f"Thrust = {reg.intercept_} + {reg.coef_[0]}*RPM + {reg.coef_[1]}*RPM^2 + {reg.coef_[2]}*RPM^3"
    )

    print("\n#########################################################")
    print("#### V_motors [V] to RPM and RPM to torque [Nm] curve")
    print("#########################################################")
    fig, axs = plt.subplots(2, 1, figsize=FIGSIZE)

    # As above
    axs[0].scatter(data["vmotors"], data["rpm_avg"], label="training data")
    if len(validations) > 0:
        axs[0].scatter(data_val["vmotors"], data_val["rpm_avg"], label="training data")
    vmotors = np.linspace(0.95 * VMOTOR_MIN, 1.05 * VMOTOR_MAX, 1000)
    axs[0].plot(vmotors, poly(vmotors, vmotor2rpm, 1), label="fit", color="tab:red")
    axs[0].set_xlabel("Motor voltage [V]")
    axs[0].set_ylabel("Motor RPM")
    axs[0].legend()
    axs[0].grid()

    axs[1].scatter(data["rpm_avg"], data["torque_z"] / 4, label="training data")
    if len(validations) > 0:
        axs[1].scatter(
            data_val["rpm_avg"], data_val["torque_z"] / 4, label="validation data"
        )
    X = np.vstack((data["rpm_avg"], data["rpm_avg"] ** 2, data["rpm_avg"] ** 3)).T
    reg = LinearRegression(fit_intercept=False, positive=True).fit(
        X, data["torque_z"] / 4
    )
    rpm2torque = [reg.intercept_, reg.coef_[0], reg.coef_[1], reg.coef_[2]]
    parameters["rpm2torque"] = rpm2torque
    rpms = np.linspace(0, 1.05 * np.max(data["rpm_avg"]), 1000)
    axs[1].plot(rpms, poly(rpms, rpm2torque, 3), label="fit", color="tab:red")
    axs[1].set_xlabel("Motor RPM")
    axs[1].set_ylabel("Torque per motor [Nm]")
    axs[1].legend()
    axs[1].grid()

    axs[0].title.set_text("Motor voltage to RPM and RPM to torque")
    plt.tight_layout()
    plt.show()

    print(
        f"Torque = {reg.intercept_} + {reg.coef_[0]}*RPM + {reg.coef_[1]}*RPM^2 + {reg.coef_[2]}*RPM^3"
    )

    print("\n#########################################################")
    print("#### Thrust [N] to torque [Nm] curve")
    print("#########################################################")
    plt.figure(figsize=FIGSIZE)
    plt.scatter(data["thrust"], data["torque_z"], label="training data")

    if len(validations) > 0:
        plt.scatter(data_val["thrust"], data_val["torque_z"], label="validation data")

    rpms = np.linspace(np.min(data["rpm_avg"]), np.max(data["rpm_avg"]), 1000)
    plt.plot(
        poly(rpms, rpm2thrust, 2) * 4,
        poly(rpms, rpm2torque, 2) * 4,
        label="rpm2torque/rpm2thrust",
    )
    vmotors = np.linspace(np.min(data["vmotors"]), np.max(data["vmotors"]), 1000)
    plt.plot(
        poly(vmotors, vmotor2thrust, 3) * 4,
        poly(vmotors, vmotor2torque, 3) * 4,
        label="vmotors2torque/vmotors2thrust",
    )

    # fit
    X = np.vstack((data["thrust"]))
    Y = data["torque_z"]
    reg = LinearRegression(fit_intercept=False).fit(X, Y)
    thrust2torque = reg.coef_[0]
    parameters["thrust2torque"] = thrust2torque
    print(f"Torque = {thrust2torque:.6f}*Thrust")

    X = np.linspace(0.0, 1.05 * np.max(data["thrust"]) / 4, 1000)
    X_fit = np.vstack((X))
    Y_fit = reg.predict(X_fit)
    plt.plot(X, Y_fit, label="fit", color="tab:red")

    plt.xlabel("Total Thrust [N]")
    plt.ylabel("Total Torque (z) [Nm]")
    plt.title("Thrust to torque")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    ### power -> thrust (+ efficiency)
    print("\n#########################################################")
    print("#### Efficiency")
    print("#########################################################")
    plt.figure(figsize=FIGSIZE)
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
    plt.plot(X, Y_fit, label="fit", color="tab:red")

    plt.xlabel("Total Power [W]")
    plt.ylabel("Total Thrust [N]")
    plt.title("Efficiency")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    # Store parameters
    filename = f"parameters_{comb}"
    with open(filename + ".toml", "w") as f:
        dump(convert_parameters(parameters), f)


def system_id_verification(filenames, validations=[]):
    data = loadFiles(filenames)
    if len(validations) > 0:
        data_val = loadFiles(validations)

    ### 3D Plot
    fig = plt.figure(figsize=FIGSIZE)
    ax = fig.add_subplot(projection="3d")

    ax.scatter(data["cmd"], data["vbat"], data["thrust"] / 4, label="compensated")
    if len(validations) > 0:
        ax.scatter(
            data_val["cmd"],
            data_val["vbat"],
            data_val["thrust"] / 4,
            label="compensated old",
        )

    # plane for verification
    X = np.linspace(0, PWM_MAX, 10)  # PWM
    Y = np.linspace(3.0, 4.2, 10)  # VBat
    X, Y = np.meshgrid(X, Y)
    Z = X / PWM_MAX * parameters["THRUST_MAX"]
    ax.plot_surface(X, Y, Z, alpha=0.2, label="ideal compensation")

    ax.set_xlabel("Thrust command in PWM")
    ax.set_ylabel("Battery voltage [V]")
    ax.set_zlabel("Actual thrust [N]")
    ax.set_title("Thrust Verification")
    ax.legend()
    ax.view_init(elev=30, azim=-100, roll=0)
    plt.tight_layout()
    plt.show()

    ### 2D Plot
    plt.figure(figsize=FIGSIZE)
    X = np.linspace(0, PWM_MAX, 10)  # PWM
    Y = X / PWM_MAX * parameters["THRUST_MAX"]
    plt.plot(X, Y, label="ideal compensation", color="tab:red")
    plt.scatter(data["cmd"], data["thrust"] / 4, label="compensated")
    if len(validations) > 0:
        Y_OLD = X / PWM_MAX * (0.06 * 9.81 / 4)
        plt.plot(X, Y_OLD, "--", label="ideal compensation old")
        plt.scatter(data_val["cmd"], data_val["thrust"] / 4, label="compensated old")
    plt.xlabel("Thrust command in PWM")
    plt.ylabel("Thrust per motor [N]")
    plt.title("Thrust Verification")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    points_plane = data["cmd"] / PWM_MAX * parameters["THRUST_MAX"]
    errors = points_plane - data["thrust"] / 4
    print(f"max error = {np.max(np.abs(errors)) * 1000:.4f}mN")
    print(f"mean error = {np.mean(np.abs(errors)) * 1000:.4f}mN")


def system_id_dynamic(filenames, validations=[]):
    data = loadFiles(filenames)
    data = cutData(data, tStart=17.5, tEnd=33)  # cut off the non compensated part

    thrustCMD = data["cmd"] / PWM_MAX * parameters["THRUST_MAX"] * 4
    rpmCMD = inversepoly(thrustCMD / 4, parameters["rpm2thrust"], 3)
    VmotorCMD = np.zeros_like(data["cmd"])
    for i in range(len(data["cmd"])):
        if thrustCMD[i] < parameters["THRUST_MIN"]:
            VmotorCMD[i] = 0
        else:
            VmotorCMD[i] = inversepoly(thrustCMD[i] / 4, parameters["vmotor2thrust"], 3)

    Vmotor = data["vbat"] * data["pwm"] / PWM_MAX

    ### Fit dynamics
    # There are two possible ways to calculate the thrust dynamics:
    # The first one is to assume the motor speed as a first order system, which is
    # physically correct, if we ignore the drag, viscous part, and DC motor electrical
    # dynamics. The equation is
    # rpm_dot = 1/tau_rpm * (rpm_cmd - rpm) - k_visc * rpm - k_drag * rpm^2,
    # which is simplified to rpm_dot = 1/tau_rpm * (rpm_cmd - rpm).
    # Since we have 4 motors, we average the rpms and ramp them all at once. However,
    # due to the suboptimal battery, the dynamics will be different than ramping a
    # single motor. Still, we believe this is the better way since we want higher
    # accuracy when accelerating all motors at once.
    # Alternatively, we can assume the thrust itself to be a first order system:
    # thrust_dot = 1/tau_f * (thrust_cmd - thrust)

    def first_order_system(x, u, tau, dt, kv=0, kd=0):
        x_dot = 1 / tau * (u - x) + kv * x + kd * x**2
        return x + x_dot * dt

    def residuals(params, x, u, t):
        y = [x[0]]
        dts = np.diff(t)
        if len(params) == 1:
            params = [params[0], 0, 0]
        for i, dt in enumerate(dts):
            y.append(
                first_order_system(y[-1], u[i], params[0], dt, params[1], params[2])
            )
        return np.linalg.norm(x - y, axis=-1)

    res = least_squares(
        residuals,
        # x0=[1.0, 0.0, 0.0],
        # bounds=([1e-3, -1e-3, -1e-6], [1e3, 1e-3, 1e-6]),
        x0=[1.0],
        bounds=([1e-3], [1e3]),
        args=(data["rpm_avg"], rpmCMD, data["time"]),
        method="trf",
        xtol=1e-10,
        verbose=False,
    )
    tau_rpm = res.x[0]

    res = least_squares(
        residuals,
        x0=[1.0],
        bounds=(1e-3, 1e3),
        args=(data["thrust"], thrustCMD, data["time"]),
        method="trf",
        xtol=1e-10,
        verbose=False,
    )
    tau_f = res.x[0]

    ### Simulate dynamics
    dts = np.diff(data["time"])
    rpmPred = [rpmCMD[0]]
    thrustPred = [thrustCMD[0]]
    for i, dt in enumerate(dts):
        # rpm = first_order_system(
        #     rpmPred[-1], rpmCMD[i], 1 / rpm_params[1], dt, rpm_params[2], rpm_params[3]
        # )
        rpm = first_order_system(rpmPred[-1], rpmCMD[i], tau_rpm, dt)
        rpmPred.append(rpm)
        thrust = first_order_system(thrustPred[-1], thrustCMD[i], tau_f, dt)
        thrustPred.append(thrust)

    parameters["tau_rpm"] = tau_rpm
    parameters["tau_f"] = tau_f

    plt.figure(figsize=FIGSIZE)
    plt.plot(data["time"], data["thrust"], label="Thrust Measurement")
    plt.plot(data["time"], thrustCMD, label="Thrust CMD")
    plt.plot(
        data["time"],
        thrustPred,
        "--",
        label="Thrust Prediction from thrust dynamics",
    )
    plt.plot(
        data["time"],
        poly(np.array(rpmPred), parameters["rpm2thrust"], 3) * 4,
        "--",
        label="Thrust Prediction from RPM dynamics",
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Total Thrust [N]")
    plt.title("Thrust Delay")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=FIGSIZE)
    plt.plot(data["time"], data["vbat"], label="Vbat")
    plt.plot(data["time"], VmotorCMD, label="Vmotor CMD")
    plt.plot(data["time"], Vmotor, label="Vmotor actual")
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [V]")
    plt.title("Battery Compensation")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=FIGSIZE)
    plt.plot(data["time"], data["pwm"], label="PWM Motors")
    plt.plot(data["time"], data["cmd"], label="PWM CMD")
    plt.xlabel("Time [s]")
    plt.ylabel("PWM")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=FIGSIZE)
    # plt.plot(data["time"], data["rpm1"], label="RPM 1")
    # plt.plot(data["time"], data["rpm2"], label="RPM 2")
    # plt.plot(data["time"], data["rpm3"], label="RPM 3")
    # plt.plot(data["time"], data["rpm4"], label="RPM 4")
    plt.plot(data["time"], data["rpm_avg"], label="RPM Motors")
    plt.plot(data["time"], rpmCMD, label="RPM CMD")
    plt.plot(data["time"], np.array(rpmPred), label="RPM Prediction")
    plt.xlabel("Time [s]")
    plt.ylabel("RPM")
    plt.title("Propeller Speed Curve")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    # Store parameters
    filename = f"parameters_{comb}"
    with open(filename + ".toml", "w") as f:
        dump(convert_parameters(parameters), f)


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
            f"data/data_{mode}_{comb}_00.csv",  # 500 with torque, verif & dynamic with
            f"data/data_{mode}_{comb}_01.csv",  # 500 with torque, verif & dynamic without
            f"data/data_{mode}_{comb}_02.csv",  # 500 with torque, verif & dynamic without
            f"data/data_{mode}_{comb}_03.csv",  # 500 with torque
            # f"data/data_{mode}_{comb}_04.csv",  # 500 with torque
            # f"data/data_{mode}_{comb}_05.csv",  # 500 with torque
            # f"data/data_{mode}_{comb}_06.csv",
            # f"data/data_{mode}_{comb}_07.csv",
            # f"data/data_{mode}_{comb}_12.csv",  # 34 drone with guards
            # f"data/data_{mode}_{comb}_13.csv",  # E7 drone without guards
        ]
        combVal = "B350"
        if extra != "":
            combVal += f"_{extra}"
        filesVal = [
            # f"data/data_{mode}_{combVal}_00.csv",  # 350 with deck
            # f"data/data_{mode}_{combVal}_01.csv",  # 350 with deck
            # f"data/data_{mode}_{combVal}_02.csv",  # 350 with deck
            # f"data/data_{mode}_{combVal}_03.csv",  # 350 with deck
            # f"data/data_{mode}_{combVal}_04.csv",  # 350 without deck
            # f"data/data_{mode}_{combVal}_05.csv",  # 350 without deck
            # f"data/data_{mode}_{combVal}_06.csv",  # 350 without deck
            # f"data/data_{mode}_{combVal}_07.csv",  # 350 without deck
            # f"data/data_{mode}_{combVal}_12.csv",  # 34 drone with guards
            # f"data/data_{mode}_{combVal}_13.csv",  # E7 drone without guards
        ]

    if args.mode == "ramp_motors":
        raise NotImplementedError("For ramp_motors, please use the plotting script.")
    elif args.mode == "static":
        parameters = {}
        system_id_static(files, filesVal)
    elif args.mode == "static_verification":
        filename = f"parameters_{comb}"
        with open(filename + ".toml", "r") as f:
            parameters = load(f)
        system_id_verification(files, filesVal)
    elif args.mode == "dynamic":
        filename = f"parameters_{comb}"
        with open(filename + ".toml", "r") as f:
            parameters = load(f)
        system_id_dynamic(files, filesVal)
    else:
        raise NotImplementedError(
            f"System Identification Type {args.mode} is not implemented."
        )
