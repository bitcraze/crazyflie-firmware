import argparse

from toml import load, dump
import numpy as np
from sklearn.linear_model import LinearRegression, RANSACRegressor, Ridge
from scipy.optimize import least_squares
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
    # INFO: We want to achieve the max thrust for all SoC of the battery. Thus,
    # we limit the maximum thrust to a relatively convervative value. For a
    # discussion on that see #1526
    # The min thrust is determined by the min pwm hard coded on the drone by
    # CONFIG_MOTORS_DEFAULT_IDLE_THRUST
    if "L250" in comb or "P250" in comb:
        THRUST_MAX = 0.48 / 4
    elif "T350" in comb or "T500" in comb:
        THRUST_MAX = 0.72 / 4
    elif "B" in comb:
        THRUST_MAX = 0.8 / 4
    else:
        THRUST_MAX = 0.48 / 4
        print(f"WARNING: Comb {comb} not found, setting THRUST_MAX={THRUST_MAX}")
    THRUST_MIN = PWM_MIN / PWM_MAX * THRUST_MAX

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
    # Remove datapoints outside the min and max thrust region
    mask = np.ones_like(data["thrust"], dtype=bool)
    # mask = mask & (data["thrust"] > THRUST_MIN * 4 * 0.6)
    # mask = mask & (data["thrust"] < THRUST_MAX * 4 * 1.5)
    X = data["vmotors"][mask]
    X = np.vstack((X, X**2, X**3)).T
    reg = Ridge(positive=True).fit(
        X, data["thrust"][mask] / 4
    )  # positive=True, Ridge, LinearRegression
    error = reg.predict(X) - (data["thrust"][mask] / 4)
    vmotor2thrust = [reg.intercept_, reg.coef_[0], reg.coef_[1], reg.coef_[2]]
    parameters["vmotor2thrust"] = vmotor2thrust

    VMOTOR_MIN = inversepoly(THRUST_MIN, vmotor2thrust, 3)[0]
    VMOTOR_MAX = inversepoly(THRUST_MAX, vmotor2thrust, 3)[0]

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
    print(f"VMotor_MIN = {VMOTOR_MIN:.4f}V, VMotor_MAX={VMOTOR_MAX:.4f}V")
    print(f"Prediction RMSE = {np.sqrt(np.mean(error**2)) * 1000:.6f}mN")

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
    mask = data["torque_z"] / 4 < 0.001
    X = data["vmotors"][mask]
    X = np.vstack((X, X**2, X**3)).T
    Y = data["torque_z"][mask] / 4
    reg = RANSACRegressor(
        LinearRegression(), residual_threshold=0.00001, stop_probability=0.999
    ).fit(X, Y)
    reg = reg.estimator_
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
    X = np.vstack((data["rpm_avg"], data["rpm_avg"] ** 2)).T
    reg = LinearRegression(fit_intercept=False, positive=False).fit(
        X, data["thrust"] / 4
    )
    error = reg.predict(X) - (data["thrust"] / 4)
    print(f"Prediction RMSE = {np.sqrt(np.mean(error**2)) * 1000:.6f}mN")
    rpm2thrust = [reg.intercept_, reg.coef_[0], reg.coef_[1]]  # , reg.coef_[2]
    parameters["rpm2thrust"] = rpm2thrust
    rpms = np.linspace(0, 1.05 * np.max(data["rpm_avg"]), 1000)
    axs[1].plot(rpms, poly(rpms, rpm2thrust, 2), label="fit", color="tab:red")
    axs[1].set_xlabel("Motor RPM")
    axs[1].set_ylabel("Thrust per motor [N]")
    axs[1].legend()
    axs[1].grid()

    axs[0].title.set_text("Motor voltage to RPM and RPM to thrust")
    plt.tight_layout()
    plt.show()

    print(
        f"Thrust = {reg.intercept_} + {reg.coef_[0]}*RPM + {reg.coef_[1]}*RPM^2"  #  + {reg.coef_[2]}*RPM^3
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
    mask = data["torque_z"] / 4 < 0.001
    X = data["rpm_avg"][mask]
    X = np.vstack((X, X**2)).T
    Y = data["torque_z"][mask] / 4
    reg = RANSACRegressor(
        LinearRegression(fit_intercept=False, positive=True), residual_threshold=0.0001
    ).fit(X, Y)
    reg = reg.estimator_
    error = reg.predict(X) - Y
    print(f"Prediction RMSE = {np.sqrt(np.mean(error**2)) * 1000:.6f}mN")
    rpm2torque = [reg.intercept_, reg.coef_[0], reg.coef_[1]]  # , reg.coef_[2]
    parameters["rpm2torque"] = rpm2torque
    rpms = np.linspace(0, 1.05 * np.max(data["rpm_avg"]), 1000)
    axs[1].plot(rpms, poly(rpms, rpm2torque, 2), label="fit", color="tab:red")
    axs[1].set_xlabel("Motor RPM")
    axs[1].set_ylabel("Torque per motor [Nm]")
    axs[1].legend()
    axs[1].grid()

    axs[0].title.set_text("Motor voltage to RPM and RPM to torque")
    plt.tight_layout()
    plt.show()

    print(
        f"Torque = {reg.intercept_} + {reg.coef_[0]}*RPM + {reg.coef_[1]}*RPM^2"  #  + {reg.coef_[2]}*RPM^3
    )

    print("\n#########################################################")
    print("#### Thrust [N] to torque [Nm] curve")
    print("#########################################################")
    plt.figure(figsize=FIGSIZE)
    plt.scatter(data["thrust"], data["torque_z"], label="training data")

    if len(validations) > 0:
        plt.scatter(data_val["thrust"], data_val["torque_z"], label="validation data")

    # fit
    mask = data["torque_z"] < 0.003
    X = np.vstack((data["thrust"][mask]))
    Y = data["torque_z"][mask]
    reg = RANSACRegressor(
        LinearRegression(fit_intercept=False), residual_threshold=0.001
    ).fit(X, Y)
    reg = reg.estimator_
    thrust2torque = reg.coef_[0]
    parameters["thrust2torque"] = thrust2torque
    print(f"Torque = {thrust2torque:.6f}*Thrust")

    X = np.linspace(0.0, 1.05 * np.max(data["thrust"]), 1000)
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
    # thrust_from_curve = poly(
    #     data["vbat"] * data["pwm"] / PWM_MAX, parameters["vmotor2thrust"], 3
    # )
    # plt.scatter(data["cmd"], thrust_from_curve, label="onboard applied thrust")
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

    mask = data["cmd"] >= PWM_MIN
    points_plane = data["cmd"][mask] / PWM_MAX * parameters["THRUST_MAX"]
    errors = points_plane - data["thrust"][mask] / 4
    print(f"max error = {np.max(np.abs(errors)) * 1000:.4f}mN")
    print(f"mean error = {np.mean(np.abs(errors)) * 1000:.4f}mN")


def system_id_dynamic(filenames, validations=[]):
    data = loadFiles(filenames)
    data = cutData(data, tStart=19.0, tEnd=-1.0)

    thrustCMD = data["cmd"] / PWM_MAX * parameters["THRUST_MAX"] * 4
    rpmCMD = inversepoly(thrustCMD / 4, parameters["rpm2thrust"], 2)
    VmotorCMD = np.zeros_like(data["cmd"])
    for i in range(len(data["cmd"])):
        if thrustCMD[i] < parameters["THRUST_MIN"]:
            VmotorCMD[i] = 0
        else:
            VmotorCMD[i] = inversepoly(thrustCMD[i] / 4, parameters["vmotor2thrust"], 3)

    Vmotor = data["vbat"] * data["pwm"] / PWM_MAX

    ### Fit dynamics
    # There are multiple possible ways to calculate the thrust dynamics. Refer to the
    # readme for further information on the ODEs.
    # Since we have 4 motors, we average the rpms and ramp them all at once. However,
    # due to the suboptimal battery, the dynamics will be different than ramping a
    # single motor. Still, we believe this is the better way since we want higher
    # accuracy when accelerating all motors at once.
    scale = 1e-5  # used to better condition the problem

    def ode_dc_motor(x, u, dt, a, b, c, d):
        # x = state variable, u = input variable/setpoint
        if u > x:
            x_dot = a * (u - x) + b * (u**2 - x**2)
        else:
            x_dot = c * (u - x) + d * (u**2 - x**2)
        return x + x_dot * dt

    def residuals_simple(params, x, u, t):
        y = [x[0]]
        dts = np.diff(t)
        for i, dt in enumerate(dts):
            y.append(ode_dc_motor(y[-1], u[i], dt, params[0], 0.0, params[0], 0.0))
        return x - y

    def residuals_full(params, x, u, t):
        y = [x[0]]
        dts = np.diff(t)
        for i, dt in enumerate(dts):
            y.append(
                ode_dc_motor(
                    y[-1],
                    u[i],
                    dt,
                    params[0],
                    params[1] * scale,
                    params[2],
                    params[3] * scale,
                )
            )
        return x - y

    res_rpm_simple = least_squares(
        residuals_simple,
        x0=[1.0],
        bounds=([1e-3], [1e3]),
        args=(data["rpm_avg"], rpmCMD, data["time"]),
        method="trf",
        ftol=1e-10,
        xtol=1e-10,
        verbose=False,
    )
    rpm_coef_simple = res_rpm_simple.x
    print(f"{rpm_coef_simple=}, cond={np.linalg.cond(res_rpm_simple.jac)}")

    res_rpm = least_squares(
        residuals_full,
        x0=[10, 0.0, 0.0, 1e-3],
        bounds=([0, 0, 0, 0], [1e3, 1e3, 1e3, 1e3]),
        args=(data["rpm_avg"], rpmCMD, data["time"]),
        method="trf",
        ftol=1e-10,
        xtol=1e-10,
        verbose=False,
    )
    rpm_coef = res_rpm.x
    rpm_coef[1] *= scale  # scaling back
    rpm_coef[3] *= scale  # scaling back
    rpm_coef[np.abs(rpm_coef) < 1e-10] = 0.0
    print(f"{rpm_coef=}, cond={np.linalg.cond(res_rpm.jac)}")

    res_thrust = least_squares(
        residuals_simple,
        x0=[1.0],
        bounds=(1e-3, 1e3),
        args=(data["thrust"], thrustCMD, data["time"]),
        method="trf",
        ftol=1e-10,
        xtol=1e-10,
        verbose=False,
    )
    thrust_coef = res_thrust.x

    ### Simulate dynamics
    dts = np.diff(data["time"])
    rpmPred = [rpmCMD[0]]
    rpmPred_simple = [rpmCMD[0]]
    thrustPred = [thrustCMD[0]]
    for i, dt in enumerate(dts):
        rpmPred_simple.append(
            ode_dc_motor(
                rpmPred_simple[-1],
                rpmCMD[i],
                dt,
                rpm_coef_simple[0],
                0.0,
                rpm_coef_simple[0],
                0.0,
            )
        )
        rpmPred.append(
            ode_dc_motor(
                rpmPred[-1],
                rpmCMD[i],
                dt,
                rpm_coef[0],
                rpm_coef[1],
                rpm_coef[2],
                rpm_coef[3],
            )
        )
        thrustPred.append(
            ode_dc_motor(
                thrustPred[-1],
                thrustCMD[i],
                dt,
                thrust_coef[0],
                0.0,
                thrust_coef[0],
                0.0,
            )
        )

    parameters["rpm_coef"] = rpm_coef
    parameters["rpm_coef_simple"] = rpm_coef_simple[0]
    parameters["thrust_coef"] = thrust_coef[0]

    ### Print stats
    error_rpm_simple = rpmPred_simple - data["rpm_avg"]
    error_thrust_rpm_simple = (
        poly(np.array(rpmPred_simple), parameters["rpm2thrust"], 2) * 4 - data["thrust"]
    )
    error_rpm = rpmPred - data["rpm_avg"]
    error_thrust_rpm = (
        poly(np.array(rpmPred), parameters["rpm2thrust"], 2) * 4 - data["thrust"]
    )
    error_thrust_direct = thrustPred - data["thrust"]
    print(f"Speed RMSE (simple) = {np.sqrt(np.mean(error_rpm_simple**2)):.1f}RPM")
    print(f"Speed RMSE = {np.sqrt(np.mean(error_rpm**2)):.1f}RPM")
    print(
        f"Thrust RMSE (based on thrust) = {np.sqrt(np.mean(error_thrust_direct**2) * 1000):.3f}mN"
    )
    print(
        f"Thrust RMSE (based on speed) [simple] = {np.sqrt(np.mean(error_thrust_rpm_simple**2) * 1000):.3f}mN"
    )
    print(
        f"Thrust RMSE (based on speed) = {np.sqrt(np.mean(error_thrust_rpm**2) * 1000):.3f}mN"
    )

    ### Plot
    plt.figure(figsize=FIGSIZE)
    plt.plot(data["time"], data["thrust"], label="Thrust Measurement")
    # plt.plot(
    #     data["time"],
    #     poly(data["rpm_avg"], parameters["rpm2thrust"], 2) * 4,
    #     label="Thrust from RPM",
    # )
    plt.plot(data["time"], thrustCMD, label="Thrust CMD")
    plt.plot(
        data["time"],
        thrustPred,
        "--",
        label="Thrust Prediction from thrust dynamics",
    )
    plt.plot(
        data["time"],
        poly(np.array(rpmPred), parameters["rpm2thrust"], 2) * 4,
        "--",
        label="Thrust Prediction from RPM dynamics",
    )
    plt.plot(
        data["time"],
        poly(np.array(rpmPred_simple), parameters["rpm2thrust"], 2) * 4,
        "--",
        label="Thrust Prediction from RPM dynamics [simple]",
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
    plt.plot(data["time"], data["rpm_avg"], "--", label="RPM Motors")
    plt.plot(data["time"], rpmCMD, ":", label="RPM CMD")
    plt.plot(data["time"], np.array(rpmPred), label="RPM Prediction")
    plt.plot(data["time"], np.array(rpmPred_simple), label="RPM Prediction (simple)")
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
            f"data/data_{mode}_{comb}_00.csv",
            f"data/data_{mode}_{comb}_01.csv",
            f"data/data_{mode}_{comb}_02.csv",
        ]
        combVal = "B350"
        if extra != "":
            combVal += f"_{extra}"
        filesVal = [
            f"data/data_{mode}_{combVal}_00.csv",
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
