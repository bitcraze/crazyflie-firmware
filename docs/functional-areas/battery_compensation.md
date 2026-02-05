---
title: Battery Compensation
page_id: battery-compensation
---

## Battery Compensation for Crazyflie Platforms

As a LiPo battery discharges, its voltage decreases, which normally reduces the available motor thrust.
The **Battery Compensation** feature ensures that the Crazyflie produces a consistent thrust across the entire battery range.
This improves flight stability and performance, especially during longer sessions or when the battery gets closer to depletion.

**Recommended when:**
- You want consistent thrust and stable flight throughout the battery’s discharge.
- You want to perform normal flight maneuvers or precision flying.

**Consider disabling when:**
- You want to carry a relatively heavy payload for a short period.
- You want to perform spontaneous or aggressive maneuvers.

## Supported Platforms

Battery compensation is currently available for the following platforms:

- **Crazyflie 2.1 Brushless**
- **Crazyflie 2.1+**
- **Crazyflie 2.X with legacy propellers**
- **Crazyflie 2.X with the thrust upgrade kit**


## How to Enable Battery Compensation

The battery compensation feature is configured in `menuconfig` and is enabled by default on our supported platforms. When [modifying the `menuconfig`](/docs/development/kbuild.md), navigate to **Platform configuration** → **Enable battery thrust compensation** to enable/disable it.

## Methodology

This section explains the process of creating this battery compensation feature, as described by Marcel Rath in [this blogpost](https://www.bitcraze.io/2025/10/keeping-thrust-consistent-as-the-battery-drains/).

### System Identification

To design a proper compensation, the understanding of how thrust relates to motor voltage was needed. This meant running a series of experiments on the thrust stand.
The first step was calibrating the loadcell used to measure thrust:

![callibration](/docs/images/battery_compensation/bat_comp_callibration.png)

Once calibrated, thrust at different applied motor voltages was measured.

![static vmotor-thrust](/docs/images/battery_compensation/bat_comp_static_vmotor-thrust.png)

As expected, the thrust can be modeled well by a third-order polynomial in motor voltage.

Mathematically, the relationship comes from two simple facts:

- A DC motor torque is proportional to motor voltage and inversely related to motor speed.
- A propeller’s thrust scales approximately with the square of the rotational speed.

Combining these effects leads to a nonlinear (third-order) relation between motor voltage and thrust.

### Battery Compensation

The main idea is straightforward: instead of assuming the battery voltage is constant, it is explicitly accounted for. The battery voltage is measured and then low-pass filtered to reduce noise. Together with the necessary motor voltage from the curve above, the following equation is solved for the necessary pwm to apply:

$$
pwm = pwm_{\text{max}} \cdot \frac{v_{\text{motor}}}{v_{\text{bat}}}
$$

This corrected motor voltage is then fed into the third-order model to compensate the thrust command. With this compensation, the commanded-to-actual thrust relation is now approximately linear, which is exactly what is wanted. This can be verified by applying thrust commands and comparing them to the actual thrust.

![static verification](/docs/images/battery_compensation/bat_comp_static_verification_2D.png)


### Dynamic Behavior

To obtain a complete parameter set of the motors and propellers, dynamic tests were performed which included commanding rapid increases and decreases in PWM and measuring the thrust response.

![dynamic RPM](/docs/images/battery_compensation/bat_comp_dynamic_RPM.png)

These dynamics are not required for the battery compensation itself, but they are very useful for accurate system identification and for simulation purposes.