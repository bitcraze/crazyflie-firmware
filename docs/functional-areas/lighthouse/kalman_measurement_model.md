---
title: Lighthouse kalman measurment model
page_id: lh_measurement_model
---

This page describes the generalized measurement model used for the lighthouse in the kalman state estimator.

In the measurement model we want to get from a sensor position $$\vec{s}$$ to rotation angle $$\alpha$$.
The first step is to calculate the sensor position in the rotor reference frame.

Use a rotation matrix $$R_r$$ to go from the base station reference frame to the rotor reference frame.
For LH2 and the horizontal rotor in LH1 this is the unit matrix, while the vertical drum in LH1 gets
$$R_{vert} = \left[\begin{array}{ccc}
1 & 0 & 0 \\
0 & 0 & 1 \\
0 & -1 & 0
\end{array}\right]$$

The sensor has position $$\vec{s_{cf}}$$ in the CF reference frame and
$$\vec{s} = \vec{p_{cf}} + R_{cf} \cdot \vec{s_{cf}}$$ in the global reference frame. The sensor position
in the base station reference frame is $$s_{bs} = R_{bs}^{-1} \cdot (\vec{s} - \vec{p_{bs}}) =
R_{bs}^{-1} \cdot (\vec{p_{cf}} - \vec{p_{bs}} + R_{cf} \cdot \vec{s_{cf}})$$

Finally, the sensor position in the rotor reference frame is $$\vec{s_r} =
R_r \cdot R_{bs}^{-1} \cdot (\vec{p_{cf}} - \vec{p_{bs}} + R_{cf} \cdot \vec{s_{cf}})$$

## Measurement

The measurement is the rotation angle $$\alpha$$ when the sensor is hit by the light plane.

## Prediction

To calculate the predicted rotation angle $$\alpha_p$$ we have to go from the sensor position
($$s_r = (x, y, z)$$ in the rotor reference frame) to rotation angle, where the rotation angle is from
the X-axis to the line where the light plane intersects the XY-plane. The rotation angle to the sensor
$$\alpha_s$$ is the sum of the predicted rotation angle $$\alpha_p$$ and the rotation angle from the
intersection line to the sensor $$\alpha_t$$, caused by the tilt of the light plane,
$$\alpha_s = \alpha_p + \alpha_t$$

![Prediction geometry](/docs/images/lighthouse/prediction_geometry.png)

The rotation angle to the sensor $$\alpha_s$$ is defined by

$$\tan \alpha_s = \frac{y}{x}$$

$$\alpha_s = \tan^{-1}(\frac{y}{x})$$

To calculate $$\alpha_t$$ we first have to look at the sensor position projected on the XY-plane
$$(x, y, 0)$$. The radius to this point is $$r = \sqrt{x^2 + y^2}$$

We also need the distance $$d$$ from the intersection line to the sensor, perpendicular to the
intersection line, $$d=r\sin \alpha_t$$.

$$d$$ can also be calculated using the tilt and z, $$d=z\tan -t$$. If we combine these

$$r\sin \alpha_t = z\tan -t$$

$$\sin \alpha_t = \frac{z\tan -t}{r} = -\frac{z\tan t}{\sqrt{x^2 + y^2}}$$

$$\alpha_t = \sin^{-1}(-\frac{z\tan t}{\sqrt{x^2 + y^2}}) = -\sin^{-1}(\frac{z\tan t}{\sqrt{x^2 + y^2}})$$

Finally we can calculate the predicted rotation angle

$$\alpha_p = \alpha_s - \alpha_t = \tan^{-1}(\frac{y}{x}) + \sin^{-1}(\frac{z\tan t}{\sqrt{x^2 + y^2}})$$

## H vector

Calculate the position elements of the H vector in the rotor reference frame

$$\vec{g_r} = \begin{pmatrix}
\frac{d\alpha_p}{dx} & \frac{d\alpha_p}{dy} & \frac{d\alpha_p}{dz}
\end{pmatrix}$$

$$\vec{g_r} = \begin{bmatrix}
\frac{-y}{x^2 + y^2} - \frac{xz\tan t}{(x^2+y^2)^{\frac{3}{2}} \sqrt{1-\frac{(z\tan t)^2}{x^2+y^2}} } &
\frac{x}{x^2 + y^2} - \frac{yz\tan t}{(x^2+y^2)^{\frac{3}{2}} \sqrt{1-\frac{(z\tan t)^2}{x^2+y^2}} } &
\frac{\tan t}{\sqrt{x^2+y^2} \sqrt{1-\frac{(z\tan t)^2}{x^2+y^2}} }
\end{bmatrix}$$

Let $$r=\sqrt{x^2+y^2}$$ and $$Q=\frac{\tan t}{\sqrt{x^2+y^2} \sqrt{1-\frac{(z\tan t)^2}{x^2+y^2}}} =
\frac{\tan t}{r \sqrt{1-\frac{(z\tan t)^2}{r^2}}} = \frac{\tan t}{\sqrt{r^2-(z\tan t)^2}} $$

Which leads to

$$\vec{g_r} = \begin{bmatrix}
\frac{-y-xzQ}{x^2 + y^2} &
\frac{x-yzQ}{x^2 + y^2} &
Q
\end{bmatrix}$$

$$\vec{g_r} = \begin{bmatrix}
\frac{-y-xzQ}{r^2} &
\frac{x-yzQ}{r^2} &
Q
\end{bmatrix}$$

Rotate the position elements to the global reference frame to be used in the kalman filter

$$\vec{g}=(R_r \cdot R_{bs}^{-1})^{-1} \cdot \vec{g_r} = R_{bs} \cdot R_r^{-1} \cdot \vec{g_r}$$

Finally we have the H vector

$$H=(g_x, g_y, g_z, 0, 0, 0...)$$

## Optimizations

For rotation matrices the following is true

$$R^{-1} = R^T$$

$$R_1 \cdot (R_2 \cdot \vec{v}) = (R_1 \cdot R_2) \cdot \vec{v}$$
