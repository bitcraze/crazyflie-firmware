---
title: Lighthouse angle conversion
page_id: lh_angle_conversion
---

One way to get started with lighthouse 2 is to create a conversion from lighthouse 2 angles to lighthouse 1 angles,
and use the functionality that has already been implemented for lighthouse 1. Even though it is an easy way to get started, the main drawback is that we need both sweeps for the conversion. Lighthouse 1 angles are also easier to
understand and can be a useful debugging tool.

## Lighthouse 1 to lighthouse 2

Start from the [measurement model for the kalman filter](./kalman_measurement_model.md).

For a point $$(x, y, z)$$, the sweep angles are

$$
\begin{cases}
\alpha^{lh2}_1 = \tan^{-1}(\frac{y}{x}) + \sin^{-1}(\frac{z\tan(-t)}{\sqrt{x^2 + y^2}})\\
\alpha^{lh2}_2 = \tan^{-1}(\frac{y}{x}) + \sin^{-1}(\frac{z\tan(t)}{\sqrt{x^2 + y^2}})
\end{cases}
$$

where $$t = \frac{\pi}{6}$$

$$x$$, $$y$$ and $$z$$ are given by the lighthouse 1 sweep angles


$$
\begin{cases}
x = 1 \\
y = \tan(\alpha^{lh1}_1) \\
z = \tan(\alpha^{lh1}_2)
\end{cases}
$$

this leads to

$$
\begin{cases}
\alpha^{lh2}_1 = \alpha^{lh1}_1 + \sin^{-1}(\frac{\tan(\alpha^{lh1}_2)\tan(-t)}{\sqrt{1 + \tan^2(\alpha^{lh1}_1)}})\\
\alpha^{lh2}_2 = \alpha^{lh1}_1 + \sin^{-1}(\frac{\tan(\alpha^{lh1}_2)\tan(t)}{\sqrt{1 + \tan^2(\alpha^{lh1}_1)}})\\
\end{cases}
$$

or

$$
\begin{cases}
\alpha^{lh2}_1 = \alpha^{lh1}_1 + \sin^{-1}(q\tan(-t))\\
\alpha^{lh2}_2 = \alpha^{lh1}_1 + \sin^{-1}(q\tan(t))\\
\end{cases}
$$

where

$$q = \frac{\tan(\alpha^{lh1}_2)}{\sqrt{1 + \tan^2(\alpha^{lh1}_1)}}$$



## Lighthouse 2 to lighthouse 1

The strategy when creating an equation to convert from lighthouse 2 sweep angles to lighthouse 1, is to find the intersection line between the two light planes. From the intersection line we can easily calculate the lighthouse 1 sweep angles.

The following calculations are all in the base station reference frame.

Assume we know the normals for the two light planes, $$\vec{n_1}$$ and $$\vec{n_2}$$. The cross product of the normals $$\vec{v} = \vec{n_1} \times \vec{n_2}$$
gives us the direction of the intersection line. We also know it is passing through the origin as we are using the base station
reference frame.

### Normals of the light planes

Start by figuring out the normals when $$\alpha=0$$, and then rotate them around the Z-axis using a rotation matrix.

The rotation matrix is

$$R_{z} = \left[\begin{array}{ccc}
cos{\alpha} & -sin{\alpha} & 0 \\
sin{\alpha} & cos{\alpha} & 0 \\
0 & 0 & 1
\end{array}\right]$$

and the resulting normals

$$
\begin{cases}
\vec{n1}=R_z \cdot \begin{bmatrix}0 & -cos{(t)} & sin{(t)}\end{bmatrix} = \begin{bmatrix}cos{(t)}sin{(\alpha^{lh2}_1)} & -cos{(t)}cos{(\alpha^{lh2}_1)} & sin{(t)}\end{bmatrix} \\
\vec{n2}=R_z \cdot \begin{bmatrix}0 & -cos{(t)} & -sin{(t)}\end{bmatrix} = \begin{bmatrix}cos{(t)}sin{(\alpha^{lh2}_2)} & -cos{(t)}cos{(\alpha^{lh2}_2)} & -sin{(t)}\end{bmatrix}
\end{cases}
$$

where $$t=\pi/6$$ is the tilt angle of the light planes.


### The intersection vector

$$\vec{v} = \vec{n_1} \times \vec{n_2} = \begin{bmatrix}-\sin{(t)}\cos{(t)}(\cos{(\alpha^{lh2}_1)} + \cos{(\alpha^{lh2}_2)}) & -\sin{(t)}\cos{(t)}(\sin{(\alpha^{lh2}_1)} + \sin{(\alpha^{lh2}_2)}) & \cos^2{(t)}(\sin{(\alpha^{lh2}_1)}\cos{(\alpha^{lh2}_2)}-\cos{(\alpha^{lh2}_1)}\sin{(\alpha^{lh2}_2)})\end{bmatrix}$$

### Lighthouse 1 angles

Finally we can calculate the lighthouse 1 angles

$$
\begin{cases}
\alpha^{lh1}_1 = \tan^{-1}(\frac{v_2}{v_1})\\
\alpha^{lh1}_2 = \tan^{-1}(\frac{v_3}{v_1})
\end{cases}
$$

$$
\begin{cases}
\alpha^{lh1}_1 = \tan^{-1}(\frac{-\sin{(t)}\cos{(t)}(\sin{(\alpha^{lh2}_1)} + \sin{(\alpha^{lh2}_2)})}{-\sin{(t)}\cos{(t)}(\cos{(\alpha^{lh2}_1)} + \cos{(\alpha^{lh2}_2)})})\\

\alpha^{lh1}_2 =
\tan^{-1}(\frac{\cos^2{(t)}(\sin{(\alpha^{lh2}_1)}\cos{(\alpha^{lh2}_2)}-\cos{(\alpha^{lh2}_1)}\sin{(\alpha^{lh2}_2)})}{-\sin{(t)}\cos{(t)}(\cos{(\alpha^{lh2}_1)} + \cos{(\alpha^{lh2}_2)})})
\end{cases}
$$

$$
\begin{cases}
\alpha^{lh1}_1 = \frac{\alpha^{lh2}_1 + \alpha^{lh2}_2}{2}\\

\alpha^{lh1}_2 = \tan^{-1}(\frac{\sin{(\alpha^{lh2}_2 - \alpha^{lh2}_1)}}{\tan{(t)} (\cos{(\alpha^{lh2}_1)} + \cos{(\alpha^{lh2}_2)})})\end{cases}
$$
