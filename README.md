# Flight Computer
## State Estimation
### Attitude Estimation
The primary state model is simply $\hat q_t = \hat q_{t-1} + \int^{t}_{t-1}{\hat\omega dt}$.

Where $\hat q_t$ is the state quaternion at time $t$ and $\hat\omega$ is the current angular acceleration as measured by a gyroscope.

However since the angular accleration is not measured by the sensor as a quaternion, we have to convert it.

We used the Euler-Rodrigues rotation formula, which allows the state model to become
$$\hat q_t = \left[\cos\left(\frac{\left|\omega\right| \Delta t}{2}\right)\mathbf{I_4} +
\frac{2}{\left| \omega \right|}\sin\left(\frac{\left|\omega\right| \Delta t}{2}\right)\boldsymbol{\Omega_t}\right]\hat q_{t-1}$$

The inital estimation and the correction are calculated using the Super-fast Attitude from Accelerometer and Magnetometer (SAAM).
