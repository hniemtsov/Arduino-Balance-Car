## The Continuous-Time Kalman Filter (Kalman-Bucy) Equation

The **state vector** is:

$$\begin{bmatrix}
\theta \\ 
q_{bias}
\end{bmatrix}$$

**The system model** (how the state evolves over time) is:

$$\frac{d}{dt}\begin{bmatrix}
\theta \\ 
q_{bias}
\end{bmatrix}=A\begin{bmatrix}
\theta \\ 
q_{bias}
\end{bmatrix}+\begin{bmatrix}
1 \\ 
0
\end{bmatrix}\omega
$$

or

$$
\begin{cases}
\dot{\theta} = \omega - q_{bias} \\
\dot{q}_{bias}= 0
\end{cases}
$$

where

- $$A$$ is the system matrix.

$$A=\begin{pmatrix}0 & -1 \\
0 & 0\end{pmatrix}$$

- $$\theta$$ is the estimated angle
- $$q_{bias}$$ is the estimated gyro bias
- $$\omega$$ is the gyro measurement

- the process noise covariance matrix:

$$Q=\begin{pmatrix}Q_{angle} & 0 \\
0 & Q_{gyro}\end{pmatrix}$$

Measurement equation: $$z=Cx+v$$ , where $$C$$ is the measurement matrix

$$C = \begin{bmatrix}C_0 & 0\end{bmatrix}$$

## Kalman-Bucy Filter Background
For the continuous-time Kalman-Bucy filter (which is a continuous version of the Kalman filter), the update for the covariance matrix $$P$$ can be expressed as:

$$\dot{P}(t)=AP(t)+P(t)A^T+Q$$

- **1st term** $$AP+PA^T$$:

$$AP = \begin{bmatrix}0 & -1 \\
0 & 0\end{bmatrix}\begin{bmatrix}P_{00} & P_{01} \\
P_{10} & P_{11}\end{bmatrix}=\begin{bmatrix}-P_{10} & -P_{11} \\
0 & 0\end{bmatrix}$$
$$PA^T = \begin{bmatrix}P_{00} & P_{01} \\
P_{10} & P_{11}\end{bmatrix}\begin{bmatrix}0 & -1 \\
0 & 0\end{bmatrix}=\begin{bmatrix}-P_{01} & 0 \\
-P_{11} & 0\end{bmatrix}$$

- **2nd term**: Process noise $$Q$$

```
Pdot[0] = Q_angle - P[0][1] - P[1][0];
Pdot[1] = - P[1][1];
Pdot[2] = - P[1][1];
Pdot[3] = Q_gyro;
```

1. `Pdot[0]`:
   - `Pdot[0] = Q_angle - P[0][1] - P[1][0];`
   - This corresponds to the change in the top-left element of the covariance matrix. In the continuous case, it would be related to the system's process noise and any dependencies between state variables (like how angle and bias are related). The `Q_angle` term represents the process noise for the angle, and the subtraction of `P[0][1]` and `P[1][0]` represents the influence of the off-diagonal terms (the covariance between the angle and bias).

2. `Pdot[1]` and ` Pdot[2]`:
   - `Pdot[1] = - P[1][1];`
   - `Pdot[2] = - P[1][1];`
   - These terms represent the rate of change of the off-diagonal elements in the covariance matrix. They are influenced by the noise in the gyro (angular velocity), which is described by `Q_gyro`. The negative sign indicates that the covariance between the angle and bias is decreasing due to the system dynamics.

4. `Pdot[3]`:
   - `Pdot[3] = Q_gyro;`
   - This is the change in the bottom-right element of the covariance matrix. This term corresponds to the process noise for the gyro, which is the uncertainty in the gyro's measurements.

## Interpretation in Kalman Filter Equations
In the Kalman filter, we compute the Kalman gain $$K$$ using the measurement uncertainty:

$$E = R + CPC^T$$

$$CPC^T=\begin{pmatrix}C_0 & 0\end{pmatrix}\begin{bmatrix}P_{00} & P_{01} \\
P_{10} & P_{11}\end{bmatrix}\begin{pmatrix}C_0 \\
0\end{pmatrix}=$$
$$=\begin{pmatrix}C_0P_{00} & C_0P_{01}\end{pmatrix}\begin{pmatrix}C_0 \\
0\end{pmatrix}=C_0^2P_{00}$$

,where:
- $$R$$ is the **measurement noise covariance**
- $$C$$ is the **measurement matrix** (or just a scalar $$C_0$$ in our case)
- $$P$$ is the **error covariance matrix**

The code computes:
```
E = R_angle + C_0 * PCt_0;
```
Now, the **Kalman gain K** is computed as: $$K = PC^TE^{-1}$$

$$PC^T=\begin{bmatrix}P_{00} & P_{01} \\
P_{10} & P_{11}\end{bmatrix}\begin{pmatrix}C_0 \\
0\end{pmatrix}=\begin{pmatrix}P_{00}C_0 \\
P_{10}C_0\end{pmatrix}$$

The `code` computes:
```
PCt_0 = C_0 * P[0][0];
PCt_1 = C_0 * P[1][0];
E = R_angle + C_0 * PCt_0;
K_0 = PCt_0 / E;
K_1 = PCt_1 / E;
```

#### Kalman filter covariance update step
The covariance matrix update formula is:

$$P^{\prime}=P-KCP$$

$$KCP=\begin{pmatrix}K_0 \\
K_1\end{pmatrix}\begin{pmatrix}C_0P_{00} & C_0P_{01}\end{pmatrix}=$$
$$=\begin{bmatrix}K_0C_0P_{00} & K_0C_0P_{01} \\
K_1C_0P_{00} & K_1C_0P_{01}\end{bmatrix}$$

The `code` computes:
```
  PCt_0 = C_0 * P[0][0];
...
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
```
