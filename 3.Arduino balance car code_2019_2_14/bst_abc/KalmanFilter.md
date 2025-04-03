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

where

- $$A$$ is the system matrix.

$$A=\begin{pmatrix}0 & -1 \\
0 & 0\end{pmatrix}$$

- $$\theta$$ is the estimated angle
- $$q_{bias}$$ is the estimated gyro bias
- $$\omega$$ is the gyro measurement

$$
\begin{cases}
\dot{\theta} = \omega - q_{bias} \\
\dot{q}_{bias}= 0
\end{cases}
$$

- the process noise covariance matrix:

$$Q=\begin{pmatrix}Q_{angle} & 0 \\
0 & Q_{gyro}\end{pmatrix}$$

Measurement equation: $$z=Cx+v$$ , where $$C$$ is the measurement matrix

$$C = \begin{bmatrix}C_0 & 0\end{bmatrix}$$

#### Kalman-Bucy Filter Background
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

