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
