## Filtering Noisy Data: From Motion Models to Approximation Theory 
 📈 The [Wikipedia example](https://en.wikipedia.org/wiki/Kalman_filter#Example_application,_technical) of the Kalman filter derives the state-space equations using physics-based motion formulas for a truck's position and velocity:
 
 $$x(t+dt) = x(t) + v(t)dt + a\frac{dt^2}{2}$$
 
 But what if your data isn't about motion at all?
 
 In many real-world filtering tasks — especially when dealing with noisy signals from arbitrary sensors — the measurements may be completely unrelated to physical motion. So instead of relying on a motion model, I prefer an approach based on Approximation Theory.
 
 Assume your signal is smooth — that is, it has at least a first and second derivative — and is simply corrupted by noise. Under this assumption, we can apply a Taylor series expansion:
 
 $$x(t+dt) = x(t) + x'(t)dt + x''\frac{dt^2}{2} + O(dt^3)$$
 
 👉 As you can see, the structure of the Taylor expansion matches the motion equation exactly — without requiring any physical model.
 
 🔍 For beginners, this perspective can be powerful: even if your data isn’t related to position or velocity, you can still derive Kalman-style state equations just by assuming the signal is smooth.
 
 This reframing makes filtering accessible across domains — from IoT sensor data to biomedical signals to financial trends.

## The Discrete Linear Kalman Filter
 
 The **state vector** is:
 
 $$x_k=\\begin{bmatrix}
 \theta \\ 
 \omega
 \end{bmatrix}\_{t+dt}$$
 
 **The system model** (how the state evolves over time) is:
 
 $$\begin{bmatrix}
 \theta \\ 
 \omega
 \end{bmatrix}\_{t+dt}=\begin{pmatrix}1 & dt \\
 0 & 1\end{pmatrix}\begin{bmatrix}
 \theta \\ 
 \omega
 \end{bmatrix}\_{t}+\begin{bmatrix}
 \frac{1}{2}{dt}^2 \\ 
 dt
 \end{bmatrix}{\alpha}(t)
 $$
 
 or
 
 $$
 \begin{cases}
 \theta(t+dt) = \theta(t) + \omega(t) \cdot dt + \alpha(t) \cdot \frac{{dt}^2}{2}\\
 \omega(t+dt) = \omega(t) + \alpha(t) \cdot dt
 \end{cases}
 $$
 
 where
 
 - $$\theta(t)$$ is the angle.
 - $$\omega(t) = {\theta}'$$ is the anglular velosity
 - $$\alpha(t) = {\theta}''$$ is the acceleration
 
 so that 
 
 $$x_k=Fx_{k-1}+Ga_k$$

where

$$F=
\begin{pmatrix}1 & dt \\
 0 & 1\end{pmatrix}  \quad     G = \begin{pmatrix}\frac{1}{2}{dt}^2 \\
 dt\end{pmatrix}$$
 
 $$Ga_k \sim N(0,Q), \quad Q=GG^T{\sigma}_a^2=\begin{pmatrix}\frac{1}{4}{dt}^4 & \frac{1}{2}{dt}^3 \\
 \frac{1}{2}{dt}^3 & {dt}^2\end{pmatrix}$$

 Q matrix is a process noise covariance matrix, which models the uncertainty or randomness in the system dynamics. It accounts for things like:
- Unmodeled dynamics (e.g., friction, flexing parts)
- External disturbances (e.g., wind, vibration)
- Approximation errors (e.g., linearization)
 
Kalman filtering contains 2 steps: **Predict step** and **Update step** for states and covariance matrixes.
#### 1. Predict step
The predicted (a priori) state estimate, denoted as $$x^-$$, is the state forecasted from the previous time step using the system model, before the current measurement update.

$$x_k^-=Fx_{k-1}^+$$

The predicted covariance estimate is:

$$P_k^- = FP_{k-1}^+F^T + Q_{k-1}$$ 

The 1st term $$FP_{k-1}^+F^T$$ does not change the amount of the uncertainty, it just shifts it between the states.

```cpp
    // Predicted state x(k|k-1) = F*x(k-1|k-1)  
    float Xp1  = Xu1 + dt*Xu2;
    float Xp2 =           Xu2;
 
    // Predicted covariance P(k|k-1) = F*P(k-1|k-1)*F' + G*Q*G'
    // 1st term: F*P(k-1|k-1)*F
    // Pp := P(k|k-1)    <-- predicted cov
    // Pu := P(k-1|k-1)  <-- updated cov
    // Q  := varA := stdA*stdA - variance of angular acceleration
    //        ⌈ 0.5*dt*dt ⌉
    //  G :=  ⌊     dt    ⌋
    float Pp11 = Pu11 + (Pu12+Pu21)*dt + Pu22*dt2;    float Pp12 = Pu12 + Pu22*dt;
    float Pp21 = Pu21 + Pu22*dt;                      float Pp22 = Pu22;                       

    // 2nd term: G*Q*G'
    Pp11 += dt4*0.25*varA;   Pp12 += dt3*0.5*varA;
    Pp21 += dt3*0.5*varA;    Pp22 += dt2*varA;
```
#### 2. Update step

Measurement model (equation): $$z_k=Hx_k+v$$ , where $$H$$ is the Identity 2x2 matrix.
