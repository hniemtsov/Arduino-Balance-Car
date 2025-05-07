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
 
 Measurement equation: $$z=Hx+v$$ , where $$H$$ is the Identity 2x2 matrix.
