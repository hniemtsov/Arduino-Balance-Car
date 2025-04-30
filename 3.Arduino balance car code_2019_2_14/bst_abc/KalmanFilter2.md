The [Wikipedia example](https://en.wikipedia.org/wiki/Kalman_filter#Example_application,_technical) derives the state-space equations using physics-based formulas for the position $$x$$ and velocity $$v$$ of a truck.

$$x(t+dt)=x(t)+v(t)*dt$$
 However, I prefer an approach based on **Approximation Theory**, which assumes that the signal is smooth and differentiable (at least once), but corrupted by noise. Under this assumption, we can apply a **Taylor series expansion** to derive equations for the state vector $$y$$ and its derivative $$y'$$.

 The [Wikipedia example](https://en.wikipedia.org/wiki/Kalman_filter#Example_application,_technical) derives the state-space equations using physical motion equations for a truck’s position $$x(t)$$ and velocity $$v(t)$$. 

However, in practical filtering tasks—especially when dealing with noisy sensor data that may be distant or unrelated to any physical model—I prefer an approach grounded in **Approximation Theory**. Specifically, I assume that the underlying signal I want to recover is inherently **smooth**, meaning it has at least first and second derivatives.

Under this assumption, a **Taylor series expansion** naturally leads to the same state-space equations, without relying on physical modeling, but instead on the assumption of continuity and smoothness in the data.

For example, expanding the signal \( y(t) \) in Taylor series gives:

\[
y(t + \Delta t) = y(t) + y'(t) \Delta t + \frac{1}{2} y''(t) \Delta t^2 + \mathcal{O}(\Delta t^3)
\]

which closely resembles the same equations used in the constant-velocity model of the Kalman filter.

The [Wikipedia example](https://en.wikipedia.org/wiki/Kalman_filter#Example_application,_technical) derives the state-space equations using physical motion equations for a truck’s position and velocity. 

$$x(t+dt) = x(t) + v(t)dt + a\frac{dt^2}{2}$$

However, in practical filtering tasks — especially when dealing with noisy sensor data that may be unrelated to motion equation — I prefer an approach grounded in **Approximation Theory**. Specifically, I assume that the underlying signal I want to recover is inherently **smooth**, meaning it has at least 1st and 2nd derivatives. Under this assumption, a **Taylor series expansion** naturally leads to the same state-space equations, without relying on physical modeling, but instead on the assumption of continuity and smoothness in the data.

That said, for beginners, it may not be clear how to derive these equations to their own data — particularly when the data is unrelated to physical concepts like position or velocity. In many practical applications, such as filtering noisy signals from arbitrary sensors, the measurements may come from entirely different domains. In these cases, it's useful to think of the signal simply as a smooth function corrupted by noise. This perspective allows the same derivation using Taylor series, independent of the specific nature of the signal.

Compare Teylor series with motion equation above:

$$x(t+dt) = x(t) + x'(t)dt + x''\frac{dt^2}{2} + O(dt^3)$$


