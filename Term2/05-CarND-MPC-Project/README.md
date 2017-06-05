# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Discussions

### Model

#### State

* x: current location in X axis
* y: current location in Y axis
* psi: current angle
* v: current velocity
* cte: current cross track error
* epsi: current psi error

#### Actuators

* delta: current steering angle ranging from -1 to 1
* a: current acceleration ranging from

### Equations

````
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
````

The update equation used follows the global kinematic model. Using this equation, they minimize the cte to obtain minimum cost of steering angle (delta) and acceleration (a).

### Timestep and Duration

N defines the number of timesteps and dt defines how much time elapses between each actuation.
I started with N = 25 and dt = 0.05. But, the car steer too much and often it drove out of the road before turning back.
I reduces the number of timesteps and prolonged the duration to N to 15 and dt to 0.5. I tried several values, as long as the N is kept under 15 and dt below 0.9, the car is able to complete the course.
Since the simulator has the latency of 100ms, with N=15 and dt=0.5, it provides 7.5s prediction which is sufficient to compensate the 100ms lag.
Besides, I reduced the limit of lower and upper acceleration to 0.5 so that the car is able to not accelerate too much in case it encounters a corner.

### Polynomial Fitting and Preprocessing

Preprocessing is done by converting from global coordinate to vehicle coordinate. (Line 105 at main.cpp)
Then, the coefficients are obtained by fitting the 3rd order polynomials to the waypoints.
CTE and EPSI are obtained from the coefficients.
The mpc result is calculated from the state of the car and the coefficients.


### Latency

Latency is adjusted at Line 68 at MPC.cpp.
