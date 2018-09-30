# Self-Driving Car Model Predictive Controller (MPC)
---

## This is the fifth project of term 2 of self-driving cars engineer nanodegree.
In this project we will In this project we will revisit the lake race track from the Behavioral Cloning Project. This time, however, we'll implement a Model Predictive Controller in C++ to maneuver the vehicle around the track!

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

---

## The Model

Firts, I get the next variables from the simulator:

```cpp
vector<double> ptsx = j[1]["ptsx"]; // x position of waypoint ahead
vector<double> ptsy = j[1]["ptsy"]; // y position of waypoint ahead
double px = j[1]["x"]; // current vehicle x position
double py = j[1]["y"]; // current vehicle y position
double psi = j[1]["psi"]; // current vehicle orientation angle
double v = j[1]["speed"]; // current vehicle velocity
```

I'm using a kinematic model without taking into account the tires:

```cpp
x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v_[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

where:

- `cte` : Cross-track error.
- `epsi` : Orientation error.

the main objective is to find the `throttle_value` and the `steering_angle` to minimize these errors.

So, first I de choose `N` and `dt` to define the duration of the trajectory:

```cpp
size_t N = 10;
double dt = 0.1;
```

Next, I defined the `vehicle model` and `constraints` such as actual limitations:

```cpp
// The upper and lower limits of delta are set to -25 to 25
  // degrees (values in radians).
  for ( int i = delta_start; i < a_start; i++ ) {
    vars_lowerbound[i] = -0.436332*Lf;
    vars_upperbound[i] = 0.43632*Lf;
  }

  // Actuator limits.
  for ( int i = a_start; i < n_vars; i++ ) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
```

Finally, I defined a cost function:

```cpp
fg[0] = 0;

// The part of the cost based on the reference state.
for (int t = 0; t < N; t++) {
  fg[0] += 2000*CppAD::pow(vars[cte_start + t] - ref_cte, 2);
  fg[0] += 2000*CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
  fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Minimize the use of actuators.
for (int t = 0; t < N - 1; t++) {
  fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);
  fg[0] += 5*CppAD::pow(vars[a_start + t], 2);
}

// Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) {
  fg[0] += 200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

## Timestep Length and Elapsed Duration (N & dt)

`N` is the number of timesteps in the horizon. `dt` is how much time elapses between actuations. `N`, `dt` are hyperparameters we will need to tune for each model predictive controller. As we work with finite resources, if we set `N` to a large value, we will have performance issue, the same happens with a really small `dt`. After trying with several values, I decided to set `N = 10` and  `dt = 0.1` which give a better result.

## Polynomial Fitting and MPC Preprocessing

First, the waypoints coordinates are transformed to the vehicle's coordinate system. Then, they fit a 3rd order polynomial. 

```cpp
size_t n_waypoints = ptsx.size();
for (unsigned int i = 0; i < n_waypoints; i++ ) {
  double shift_x = ptsx[i] - px;
  double shift_y = ptsy[i] - py;
  ptsx[i] = shift_x * cos( 0.0 - psi ) - shift_y * sin( 0.0 - psi );
  ptsy[i] = shift_x * sin( 0.0 - psi ) + shift_y * cos( 0.0 - psi );
}

double* ptrx = &ptsx[0];
Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, n_waypoints);

double* ptry = &ptsy[0];
Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, n_waypoints);

// Fit polynomial to the points - 3rd order.
auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
```

with this polynomial, I calculate `cte` and `epsi`:

```cpp
double cte = polyeval(coeffs, 0);
double epsi = -atan(coeffs[1]);
```

## Model Predictive Control with Latency

To handle the contribution factor of actuator dynamics to latency. I applied a delay to teh state variables:

```cpp
// Initial state.
const double x0 = 0;
const double y0 = 0;
const double psi0 = 0;
const double cte0 = coeffs[0];
const double epsi0 = -atan(coeffs[1]);

// State after delay.
double x_delay = x0 + ( v * cos(psi0) * delay );
double y_delay = y0 + ( v * sin(psi0) * delay );
double psi_delay = psi0 - ( v * steer_value * delay / Lf );
double v_delay = v + throttle_value * delay;
double cte_delay = cte0 + ( v * sin(epsi0) * delay );
double epsi_delay = epsi0 - ( v * atan(coeffs[1]) * delay / Lf );

// Define the state vector.
Eigen::VectorXd state(6);
state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;
```