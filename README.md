# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program



---

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

## Rubric

### Build
The code compiles in the Ubuntu 16 machine without errors with cmake and make.

### Model

The model used in this project is a kinematic bicycle model. This model is non-linear model as it takes changes of direction of drive. This model however do not consider dynamics such as slip angle and slip ratio. The vehicle states is represented by [`x, y, psi, v`], where as the error states is represented by [`cte, epsi`].
So, basic version of vehicle model is as follow:
```
  x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  v[t+1] = v[t] + a[t] * dt
  cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
  epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

### Step Length and Elapsed duration
   The values chosen for N and dt are:

   ```
   N = 10
   dt = 0.1
   ```

Based on the recommendation from the lecture and simulation results, If N is small we cannot predict the future well and if the N is choosen too large we plan for long future which is redundant. I have tested  with several values  like 6/0.5  where the car wobbles and large values like 20/0.05 resulted in crash. I have choosen the prediction horizon as 1 sec as ideal case and higher values leads to crash.

### Polynomial Fitting and MPC Reprocessing

   Computations performed in vehicle coordinate system and cooridinates of the waypoints in vehicle coordinates are obtained by following equations:
   ```
   X = dX * cos(-psi) - dY * sin (-psi)
   Y = dX * sin(-psi) + dY * con (-psi)
   ```
   where, `dX = (psix[i] - x)`, `dY = (psiy[i] - y)` and `(X, Y)` = coordinates in Vehicle Coordinate System.

   Note that initial position of the car and heading direction are always assumed to be Zero in this frame, thus state of the Car in Vehicle Coordinate system can be represented as:

   ```
   auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);
   const double cte = polynomialeval(coeffs, 0);
   const double epsi = psi - atan(coeffs[1]);

   Eigen::VectorXd state(6);
   state << 0, 0, 0, v, cte, epsi;
   ```

### Model Predictive Control with Latency

   Although in order to take system latency into considerations, I had to compute state values with kinematic model equations. With updated value, I have initialized state.

   ```
   double latency = 100;
   double latency_dt = 1.0 / latency;
   double x1 = v * cos(0) * latency_dt;
   double y1 = v * sin(0) * latency_dt;
   double psi1 = - (v / mpc.Lf) * delta * latency_dt;
   double v1 = v + (a * latency_dt);
   double cte1 = cte + (v * sin(epsi) * latency_dt);
   double epsi1 = epsi - ((v / mpc.Lf) * delta *  latency_dt);

   Eigen::VectorXd state(6);
   state << x1, y1, psi1, v1, cte1, epsi1;
   ```

   ##  Result

   The car finished the lap without leaving the drivable part of road. The video is attached for the reference.