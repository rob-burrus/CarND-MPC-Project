# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Final Results
This model is able to successfully drive around the track with reference velocity set to 100mph (Note: on some runs, it loses control). This result was achieved with the following hyper parameteres: 
* N = 10
* dt = 0.05
* change_steer_weight = 20,000 --> heavier weight minimizes the change in sequential steering actuations
* steer_weight = 50 --> minimizes use of the steering actuator
* all other cost weights = 1
* Initial state passed to the MPC solver is adjust for 100 millisecond latency using the kinematic equations with x, y, psi = 0 (discussion below)


## Model description

In main.cpp, a websocket message is received from the simulator and parsed out into waypoints (ptsx, ptsy), vehicle position (x, y), psi, speed (velocity in mph), steering_angle, and throttle. The waypoints are given and map coordinates and need to be converted into vehicle coordinates for use in the MPC solver 
```
for(int i =0; i < len; i++){ 
   ptsx_vehicle_coords[i] = cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
   ptsy_vehicle_coords[i] = -sin(psi) * (ptsx[i] - px) + cos(psi) * (ptsy[i] - py);
}
 ```
Next, use the converted waypoints to find coeeficients of a 3rd degree polynomial. The cross track error (CTE) is equal to the coefficients evaluated at x=0 (the car is always at the origin in the vehicle coordinate system). The psi error is the negative arctangent of the second coefficent. In the vehicle coordinate system, the vehicle x, y, and psi = 0
```
auto coeffs = polyfit(ptsx_vehicle_coords, ptsy_vehicle_coords, 3);
double cte = polyeval(coeffs, 0);
double epsi = -atan(coeffs[1]);
```
The MPC solver takes the current state and the coefficients as arguments. The state is a vector with 6 elements = x, y, psi, cte, epsi.
Because of the 100 millisecond latency, which causes the effect of the actuations to be delayed, we use the kinematic model equations to adjust the initial state to where we estimate the car will be in 100 milliseconds. Again, because we're using the vehicle coordinate system, the x, y, psi = 0 ... and the kinematic equations simply to the following:
```
Eigen::VectorXd state(6);
px = v * latency;
py = 0;
psi = - v / 2.67 * steer_value * latency;
v = v + throttle_value * latency;
state << px, py, psi, v, cte, epsi;
```
The solver sets the initial state (adjusted for latency), as well as the lower and upper limits for actuator variables. In general, a model predictive controller frames the task of following the trajectory of the waypoints as an optimization problem. The MPC simulates different actuator inputs over a time period (T) in an attempt to minimize a cost. The simulation function uses the kinematic motion model do predict where the car will be given the actuation (steering angle and acceleration). The solution to a MPC is the trajectory with the lowest cost. There are a few hyperparameters to tune:
* The time period is the product of two hyper parameters: N (number of steps) and dt (length of time between each step). In my final solution, I set N = 10 and dt = 0.05. I found that increasing the time period made the car unstable around turns at high speeds. 
* The cost is the sum of several components: cte, epsi, v, steer angle, acceleration, change in steering angle, and change in acceleration. I gave each of these components a weight that scaled their importance to the overall cost. The most important contributor to the cost was the change in steering angle, for which I sclaed by a factor of 20,000. This huge weight heavily penalized the optimization function for making sharp changes in the steering angle. The result was a significantly smoother drive, especially at high speeds.  

After the simulation, the solver returns x,y points for the optimal trajectory, and the first set of actuations (steering angle and acceleration). These values are passed to the simulator to control the car and display the vehicle / waypoint trajectories. When we get the next message from the simulator, the previous actuations and trajectories are thrown out, and we repeat this process with a new initial state. 




## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.


