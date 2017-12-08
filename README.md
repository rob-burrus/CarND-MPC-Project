# Model Predictive Control

<a href="https://imgflip.com/gif/20qvej"><img src="https://i.imgflip.com/20qvej.gif" title="made at imgflip.com"/></a><a href="https://imgflip.com/gif/20qvhp"><img src="https://i.imgflip.com/20qvhp.gif" title="made at imgflip.com"/></a>
<a href="https://imgflip.com/gif/20qvje"><img src="https://i.imgflip.com/20qvje.gif" title="made at imgflip.com"/></a><a href="https://imgflip.com/gif/20qvmw"><img src="https://i.imgflip.com/20qvmw.gif" title="made at imgflip.com"/></a>


## Overview
Model Predictive Control (MPC) frames the task of following a trajectory as an optimization problem. MPC involves simulating different actuator inputs, predicting the resultant trajectory, and selecting the trajectory with the minimum cost. Because the model is only approximate,  only the first actuations for the trajectory are carried out. This acutation command may not result in the trajectory that we predicted. Therefore, a new optimal trajectory is calculated for each vehicle state received by the controller, meaning the trajectory calculated with the previous vehicle state is discarded. This approach is sometimes called "receeding horizon control". 


### Vehicle State
["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["psi"] The car's steering angle

["v"] The car's velocity

["cte"] cross track error

["epsi"] psi error

### Waypoints
lake_track_waypoints.csv is the track waypoints provided by Udacity. This is the reference trajectory

### Actuator Commands
["delta"] steering angle  

["a"] acceleration


## MPC Setup

### Prediction Horizon
The time period is the product of two hyper parameters: N (number of steps) and dt (length of time between each step):
* T (the prediction horizon over which future predictions are made) = N * dt = 0.5 seconds
* N (number of timesteps in the horizon) = 10
* dt (time between actuations) = 0.05 seconds

I found that increasing the time period made the car unstable around turns at high speeds (100mph). T can be safely extended at lower speeds (<60mph)

### Vehicle Model
Kinematic Motion model

![motion model](motion_model.png)

### Constraints
Actuator limitations

### Cost Function
The cost is the sum of several components: cte, epsi, v, steer angle, acceleration, change in steering angle, and change in acceleration. I gave each of these components a weight that scaled their importance to the overall cost. The most important contributor to the cost was the change in steering angle, for which I sclaed by a factor of 20,000. This huge weight heavily penalized the optimization function for making sharp changes in the steering angle. The result was a significantly smoother drive, especially at high speeds.  


## MPC Program Loop
1. Convert waypoints from map coordinates to vehicle coordinates for use in the MPC solver.
```
for(int i =0; i < len; i++){ 
   ptsx_vehicle_coords[i] = cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
   ptsy_vehicle_coords[i] = -sin(psi) * (ptsx[i] - px) + cos(psi) * (ptsy[i] - py);
}
 ```
2. Fit a 3rd degree polynomial to the converted waypoints. The cross track error (cte) is equal to the polynomial coefficients evaluated at x=0. The psi error is the negative arctangent of the second coefficent. In the vehicle coordinate system, the vehicle x, y, and psi = 0
```
auto coeffs = polyfit(ptsx_vehicle_coords, ptsy_vehicle_coords, 3);
double cte = polyeval(coeffs, 0);
double epsi = -atan(coeffs[1]);
```
3. To account for 100 millisecond latency, use the kinematic model equations to predict where the car will be in 100 milliseconds. Because, in the vehicle coordinate system, x, y, psi = 0, the kinematic equations simply to the following:
```
Eigen::VectorXd state(6);
px = v * latency;
py = 0;
psi = - v / 2.67 * steer_value * latency;
v = v + throttle_value * latency;
state << px, py, psi, v, cte, epsi;
```
4. Pass the adjusted initial vehicle state and the waypoint polynomial coefficents to the MPC optimization solver. The solver uses the vehicle state, the kinematic vehicle model, vehicle constraints, and cost function, to simulate different actuator inputs over a time period (T) in an attempt to minimize a cost. The solution to a MPC is the trajectory with the lowest cost.
5. After the simulation, the solver returns x,y points for the optimal trajectory, and the first set of actuations (steering angle and acceleration). These values are passed to the simulator to control the car and display the vehicle / waypoint trajectories. When the next message is received from the simulator, the previous actuations and trajectories are thrown out, and this process is repeated with a new initial state.  

#### Ipopt
Ipopt is the C++ library used to optimize the control inputs. It's able to find locally optimal values (non-linear problem) while keeping the constraints set directly to the actuators and the constraints defined by the vehicle model. Ipopt requires jacobians and hessians as input. For this, the CppAD library is used for automatic differentiation


## Final Results
This model is able to successfully drive around the track with reference velocity set to 100mph (Note: The goal was 60mph. At 100mph, some trials will end with the car running off the road on sharp turns). 

Taking a turn at 97 MPH:

<a href="https://imgflip.com/gif/20qvw2"><img src="https://i.imgflip.com/20qvw2.gif" title="made at imgflip.com"/></a>




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



