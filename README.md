# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Objective 

MPC(Model Predictive Control) is one of the important control methods to minimize the cross track and orientation errors of the vehicle with respect to the way points on the track. Objective of this project is to run around the track on the simulator successfully. This is achieved by MPC which has the ability to approximate a continuos reference trajectory by means of discrete paths between actuations.   


## MPC Algorithm

Setup:

    Define the length of the trajectory, N, and duration of each timestep, dt.
    Define vehicle dynamics and actuator limitations along with other constraints.
    Define the cost function.

Loop:

    We pass the current state as the initial state to the model predictive controller.
    We call the optimization solver. Given the initial state, the solver will return the vector of control inputs that minimizes the cost function. The solver we'll use is called Ipopt.
    We apply the first control input to the vehicle.
    Back to 1.



## The Model

The vehicle state:

* x: position of the vehicle in logintudinal direction
* y: position of the vehicle in lateral direction
* psi: orientation of the vehicle
* v: speed of the vehicle

The actuations:

* delta: steering angle in radian
* a: acceleration 

The errors:
* cte: cross track error
* epsi: orientation error

The others:
* Lf: distance between the vehicle's front and center of gravity
* dt: time duration

The update equations for the state at next time step:
~~~
      // Equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
~~~

## Time Length and Elapsed Duration (N & dt)

N is the number of timesteps. dt is how much time elapses between actuations. If N is 15 and dt is 0.1, then T would be 1.5 seconds.I started with N=25 and dt=0.05 (1.25 sec), but it didn't make good result. Next I tried N=10 and dt=0.1 (1.0 sec) and this makes better result, but when the car enters into sharp corners, it tended to overshoot probably due to latency. Finally I set N=15 and dt=0.1 (1.5 sec) and made the best result considering latency.    


## Polynomial Fitting and MPC Preprocessing

At first, I transformed waypoints into the vehicle cordinate system. The x, y, and psi are zero at initial state because the cordinate is relative to the vehicle. I fit 3 order polynomial and got coefficients in (coeffs[0] + coeffs[1] * x + coeffs[2] * x^2 + coeffs[3] * x^3)  to calculate the cross track error and orientation error for passing to the MPC solver later. 

## Model Predictive Control with Latency

The latency issue affects cross track error and orientation error because 100ms latency makes increase those values. The steering and acceleration values are already past value when calculating those values by solver for the first time step. I adjusted N and dt values and look 2 steps ahead for mpc predicted way points to mitigate this issue. As result with N=15 and dt=0.1 (1.5 sec) made the smooth run and the best result.  

~~~
          //Use 2 time-steps ahead
          for (int i = 6; i < vars.size(); i ++) {
            if (i % 2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            }
            else {
              mpc_y_vals.push_back(vars[i]);
            }
          }
~~~

---

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./


## Build

* Under /lib/ ipopt library is installed
* Under /lib/include there are coin/*.h files

~~~
g++ -o mpc MPC.cpp MPC.h json.hpp main.cpp -std=c++11 -I/lib/include/ -L/lib/ -luWS -luv -lz -lssl -lipopt
~~~


