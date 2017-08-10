# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Overview
This project implements a PID controller to keep a car on a race track by properly adjusting the steering angle.

### What is a PID controller?
A controller is used to manage the steering, throttle and brakes to move a vehicle where we'd like it to go. A Proportional–Integral–Derivative (PID) controller is one of the most common forms of these control loop feedback mechanisms. A PID controller continuously calculates an error function (in this case: the distance from the center of the lane) and applies a correction based on proportional (P), integral (I), and derivative (D) terms.

### Choosing PID Parameters
The behavior of a PID controller depends on three main parameters, or control gains:

1) Proportional - involves steering the vehicle proportionally to cross track error, which is the lateral difference between the vehicle and its reference trajectory. This process results in slightly, continuously overshooting the desired path:
   
   steering = -tau * cte

   Tau is the control parameter and increasing this figure makes the vehicle converge with the reference trajectory faster (oscillation is faster with a higher tau).
   
2) Integral - magnitude of error, as well as the duration of error are taken into account. In this way the controller is able to eliminate the residual steady-state error that occurs with a pure proportional controller (i.e. a purely proportional controller operates only when error is non-zero) and is able to deal with systematic biases. The integral is utilized to eliminate steering drift.

3) Derivative (or Differential) - the goal is to avoid the overshoot that occurs when using a P controller. The steering alpha is also related  to the temporal derivative of the cross track error. This means that when the car has turned enough to reduce the cross track error, it will recognize this and counter steer as the error becomes smaller over time.

In this project, parameters were manually tuned by qualitatively inspecting the driving behavior in the simulator, in response to parameter changes. I used the classroom examples as a starting point for trial and error. Test run videos are included in the submission for P controller and PID controller (doesn't seem to be any bias so showing a PD controller example as well seems pointless). Results from various parameters are noted in the code.


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.
