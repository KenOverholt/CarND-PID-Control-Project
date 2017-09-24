# CarND-PID-Control-Project
Self-Driving Car Engineer Nanodegree Program

---
## PID Controller Description

A PID Controller can be used as a mechanism for steering devices such as an automobile.  We start with a location in the lane that we would like the car to maintain.  We call this the reference trajectory.  The distance from that location is constantly checked and given a value.  Here we call it the Cross Track Error (CTE).  When the car drifts from our ideal location, the CTE grows.

The "P" in PID stands for Proportional.  Adjusting its value proportionally turns the car towards the reference trajectory.

The "D" in PID stands for Derivative.  This value can help reduce the oscilations that occur when just the P value is used.

The "I" in PID stands for Integral.  This value can help account for slight errors in alignment of the tires.  The tires on the car in this sumulator seem to be aligned correctly so I did not need to use this value.

## Parameter Tuning

I tuned my parameters by hand.  I started with just modifying P.  A larger value caused higher oscilations so I slowly lowered it.  Next I started modifying D which helped with the oscilations.  At one point, I was able to get the car completely around the track but was unable to reproduce my success so I continued experimenting.

Eventually, I concluded that the controller wasn't running fast enough to keep up with the speed of the car.  It was successful when the car first started but the simulator is set to keep increasing the car's speed so I changed the speed control.  I modified the program to keep the throttle at the pre-defined 0.0 but watch the speed and when it hit 10, drop the throttle back to 0.  This allowed my car success in getting around the track.

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

Recommended editor settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)
