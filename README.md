# CarND-Controls-PID
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

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Writeup

I decided to slightly change the implementation of the PID class and use vector a lot as opposed to single variables or arrays. 
Some of the reasons that motivated this choice are the folloing:
* vector<T> enables random access just like arrays
* vector<T> offers many convenient methods to obtain current size, resize, etc
* vector is part of the C++ standard library

### Twiddle

The variant of my Twiddle algorithm took some time to figure out as I could not use the same Twiddle from the lessons since I did not have the
ability to run a loop in the code, and therefore had to rely on callbacks from another process (i.e. the simulator). Essentially the Twiddle has
2 phases at the most for each component (i.e. P, I, and D):
* _Phase 0_ where we add a delta to the current component and calculate the steering angle for _N_ callbacks
 * If after N callbacks the error has gone down we increase the delta by 10% (i.e. multiply by 1.1) and move on to to the next component
 * If after N callbacks the error has **not** gone down we move to _Phase 1_ on the same component
* _Phase 1_ is simular to _Phase 0_ except that we subtract the delta from the current component as opposed to adding it
  * If after N callbacks the error has gone down we increase the delta by 10% (i.e. multiply by 1.1) and move on to to the next component
  * If after N callbacks the error has **not** gone down we decide to reduce the delta for this component by 10% (i.e. multiply by 0.9) and move on to the next component

Twiddle is performed for as long as the sum of deltas for all components is greater than a small epsilon value.

### Parameters P, I, and D

#### Delta Parameters


I initially set the deltas dPp, dPi, and dPd to the same values (0.1) and noticed that the car would quickly go out of bounds. 
I thought it was initially due to the speed and so decided to reduce the throttle from 0.3 to 0.2 but this only delayed the problem. 
I then attempted to think logically about this problem and looked again at the formula from Sebastian Thrun un the lectures:
* The _dPp_ parameter is only affected by the current cte and therefore retains no history of previous CTE. On its own it will make the car oscillate too much from the reference trajectory it should espouse
* The _dPi_ parameter takes into account the sum of previous ctes. **As this sum can grow large it is important to set this coefficient to a smaller value otherwise it will have a disproportionate importance in how we compute the steering angle.** 
* The _dPd_ parameter would capture changes between two consecutive runs and is therefore possibly the most important component, as the difference between the current and previous ctes provides critical information on how to compensate for mainly the _dPp * cte_ part of the formula. **It is therefore the parameter that should be initialised with the highest value**

I decided to apply the following logic:
_dPi_ = _dPd * 0.001_
_dPp_ = _dPd * 0.01_

However, too strong values for _dPd_ cause the car to unecessarily turn too often which affects the speed and general comfort of the ride, as can be seen in the gif below:

![Strong value for Dpd](docs/PID_strong_value_for_dPd.gif)

Therefore we should make sure to also set it to a reasonable value (e.g. 0.1).

#### Initial PID Values

I decided to start with a _P_ having a larger value than other components, because I thought it would be the most important component at the start since we won't have any deltas nor enough accumulated cross-track error for the other components to have a big enough impact.

The values for the _I_ and _D_ components were found by trying out different values. It turns out setting _D_ to the smallest value at the outset helps obtain the best results.


### Further Observations

I am not sure but it seems inverting the scale of values between the initial _PID_ and _dPp, dPi dPd_ values produces the best results: e.g. if _D_ has the smallest initial value of the 3 then it should have the largest initial delta value _dPd_.

Throttle was kept at 0.3.

I did consider somehow making the steering proportional to the speed as well but have not yet found the right approach.

The gif below shows a successful run the car on a small portion of the track:

![Successful PID run](docs/PID_success_small_portion_track.gif)

