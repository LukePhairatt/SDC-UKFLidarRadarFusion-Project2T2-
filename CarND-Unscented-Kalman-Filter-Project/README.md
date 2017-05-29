# **Unscented Kalman Filter(UKF) Project-2 Term-2**
Self-Driving Car Engineer Nanodegree Program
![alt text][image0]

[//]: # (Image References)
[image0]: ./Docs/ukf_xy.png "xy result"
[source1]: ./data/PathVis_in.py "log input check"
[source2]: ./data/PathVis_out.py "log output check"

---

### Overview ###
In this project utilize Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.   
Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project reburic. 


### Dependencies ###

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

### Basic Build Instructions ###

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

### Configuration ###
1. Using Lidar and/or Radar update: Change use_laser_ and use_radar_ Flag in ukf.cpp and recompile
2. Process noise: std_a_ and std_yawdd_ has been tuned to this particular project in ukf.cpp 


### Additional Resources ###
* PathVis_in.py: Analyse the input log file for the object positions, speeds, and accelerations [here][source1]  
* PathVis_out.py: Analyse the output log file after running the UKF for Normalised Innovation Error and Path [here][source2]
* Eclipse project: see ide_eclipse for further instruction


