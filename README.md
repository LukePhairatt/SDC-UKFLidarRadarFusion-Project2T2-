# **Unscented Kalman Filter(UKF) Project-2 Term-2**
Self-Driving Car Engineer Nanodegree Program
![alt text][image0]
![alt text][image1]

[//]: # (Image References)
[image0]: ./Docs/ukf_xy.png "xy result"
[image1]: ./Docs/RunAwayRobot.png "Runaway Robot"
[image2]: ./Docs/SigmaGen.png "Sigma Point Generation"
[image3]: ./Docs/PredictedMean.png "Predicted Mean"
[source1]: ./data/PathVis_in.py "log input check"
[source2]: ./data/PathVis_out.py "log output check"


---

### Overview ###
**Object Tracking**

In this project utilize Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.   

Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project reburic. 


```sh

                               
                   y
                   ^     
                   |  [object] (px,py)
                   |   /
                   |  / range, range_dot
                   | /      (Radar)
                   |/ angle
                   ------------> 
		lidar: px,py
		radar: range,range_dot,angle

```

### UKF State Params###
**System State**
```sh
	p_x  = x position
	p_y  = y position
	v    = velocity 
	yaw  = heading
	yawd = angular velocity
	nu_a = linear acceleration(Augment)
	nu_yawdd = angular acceleration(Augment)
```

**Measurement**
_Radar Measurement_
```sh
	rho = range measurement of a target
	theta = bearing angle of a target
 	rho_dot = rate of change in range of a target 

	Note: we need to map the state variables to these measurements for the correction step
	      using the following equation

	v_x = v*cos(yaw);
	v_y = v*sin(yaw);
	rho = sqrt(p_x*p_x + p_y*p_y);
	theta = atan2(p_y,p_x);
	rho_dot = (p_x*v_x + p_y*v_y)/rho;
```


_Lidar Measurement_
```sh
	px = a target/object x-position
	py = a target/object y-position
```

### UKF State Estimation Steps ###
**STEP 1-Sigma Points Prediction:**

_Generating the state sigma points(Xsing_aug from the current mean state)_

![sigma point generation][image2]

```sh
	Augmented X State and P Covariance Matrix
		n_aug_ = 7;
		MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
		MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
	
	Computing augmented X State
		MatrixXd A = P_aug.llt().matrixL();
		lambda_ = 3 - n_aug_
		sigma_p = sqrt(lambda_ + n_aug_) * A.array();
		Xsig_aug << x_aug.array(),(sigma_p.colwise() +  x_aug).array(),((-sigma_p).colwise() +  x_aug).array();

```

_Predict the next augmented sigma points(Xsig_Pred)_

For each X sigma points (vector)  
		Compute the next state Xsig_pred_

```sh
	Using integral motion model
		X = X + integral X' dt + acceleration residual 

	Predict the next state (Avoiding division by zero yawd==0)
		//With turning motion
		px_p = p_x + v/yawd * ( sin(yaw + yawd*delta_t) - sin(yaw));
		py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
		// With straight line motion
		px_p = p_x + v*delta_t*cos(yaw);
		py_p = p_y + v*delta_t*sin(yaw);

		v_p = v;
		yaw_p = yaw + yawd*delta_t;
		yawd_p = yawd;

	Adding acceleration residual terms
		px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
		py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
		v_p  = v_p + nu_a*delta_t;
		yaw_p  = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
		yawd_p = yawd_p + nu_yawdd*delta_t;	
```


**STEP 2-Compute Predicted Mean and Covariance:**
![Predicted Mean][image3]

_Commpute the predicted mean and covariance (x, P)_  

```sh
        x = [0], P = [0]
	for each vector i
		x = x+ weights_(i) * Xsig_pred_.col(i);

	for each vector i
		x_diff = Xsig_pred_.col(i) - x;
		P = P + weights_(i) * x_diff * x_diff.transpose();
```


**STEP 3-Measurement Correction:**

_Compute the measurement prediction (Zsig) from the predicted sigma points(Xsig_pred)_

```sh
	for each column vector
		Lidar: just copy the Xsig_pred state[0:1] for px,py
		Radar: map the state to the measurement rho, theta, rho_dot


```

_Compute the measurement covariance (S) and cross correlation (Tc)_

```sh
	Compute predicted measurement: z_pred

		for each column vector i
			z_pred = z_pred + weights_(i) * Zsig.col(i); 

	Compute S and Tc: 
		init S,Tc to 0.0
		for each column vector i
			z_diff = Zsig.col(i) - z_pred;
			x_diff = Xsig_pred_.col(i) - x; 
			S = S + weights_(i) * z_diff * z_diff.transpose();
			Tc = Tc + weights_(i) * x_diff * z_diff.transpose();

		S = S+R
```

_Do the measurement correction_

```sh
	Kalman gain:
		z_diff = z - z_pred;
		K = Tc * S.inverse()
		x = x + K * z_diff;
		P = P - K*S*K.transpose();
```

_Normalised Square Innovation(NIS)_

```sh
	NIS = z_diff.transpose() * S.inverse() * z_diff;

```


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
* Chasing the runaway target, see **bonus_challenge** folder for implementation  
* PathVis_in.py: Analyse the input log file for the object positions, speeds, and accelerations [here][source1]  
* PathVis_out.py: Analyse the output log file after running the UKF for Normalised Innovation Error and Path [here][source2]  
* Eclipse project: see **ide_eclipse** folder for further instructions  



