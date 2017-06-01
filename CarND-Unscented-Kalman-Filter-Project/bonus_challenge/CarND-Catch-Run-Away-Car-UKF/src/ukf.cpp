#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector- target tracked by UKF
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // (required experimenting) Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.07;

  // (required experimenting) Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.09;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  
  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter (will be changed later)
  lambda_ = 0;

  ///* the current NIS for radar
  NIS_radar_ = 0.0;

  ///* the current NIS for laser
  NIS_laser_ = 0.0;

  ///* Weights of sigma points
  weights_ = VectorXd(2 * n_aug_+ 1);

  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1) ;
  
  ///* set point to be chased
  xt_ = VectorXd(2);
  
  ///* set look ahead time step in seconds
  time_lookahead_ = 1.5;
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage &meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.

  This is a main function to run a process prediction and Lidar or Radar update
  */

  // init state from the first measurement
  if(!is_initialized_)
  {
	  if(!use_radar_ && !use_laser_){
		  cout << "Error, you must define one source of measurement!" << endl;
		  exit(-1);
	  }

	  // State vector
	  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		  double ix = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
		  double iy = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
		  x_ << ix,iy,0,0,0;   // Target tracked by UKF
		  
	  }
      else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    	  x_ << meas_package.raw_measurements_[0],meas_package.raw_measurements_[1],0,0,0; // Target tracked by UKF 
    	  
	  }
      else{
    	  cout << "Undefined measurement data" << endl;
	  }

	  // Covariance P 
	 
	  P_ << 0.02, 0,    0,    0,    0,
	  	    0,    0.02, 0,    0,    0,
	  	    0,    0,    5.0,  0,    0,
	  	    0,    0,    0,    2.0,  0,
			0,    0,    0,    0,    0.2;
	 

	  // set weights
	  lambda_ = 3 - n_aug_;
	  double weight_0 = lambda_/(lambda_+n_aug_);
	  double weight_n = 0.5/(n_aug_+lambda_);
	  weights_(0) = weight_0;
	  int index = 2*n_aug_+1;
	  for (int i=1; i<index; i++) {
	    weights_(i) = weight_n;
	  }

	  // Set time at this state for the next measurement
	  time_us_ = meas_package.timestamp_;
	  
	  
	  // Set first target points (anywhere inside the circle)
	  xt_ << 0.0, 10.0;
	  
	  is_initialized_ = true;
	  return;
  }//end if
  
  // Predict the next position dt //
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  // Record this time stamp for the next cycle
  time_us_ = meas_package.timestamp_;
  
  if(dt > 0.01){ 
	 Prediction(dt);
  }
   
  // Correction //
  // Radar/Lidar update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
	  UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
	  UpdateLidar(meas_package);
  }
  
  // Setting the target waypoint//
  time_lap_ += dt;
  // set the new way point if the elapsed time expired 
  if(time_lap_ > time_lookahead_){
	  SetTargetPoint(time_lookahead_); // set look ahead way points
	  time_lap_ = 0.0;                 // reset timer
  }
  
}//end ProcessMeasurement


void UKF::SetTargetPoint(const double timestep)
{
  double delta_t = timestep;
  double p_x = x_(0);
  double p_y = x_(1);
  double v = x_(2);
  double yaw = x_(3);
  double yawd = x_(4);

  //predicted state values
  double px_p, py_p;
  //avoid division by zero
  if (fabs(yawd) > DIV_ZERO_TOL_) {
	  // Turning
	  px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
	  py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
  }
  else{
	  // Straight line
	  px_p = p_x + v*delta_t*cos(yaw);
	  py_p = p_y + v*delta_t*sin(yaw);
  }
  // set the target point for the chaser bot
  xt_ << px_p,py_p;	
}



/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(const double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //
  // General Sigma points
  //

  lambda_ = 3 - n_x_;
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
  MatrixXd A = P_.llt().matrixL();
  MatrixXd sigma_p = MatrixXd(n_x_,n_x_);
  sigma_p = sqrt(lambda_ + n_x_) * A.array();
  Xsig << x_.array(),(sigma_p.colwise() +  x_).array(),((-sigma_p).colwise() +  x_).array();

  //
  // Augment sigma points
  //

  lambda_ = 3 - n_aug_;
  // Augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // Augmented state covariance
  //[P 0   0
  // 0 qa  0
  // 0 0   qw]
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // Augment sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // fill x augment state (linear and angular acceleration = 0)
  x_aug << x_,0,0;
  // fill P augment covariance
  P_aug.fill(0.0); 						        //fill all elements with 0.0
  P_aug.topLeftCorner(n_x_,n_x_) = P_; 	 	    //fill first 5x5 with P
  P_aug(n_x_,n_x_) = std_a_*std_a_;             //fill linear  acceleration variance
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_; //fill angular acceleration variance

  // Compute X augment sigma points
  A = P_aug.llt().matrixL();
  //create augmented sigma points
  sigma_p = MatrixXd(n_aug_,n_aug_);
  sigma_p = sqrt(lambda_ + n_aug_) * A.array();
  Xsig_aug << x_aug.array(),(sigma_p.colwise() +  x_aug).array(),((-sigma_p).colwise() +  x_aug).array();

  //
  // Predict sigma points using the x = x + integral  x' dt model
  //

  int index = 2*n_aug_+1;
  for (int i = 0; i< index; i++){
	  //extract values for better readability
	  double p_x = Xsig_aug(0,i);
	  double p_y = Xsig_aug(1,i);
	  double v = Xsig_aug(2,i);
	  double yaw = Xsig_aug(3,i);
	  double yawd = Xsig_aug(4,i);
	  double nu_a = Xsig_aug(5,i);
	  double nu_yawdd = Xsig_aug(6,i);

	  //predicted state values
	  double px_p, py_p;
	  //avoid division by zero
	  if (fabs(yawd) > DIV_ZERO_TOL_) {
		  // Turning
		  px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
		  py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
	  }
	  else{
		  // Straight line
		  px_p = p_x + v*delta_t*cos(yaw);
		  py_p = p_y + v*delta_t*sin(yaw);
	  }

	  double v_p = v;
	  double yaw_p = yaw + yawd*delta_t;
	  double yawd_p = yawd;

	  //add noise
	  px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
	  py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
	  v_p = v_p + nu_a*delta_t;
	  //Check yaw_p within +pi,-pi ? (or we handle this later before the final prediction)
	  yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
	  if(fabs(yaw_p) >  M_PI) yaw_p = fmod(yaw_p + M_PI, 2.*M_PI) - M_PI;

	  yawd_p = yawd_p + nu_yawdd*delta_t;

	  //write predicted sigma points
	  Xsig_pred_(0,i) = px_p;
	  Xsig_pred_(1,i) = py_p;
	  Xsig_pred_(2,i) = v_p;
	  Xsig_pred_(3,i) = yaw_p;
	  Xsig_pred_(4,i) = yawd_p;
  }//end for

  //
  // Predict Mean and Covariance
  //

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //predict state mean
  x.fill(0.0);
  //iterate over sigma points
  for (int i = 0; i < index; i++) {
    x = x+ weights_(i) * Xsig_pred_.col(i);
  }// end for

  //predict state covariance matrix
  P.fill(0.0);
  VectorXd x_diff = VectorXd(n_x_);           //state diff
  for (int i = 0; i < index; i++) {
    x_diff = Xsig_pred_.col(i) - x;
    //+pi,-pi
    if(fabs(x_diff(3)) >  M_PI) x_diff(3) = fmod(x_diff(3) + M_PI, 2.*M_PI) - M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }// end for

  // copy state and covariance
  x_ = x;
  P_ = P;
  
  cout << "px = " << x[0] << "  " << "py = " << x[1] << " " << "v = " << x[2] << "  " << "theta = " << x[3] << " " << "theta_d = "  << x[4] << endl;
  
}// end Prediction

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage &meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  //
  //   Measurement Prediction  //
  //

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 2;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space -> Zsig
  int index = 2*n_aug_+1;
  for (int i = 0; i< index; i++)
  {
	 //extract values px, py state
	 Zsig(0,i) = Xsig_pred_(0,i);
	 Zsig(1,i) = Xsig_pred_(1,i);
  }// end for

  //Radar measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<     std_laspx_*std_laspx_, 0,
		   0, std_laspy_*std_laspy_;

  //
  //   Measurement Update  //
  //
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0],meas_package.raw_measurements_[1];
  NIS_laser_ = UKFCorrection(Zsig, R, z);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage &meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //
  //   Measurement Prediction  //
  //

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space -> Zsig
  int index = 2*n_aug_+1;
  for (int i = 0; i< index; i++)
  {
     //extract values for better readability
     double px = Xsig_pred_(0,i);
     double py = Xsig_pred_(1,i);
     double v = Xsig_pred_(2,i);
     double yaw = Xsig_pred_(3,i);

     double vx = v*cos(yaw);
     double vy = v*sin(yaw);

     double rho = sqrt(px*px + py*py);
     double theta = atan2(py,px);
     double rho_dot;
     if(rho < DIV_ZERO_TOL_){
		// This will happen we the target goes near the radar or the target
		// starting/passing at 0,0 point
		// To solve this, we either skip this update or use tolerance
 		rho_dot = DIV_ZERO_TOL_;
 	    //rho_dot = vx*cos(theta) + vy*sin(theta);
 		//cout << "Measurement distance near zero" << rho_dot << endl;
     }
     else{
 		rho_dot = (px*vx + py*vy)/rho;
 	}

     Zsig(0,i) = rho;
     Zsig(1,i) = theta;
     Zsig(2,i) = rho_dot;
     

  }// end for

  //Radar measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<     std_radr_*std_radr_, 0, 0,
           0, std_radphi_*std_radphi_, 0,
           0, 0,std_radrd_*std_radrd_;

  //
  //   Measurement Update  //
  //
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0],meas_package.raw_measurements_[1],meas_package.raw_measurements_[2];
  NIS_radar_ = UKFCorrection(Zsig, R, z);


}// end UpdateRadar

/**
 * Updates the state and the state covariance matrix using a radar measurement
 * @param meas_package The measurement at k+1
*/
double UKF::UKFCorrection(const MatrixXd &Zsig, const MatrixXd &R, const VectorXd &z){
  //
  //   Measurement Update  //
  //
  
  int n_z = Zsig.rows();
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //calculate mean predicted measurement -> z_pred
  z_pred.fill(0.0);
  int index = 2*n_aug_ + 1;
  for (int i=0; i < index; i++) {
	   z_pred = z_pred + weights_(i) * Zsig.col(i);
  }// end for

  //calculate measurement covariance matrix S and cross correlation Tc
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  MatrixXd Tc = MatrixXd(n_x_, n_z);         //cross correlation Tc matrix
  Tc.fill(0.0);
  VectorXd z_diff = VectorXd(n_z);           //measurement difference
  VectorXd x_diff = VectorXd(n_x_);          //state difference

  for (int i = 0; i < index; i++) {
	// state and measurement difference between Sigma points and Predicted mean points
	z_diff = Zsig.col(i) - z_pred;           // z_pred is the measurement from the measurement prediction step
	x_diff = Xsig_pred_.col(i) - x_;         // x_ is the state from the state prediction step

	//+pi,-pi control
	if(fabs(z_diff(1)) >  M_PI) z_diff(1) = fmod(z_diff(1) + M_PI, 2.*M_PI) - M_PI;
	if(fabs(x_diff(3)) >  M_PI) x_diff(3) = fmod(x_diff(3) + M_PI, 2.*M_PI) - M_PI;

	// compute S and Tc
	S = S + weights_(i) * z_diff * z_diff.transpose();
	Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }// end for

  //add Radar measurement noise covariance matrix
  S = S + R;


  //
  //    State Correction  //
  //

  //Kalman gain K;
  MatrixXd Sinv = S.inverse();
  MatrixXd K = Tc * Sinv;

  //measurement residual between the ACTUAL measuremennt and the PREDICTED ones
  z_diff = z - z_pred;
  //+pi,-pi control
  if(fabs(z_diff(1)) >  M_PI) z_diff(1) = fmod(z_diff(1) + M_PI, 2.*M_PI) - M_PI;

  //correct the predicted state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //NIS (Normalised Innovation Square)
  MatrixXd EE = z_diff.transpose() * Sinv * z_diff;   // this should result in [1x1]
  return EE(0,0);                                     // return index 0,0
}




