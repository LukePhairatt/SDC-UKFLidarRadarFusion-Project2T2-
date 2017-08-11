#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"
#include <functional>   // std::bind

using namespace Eigen;
using namespace std::placeholders;
using namespace std;


const double DIV_ZERO_TOL_ = 0.001;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  ///* Radar measurement length
  int nz_radar_;

  ///* Lidar measurement length
  int nz_lidar_;

  ///* Radar measurement noise covariance matrix
  MatrixXd R_radar_;

  ///* Lidar measurement noise covariance matrix
  MatrixXd R_lidar_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage &meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(const double delta_t);

  /**
   * Updates the state and the state covariance
   * @param The measurement at k+1, measurement size, Covariance Noise, update function
   */
  double UpdateMeasurements(const MeasurementPackage &meas_package, int n_z, const MatrixXd& R, \
		  std::function<double (const MeasurementPackage& meas_package, int index, const MatrixXd& Xsig_pred, MatrixXd& Zsig)> mapping_func);

  /**
   * Compute measurement sigma points using a lidar measurement, mapping the state to the measurement space
   *
   */
  double LidarMeasurementMapping(const MeasurementPackage& meas_package, int index, const MatrixXd& Xsig_pred, MatrixXd& Zsig);

  /**
   * Compute measurement sigma points using a radar measurement, mapping the state to the measurement space
   *
   */
  double RadarMeasurementMapping(const MeasurementPackage& meas_package, int index, const MatrixXd& Xsig_pred, MatrixXd& Zsig);

  /**
   * Compute measurement covariance matrix S and cross correlation Tc and State correction
   * @param input: measurement sigma points, measurement noise, and actual measurements
   *        output: NIS
  */
  double UKFCorrection(const MatrixXd &Zsig, const MatrixXd &R, const VectorXd &z);
  
  double NormaliseAngle(double theta);
  
};

#endif /* UKF_H */
