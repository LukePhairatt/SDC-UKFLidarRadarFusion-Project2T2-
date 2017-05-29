#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here. input:stack of [px,py,vx,vy] vector
  */

  // error at each point along the path
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()|| estimations.size() == 0){
	  cout << "Invalid estimation or ground_truth data" << endl;
	  return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < int(estimations.size()); ++i){
	  // residual of each element
	  VectorXd residual = estimations[i]-ground_truth[i];
	  // sq error element-wise multiplication
	  residual = residual.array()*residual.array();
	  // element-wise summation
	  rmse += residual;
	  // rmse is still a vector
  }

  //calculate the mean
  rmse = rmse/estimations.size();
  //calculate the squared root (element wise)
  rmse = rmse.array().sqrt();
  return rmse;  // error at each point along the path

}
