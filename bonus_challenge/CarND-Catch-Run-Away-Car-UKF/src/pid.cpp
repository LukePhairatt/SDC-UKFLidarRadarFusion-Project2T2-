#include "pid.hpp"
#include "Eigen/Dense"    //M_PI


PID::PID(double Kp, double Ki, double Kd, bool NormalisedAngle) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
	normalised_angle_ = NormalisedAngle;
	error_ = 0.0;       
    diff_error_ = 0.0;  
    int_error_ = 0.0;  

}


PID::~PID() {}


double PID::RunPID(const double error) {
	double control_output;
	error_ = error;
	diff_error_ -= error_;
    int_error_ += error_;
    //+pi,-pi control
    if(normalised_angle_){
	  if(fabs(diff_error_) >  M_PI) diff_error_ = fmod(diff_error_ + M_PI, 2.*M_PI) - M_PI;
	  if(fabs(int_error_) >  M_PI) int_error_ = fmod(int_error_ + M_PI, 2.*M_PI) - M_PI;
	  // output
	  control_output = Kp_*error_ + Ki_*int_error_ + Kd_*diff_error_;
	  if(fabs(control_output) >  M_PI) control_output = fmod(control_output + M_PI, 2.*M_PI) - M_PI;
	  
    }else
    {
	  control_output = Kp_*error_ + Ki_*int_error_ + Kd_*diff_error_;
	}
    
    
	return control_output;
}
