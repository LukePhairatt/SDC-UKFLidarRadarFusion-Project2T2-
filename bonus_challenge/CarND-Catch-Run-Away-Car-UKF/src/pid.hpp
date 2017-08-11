#ifndef PID_H
#define PID_H


class PID {
  public:
    double Kp_; ///* Kp gain
    double Ki_; ///* Ki gain
    double Kd_; ///* Kd gain
    
    /**
    * Constructor
    */
    PID(double Kp, double Ki, double Kd, bool NormalisedAngle);

    /**
    * Destructor
    */
    virtual ~PID();
    
    double RunPID(const double error);
  
  private:
    double error_;       ///* unit error
    double diff_error_;  ///* unit error different
    double int_error_;   ///* unit error accumulation
    double normalised_angle_;
    
};

#endif /* PID_H */
