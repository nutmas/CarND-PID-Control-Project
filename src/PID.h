#ifndef PID_H
#define PID_H

#include <math.h>
#include <iostream>
#include <vector>

#include <fstream> // for file writing

#include <float.h> // for DBL_MAX

#include <numeric> // for accumulate


class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double cte);
    

  
private:
  /*
  * Functions to tune PID paramters
  */
    
    // variable to keep track of previous CTE
    double prev_cte;
    
    // file for writing values
    std::ofstream outputFile;
    
    // variables to keep track of time
    double current_time;
    double previous_time;
    double total_time;
    double dt;
    
};

#endif /* PID_H */
