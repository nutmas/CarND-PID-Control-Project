#include "PID.h"



using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    
    // set the local coefficients Kp,Ki, Kd variables to be the values passed in
    // pass proportional value to PID controller coefficient
    this->Kp = Kp;
    // pass derivative value to PID controller coefficient
    this->Kd = Kd;
    // pass integral value to PID controller coefficient
    this->Ki = Ki;
    
    
    // intialise all errors to 0
    p_error = d_error = i_error = 0.0;
    
    // initialise previous cte to 0
    prev_cte = p_error;
    
    current_time = 0.0; // Current frame time
    previous_time = clock(); // Previous frame time
    total_time = 0.0;  // Total time
    dt = 0.0; // time differnce
    
}

void PID::UpdateError(double cte) {
    
    // update saved current time
    previous_time = current_time;
    
    // set current time from clock
    current_time = clock();
    
    // keep log of total time
    total_time += current_time;
    
    // calculate time difference
    dt = (current_time - previous_time) / CLOCKS_PER_SEC;
    
    // update Proportional error
    p_error = cte;
    
    // update Integral error
    i_error += cte; // *dt;
    
    // reset integral if errro approaches low value
    if (fabs(cte) < 1.0) {
        i_error = 0;
    }
    
    // limit range integral can build up to - disable integral
    if (fabs(i_error) > 80) {
        i_error = 0;
    }
    
    // update Differential error - // /dt is removed as too much vibration
    d_error = (cte - prev_cte); // /dt;
    
    // store previous cte
    prev_cte = p_error;
    
    
    
}

double PID::TotalError(double cte) {
    
    //outputFile.open("PID_logs.txt", std::ios::app);
    
    // P controller only
    //double PID_calc =  -Kp * p_error;
    
    // PD controller only
    //double PID_calc = -Kp * p_error -Kd * d_error;
    
    // PID controller
    double PID_calc = -Kp * p_error -Ki * i_error -Kd * d_error;
    
    std::cout << "Target:" << cte << "\tP error:" << p_error << "\tI error:" << i_error << "\tD error:" << d_error << "\t\tTotal error:" << PID_calc << std::endl;
    
    //outputFile << dt << "," << cte << "," << PID_calc << "," << total_time << "," << i_error << "," << d_error << std::endl;
    
    //outputFile.close();
    
    return PID_calc;
    
    
}

