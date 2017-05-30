#include "PID.h"
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    
    p_error = 0.0 ;
    i_error = 0.0 ;
    d_error = 0.0 ;
}

void PID::UpdateError(double cte)
{
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    //cout << "p_error: " << p_error << " i_error: " << i_error << " d_error: " << d_error << endl;
}

double PID::TotalError()
{
    double total_error = Kp * p_error + Kd * d_error + Ki * i_error;
    //cout << "Total_error: " << total_error;
    return total_error;
}

