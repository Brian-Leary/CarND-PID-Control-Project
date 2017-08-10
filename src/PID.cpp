#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    p_error = 0;
    i_error = 0;
    d_error = 0;
}

void PID::UpdateError(double cte) {
    // P controller
    //p_error = cte;

    // PD controller
    //d_error = cte - p_error;
    //p_error = cte;

    // PID controller
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    // P controller
    //return -(Kp_ * p_error);

    // PD controller
    //return -(Kp_ * p_error + Kd_ * d_error);

    // PID controller
    return -(Kp_ * p_error + Ki_ * i_error + Kd_ * d_error);
}

