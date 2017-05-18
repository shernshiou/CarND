#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  this->i_error += cte;
  this->d_error = cte - p_error;
  this->p_error = cte;
}

double PID::TotalError() {
  return -this->Kp * this->p_error - this->Ki * this->i_error - this->Kd * this->d_error;
}
