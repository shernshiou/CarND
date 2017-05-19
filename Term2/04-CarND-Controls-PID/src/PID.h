#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

class PID {
public:
  double* error; // Errors
  double* K; //Coefficients
  bool twiddle_switch; // Twiddle switch

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
  void Init(double Kp, double Ki, double Kd, bool twiddle_switch);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Get the clamped PID error.
   */
  double ClampValue(double min, double max);

  /*
  * Twiddle the parameters.
   */
  void Twiddle(uWS::WebSocket<uWS::SERVER> ws);

private:
  int iterations; // Twiddle iterations
  double err;
  double best_err;
  double* dp;
  int dpi;
  int twiddle_lvl;
};

#endif /* PID_H */
