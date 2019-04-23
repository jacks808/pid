#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  p_error = 0;
  i_error = 0;
  d_error = 0;

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */

  // d_error is differential of current error. That means, the speed of the error increase
  // update d_error is: use current error substract last error (which is p_error)
  d_error = cte - p_error;

  // p_error is how much distance from the currently value to required value, so it is called 'proportion'
  // update p_error is: use current error value to update p_error
  p_error = cte;

  // i_error is how much error from begin point to current position, so it is called 'integral'
  // update i_error is: sum all error together
  i_error = i_error + cte;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  // Total error calc here!
  return -Kp * p_error - Kd * d_error - Ki * i_error;
}
