#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;

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
  * Initializes PID.
  */
  void Init(double K_p, double K_i, double K_d);

  /*
  * Updates the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculates the total PID error.
  */
  double TotalError(double cte);

  /*
  * Calculates the steering angle.
  */
  double CalculateSteering();

  /*
  * Optimizes tau values (Kp, Ki, Kd).
  */
  void Twiddle();
};

#endif /* PID_H */
