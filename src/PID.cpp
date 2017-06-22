#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double K_p, double K_i, double K_d) {
  Kp = K_p;
  Kd = K_d;
  Ki = K_i;
  p_error = 0;
  i_error = 0;
  d_error = 0;
  prev_cte = 0;

  // Later we need to call Twiddle;
}

void PID::UpdateError(double cte) {

  if (prev_cte == 0) prev_cte = cte;

  p_error = cte;
  d_error = cte - prev_cte;
  i_error += cte;

  //cout << "\n\n ***** DEBUGGING UPDATE ERROR ***** " << endl;
  //cout << "CTE: " << cte << endl;  
  //cout << "CTE (T-1): " << prev_cte << endl;
  //cout << "p_error: " << p_error << endl;
  //cout << "d_error: " << d_error << endl;
  //cout << "i_error: " << i_error << endl;
  //cout << "***** END OF UPDATE ERROR ***** \n\n" << endl;

  prev_cte = cte;

}

double PID::TotalError(double cte) {
  double total_error;
  total_error = cte * cte;

  return total_error;
}

double PID::CalculateSteering() {
  double steering;
  double max_steering = 1.;
  double min_steering = -1.;

  steering = -Kp * p_error - Kd * d_error - Ki * i_error;

  if (steering > max_steering) steering = max_steering;
  if (steering < min_steering) steering = min_steering;

  //cout << "\n\n ***** DEBUGGING CALCULATE STEERING ***** " << endl;
  //cout << "p_error: " << p_error << "\t\tKp: " << Kp << endl;
  //cout << "d_error: " << d_error << "\t\tKd: " << Kd << endl;
  //cout << "i_error: " << i_error << "\t\tKi: " << Ki << endl;
  //cout << "steering: " << steering << endl;
  //cout << "***** END OF CALCULATE STEERING ***** \n\n" << endl;

  return steering;
}

void PID::Twiddle(){

}