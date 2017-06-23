#include "PID.h"
#include <iostream>
#include <uWS/uWS.h>
#include <numeric>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double K_p, double K_i, double K_d) {
  // Initializes K
  p[0] = K_p;
  p[1] = K_i;
  p[2] = K_d;

  // Initializes errors
  e[0] = 0;       // p_error
  e[1] = 0;       // i_error
  e[2] = 0;       // d_error
  prev_cte = 0;
  twiddle_error = 0;


  // optimizer timesteps
  optimizer_t = 0;

}

void PID::UpdateError(double cte) {

  if (prev_cte == 0) prev_cte = cte;

  e[0] = cte;             // p_error
  e[1] += cte;            // i_error
  e[2] = cte - prev_cte;  // d_error
  
  //cout << "\n\n ***** DEBUGGING UPDATE ERROR ***** " << endl;
  //cout << "CTE: " << cte << endl;  
  //cout << "CTE (T-1): " << prev_cte << endl;
  //cout << "p_error: " << e[0] << endl;
  //cout << "i_error: " << e[1] << endl;
  //cout << "d_error: " << e[2] << endl;
  //cout << "***** END OF UPDATE ERROR ***** \n\n" << endl;

  prev_cte = cte;



}

double PID::TotalError() {
  double total_error;
  total_error = e[0] * e[0];

  return total_error;
}

double PID::CalculateOutput() {
  double output;
  double max_output = 1.;
  double min_output = -1.;

  output = -p[0] * e[0] - p[1] * e[1] - p[2] * e[2];

  if (output > max_output) output = max_output;
  if (output < min_output) output = min_output;

  //cout << "\n\n ***** DEBUGGING CALCULATE STEERING ***** " << endl;
  //cout << "p_error: " << e[0] << "\t\tKp: " << p[0] << endl;
  //cout << "d_error: " << e[2] << "\t\tKd: " << p[2] << endl;
  //cout << "i_error: " << e[1] << "\t\tKi: " << p[1] << endl;
  //cout << "steering: " << steering << endl;
  //cout << "***** END OF CALCULATE STEERING ***** \n\n" << endl;

  return output;
}

double PID::CalculateThrottle(double speed, double steering_angle) {
  double throttle;
  //double max_throttle = 1.;
  //double min_throttle = -1.;

  /*
  if (fabs(steering_angle) < 0.1) {
    throttle = 1.0;
  } else if ((fabs(steering_angle) > 0.25) & (speed > 50)) {
    throttle = -0.5;
  } else {
    throttle = 0.7;
  }*/

  throttle = 0.3;

  //cout << "\n\n ***** DEBUGGING CALCULATE THROTTLE ***** " << endl;
  //cout << "speed: " << speed << endl;
  //cout << "steering angle: " << steering_angle << endl;
  //cout << "calculated throttle: " << throttle << endl;
  //cout << "***** END OF CALCULATE THROTTLE ***** \n\n" << endl;

  //if (throttle > max_throttle) throttle = max_throttle;
  //if (throttle < min_throttle) throttle = min_throttle;


  return throttle;
}

void PID::Twiddle(){


  // Accumulates Error
  if (use_twiddle) {
    if (optimizer_t == start_meas) {
      twiddle_error = TotalError();
      //cout << "\noptimizer started accruing at timestep: " << optimizer_t << endl;
      //cout << "total error: " << twiddle_error << endl;; 
    } 
    if (optimizer_t > start_meas) {
      twiddle_error += TotalError();
      //cout << "total error: " << twiddle_error << endl; 
    }
    if (optimizer_t > end_meas) {
      cout << "\noptimizer error measured at timestep " << optimizer_t;
      cout << "\ttotal error = " << twiddle_error << endl; 

      if (best_err == -1) best_err = twiddle_error + 1;

      double sum_dp = accumulate(begin(dp), end(dp), 0.0, plus<double>());

      if (sum_dp > twiddle_w_tol) {
      //while (sum_dp > twiddle_w_tol) {

        //Debugging steps
        cout << " Iteration " << twiddle_iter << " best error = " << best_err << endl;

        // resets parameter we are optimizing (replaces the for loop)
        if (twiddle_pass >= num_params) twiddle_pass = 0;

        p[twiddle_pass] += dp[twiddle_pass];
        cout << "Optimizing Parameter " << twiddle_pass << endl;

        if (is_retrial) {
          if (twiddle_error < best_err) {
            cout << "Now it improves. Moving to next parameter" << endl;
            best_err = twiddle_error;
            dp[twiddle_pass] *= 1.1;
            twiddle_pass++;
            twiddle_iter++;
            RestartSim(server);
            is_retrial = false;
          } else {
            cout << "Still not improving. Trying smaller dp" << endl;
            p[twiddle_pass] += dp[twiddle_pass];
            dp[twiddle_pass] *= 0.9;
            twiddle_iter++;
            RestartSim(server);
          }
        }
        else {
          if (twiddle_error < best_err) {
            best_err = twiddle_error;
            cout << "New Error is better than previous best error. Moving to next parameter" << endl;
            dp[twiddle_pass] *= 1.1;
            twiddle_pass++;
            twiddle_iter++;
            RestartSim(server);
          } 
          else {
            p[twiddle_pass] -= 2 * dp[twiddle_pass];
            cout << "New Error is worse than previous best trying in the opposite direction" << endl;          
            is_retrial = true;
            twiddle_iter++;
            RestartSim(server);
          }
        }
      }
      else {
        cout << "\n\n***** Twiddle best parameters *****" << endl;
        cout << "Kp = " << p[0] << endl;
        cout << "Ki = " << p[1] << endl;
        cout << "Kd = " << p[2] << endl;
        use_twiddle = false;
      }
    }
    optimizer_t++;
  }
}

void PID::InitializeTwiddle() {

  // Initializes optimizer parameters
  for (int i = 0; i < num_params; i++) {
    if (p[i] == 0) dp[i] = 1;
    else dp[i] = 0.1 * p[i];
  }

  // sets optimizer flag to true
  use_twiddle = true;

  // sets optimizing parameter to 0
  twiddle_pass = 0;

  // Sets best_err to -1 so twiddle knows it's a first run
  best_err = -1;

  // Reset twiddle iteraction counter
  twiddle_iter = 0;

}

void PID::RestartSim(uWS::WebSocket<uWS::SERVER> ws) {
  Init(p[0], p[1], p[2]);
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

void PID::SetServer(uWS::WebSocket<uWS::SERVER> ws) {
  server = ws;
}