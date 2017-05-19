#include "PID.h"
#include <uWS/uWS.h>

using namespace std;

#define TOL               0.2

PID::PID() {
  error = new double[3] { 0.0, 0.0, 0.0 };
}

PID::~PID() {
  delete[] error;
  delete[] K;
  delete[] dp;
}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle_switch) {
  this->twiddle_switch = twiddle_switch;
  if (twiddle_switch) {
    dp = new double[3] { 1.0, 1.0, 1.0 };
    dpi = 0;
    twiddle_lvl = 0;
    err = 0.0;
    best_err = std::numeric_limits<double>::max();
    iterations = 0;
    K = new double[3] { 0.0, 0.0, 0.0 };
  } else {
    K = new double[3] { Kp, Ki, Kd };
  }
}

void PID::UpdateError(double cte) {
  error[1] += cte;
  error[2] = cte - error[0];
  error[0] = cte;

  err += pow(cte, 2.0);
  err /= iterations+1;
}

double PID::TotalError() {
  err = abs(-K[0] * error[0] - K[1] * error[1] - K[2] * error[2]);
  return -K[0] * error[0] - K[1] * error[1] - K[2] * error[2];
}

double PID::ClampValue(double min, double max) {
  double pid_error = TotalError();
  if(pid_error < min) {
    pid_error = min;
  }
  if(pid_error > max) {
    pid_error = max;
  }

  return pid_error;
}

void PID::Twiddle(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  if((dp[0]+dp[1]+dp[2]) > TOL) {
    cout << " Error: " << err << "DP: " << (dp[0]+dp[1]+dp[2]) << endl;
    switch (twiddle_lvl) {
      case 0:
      cout << "Iteration " << iterations << ", best error: " << best_err << endl;
      K[dpi] += dp[dpi];
      cout << "P: " << K[0] << "," << K[1] << "," << K[2] << endl;
      ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT); // Wait for next error
      twiddle_lvl = 1;
      return;

      case 1:
      if(err < best_err) {
        best_err = err;
        dp[dpi] *= 1.1;

        iterations++;
        twiddle_lvl = 0;
        dpi = (dpi + 1) %3;
        return;
      } else {
        K[dpi] -= 2 * dp[dpi];
        ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT); // Wait for next error
        twiddle_lvl = 2;
        return;
      }

      case 2:
      if(err < best_err) {
        best_err = err;
        dp[dpi] *= 1.1;
      } else {
        K[dpi] += dp[dpi];
        dp[dpi] *= 0.9;
      }

      iterations++;
      twiddle_lvl = 0;
      dpi = (dpi + 1) %3;
      return;
    }
  }
}
// void PID::Twiddle(uWS::WebSocket<uWS::SERVER> ws) {
//   std::string reset_msg = "42[\"reset\",{}]";
//   if((dp[0]+dp[1]+dp[2]) > TOL) {
//     switch (twiddle_lvl) {
//       case 0:
//       cout << "Iteration " << iterations << ", best error: " << best_err << endl;
//       cout << "Error: " << err << endl;
//       K[dpi] += dp[dpi];
//       cout << "P: " << K[0] << "," << K[1] << "," << K[2] << endl;
//       cout << "DP: " << dp[0] << "," << dp[1] << "," << dp[2] << endl;
//       ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
//       twiddle_lvl = 1;
//       return;
//
//       case 1:
//       cout << " Error: " << err << endl;
//       cout << " P: " << K[0] << "," << K[1] << "," << K[2] << endl;
//       cout << " DP: " << dp[0] << "," << dp[1] << "," << dp[2] << endl;
//       if(err < best_err) {
//         best_err = err;
//         dp[dpi] *= 1.1;
//
//         twiddle_lvl = 0;
//         iterations += 1;
//         dpi += 1;
//         if (dpi > 2) {
//           dpi = 0;
//         }
//         return;
//       } else {
//         K[dpi] -= 2*dp[dpi];
//         ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
//         twiddle_lvl = 2;
//         return;
//       }
//
//       case 2:
//       cout << "  Error: " << err << endl;
//       cout << "  P: " << K[0] << "," << K[1] << "," << K[2] << endl;
//       cout << "  DP: " << dp[0] << "," << dp[1] << "," << dp[2] << endl;
//       if(err < best_err) {
//         best_err = err;
//         dp[dpi] *= 1.1;
//       } else {
//         K[dpi] += dp[dpi];
//         dp[dpi] *= 0.9;
//       }
//
//       twiddle_lvl = 0;
//       iterations += 1;
//       dpi += 1;
//       if (dpi > 2) {
//         dpi = 0;
//       }
//       return;
//     }
//     // for(int i=0; i<3; i++) {
//     //   K[i] += dp[i];
//     //   std::string reset_msg = "42[\"reset\",{}]";
//     //   ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
//     //
//     //   if(err < best_err) {
//     //     best_err = err;
//     //     dp[i] *= 1.1;
//     //   } else {
//     //     K[i] -= 2*dp[i];
//     //     std::string reset_msg = "42[\"reset\",{}]";
//     //     ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
//     //
//     //     if(err < best_err) {
//     //       best_err = err;
//     //       dp[i] *= 1.1;
//     //     } else {
//     //       K[i] += dp[i];
//     //       dp[i] *= 0.9;
//     //     }
//     //   }
//     // }
//     // iterations++;
//     // cout << "P: " << K[0] << "," << K[1] << "," << K[2] << endl;
//   } else {
//     cout << "P after Twiddle: " << K[0] << "," << K[1] << "," << K[2] << endl;
//   }
// }
