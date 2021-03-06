#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth) {
  // Done: Calculate the RMSE here.
  int n = estimations.size();
  int k = estimations[0].size();
  VectorXd rmse(k);
  for (int i = 0; i < k; i++) {
    float var = 0.0;
    for (int j = 0; j < n; j++) {
      float diff = estimations[j][i] - ground_truth[j][i];
      var += diff * diff;
    }
    rmse[i] = sqrt(var / n);
  }
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Done: Calculate a Jacobian here.
  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // compute the Jacobian matrix
  float px2 = px * px;
  float py2 = py * py;
  float c1 = px * px + py * py;
  if (fabs(c1) < 0.0001) {
    cout << "Divided by zero";
    Hj << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;
    return Hj;
  }
  float c2 = sqrt(c1);
  float c3 = c1 * c2;
  Hj << px/c2, py/c2, 0, 0,
       -py/c1, px/c1, 0, 0,
       py*(vx*py - vy*px)/c3, px * (vy*px - vx*py)/c3, px/c2, py/c2;

  return Hj;
}
