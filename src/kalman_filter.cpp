#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
   // Done: predict the state
   x_ = F_ * x_;
   MatrixXd Ft = F_.transpose();
   P = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
   //Done: update the state by using Kalman Filter equationss
   VectorXd z_pred = H_ * x_;
   VectorXd y = z - z_pred;
   MatrixXd Ht = H_.transpose();
   MatrixXd S = H * P_ * Ht + R_;
   MatrixXd Si = S.inverse();
   MatrixXd K = P * Ht * Si;

   // New estimate
   x_ = x_ + K*y;
   int x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   P = (I - K*H_) * P_;
}

namespace {

MatrixXd CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // compute the Jacobian matrix
  float px2 = px*px;
  float py2 = py*py;
  float c1 = px*px + py*py;
  if (fabs(c1) < 0.0001) {
      cout << "Divided by zero";
      return Hj;
  }
  float c2 = sqrt(c1);
  float c3 = c1*c2;
  Hj << px / c2, py / c2, 0, 0,
       -py/c1, px/c1, 0, 0,
   py*(vx*py - vy*px) / c3, px*(vy*px - vx*py) / c3, px/c2, py/c2;

  return Hj;
}
}  // namespace

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Done: update the state by using Extended Kalman Filter equations
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  VectorXd hx(x_.size());
  float c1=sqrt(px*px+py*py);
  if(fabs(c1)<0.0001 || fabs(px)<0.0001) {
    print("Divided by zero.");
    return
  }
  hx << c1,
        atan2(py/px),  // should be -pi~pi
        (px*vx+py*vy)/c1;
   VectorXd y = z - hx;

   MatrixXd Hj = CalculateJacobian(x_);
   MatrixXd Hjt = Hj.transpose();
   MatrixXd S = Hj * P * Hjt + R;
   MatrixXd Si = S.inverse();
   MatrixXd K = P * Hjt * Si;

   // New estimate
   x = x + K*y;
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   P = (I-K*Hj)*P
}
