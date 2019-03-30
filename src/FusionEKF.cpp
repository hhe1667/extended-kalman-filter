#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  Hj_ = MatrixXd(3, 4);


  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  ekf_.x_ = VectorXd(4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  cout << "Input " << num_inputs_++ << ": " << measurement_pack.to_string() << endl;
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Done: Initialize the state ekf_.x_ with the first measurement.
     * Done: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Done: Convert radar from polar to cartesian coordinates
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float c1 = tan(phi);
      float c2 = sqrt(1 + c1*c1);
      if (fabs(c2) < 0.0001) {
        printf("Divided by zero.");
        return;
      }
      float px = rho / c2;
      float py = sqrt(rho*rho - px*px);
      ekf_.x_ << px, py, 0, 0;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Done: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0],
                 measurement_pack.raw_measurements_[1],
                 0, 0;
    }

    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

   // Done: Update the state transition matrix F according to the new elapsed time.
   // Time is measured in seconds.
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;
   ekf_.F_ << 1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1, 0,
              0, 0, 0, 1;

  // Done: Update the process noise covariance matrix.
  // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  float noise_ax = 5;
  float noise_ay = 5;
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt2 * dt2;
  ekf_.Q_ << dt4*noise_ax/4, 0, dt3*noise_ax/2, 0,
            0, dt4*noise_ay/4, 0, dt3*noise_ay/2,
            dt3*noise_ax/2, 0, dt2*noise_ax, 0,
            0, dt3*noise_ay/2, 0, dt2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * Done:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Done: Radar updates
    float rho = measurement_pack.raw_measurements_[0];
    float phi = measurement_pack.raw_measurements_[1];
    float rho_dot = measurement_pack.raw_measurements_[2];
    VectorXd z(3);
    z << rho, phi, rho_dot;
    // use ekf_.Init() instead of direct access?

    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(z);
  } else {
    // Done: Laser updates
    float px = measurement_pack.raw_measurements_[0];
    float py = measurement_pack.raw_measurements_[1];
    VectorXd z(2);
    z << px, py;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
