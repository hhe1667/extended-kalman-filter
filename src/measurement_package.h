#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include<string>
#include "Eigen/Dense"

class MeasurementPackage {
public:
  enum SensorType {
    LASER,
    RADAR
  } sensor_type_;

  long long timestamp_;

  Eigen::VectorXd raw_measurements_;
  std::string to_string() const {
    std::string s = sensor_type_ == LASER ? "laser" : "radar";
    for (int i = 0; i < raw_measurements_.size(); i++) {
      s += " ";
      s += std::to_string(raw_measurements_[i]);
    }
    return s;
  }
};

#endif // MEASUREMENT_PACKAGE_H_
