# Extended Kalman Filter Project Writeup

This project is to implement an extended Kalman Filter (EKF) that receives
two kinds of noisy sensor measurements, and estimates the state of a moving
object.

One sensor is lidar, measuring the object's 2D position in cartesian
coordinates (p_x, p_y). The other sensor is radar, measuring the object's
radial velocity (range, bearing angle, range rate).
The state of the moving object is represented as position and
speed (p_x, p_y, v_x, v_y), a state covariance matrix P.

The EKF works as follows. Every time it receives a lidar or radar measurement.
It first predicts the next state using a state transition matrix F with
delta time.
Then it updates the state with the given measurement. If the measurement is
from lidar, it uses the Kalman Filter update equations.
If the measurement is from radar, it uses the extended Kalman Filter update
equations. The extended KF is needed here because the radar measurement is not
linear from the state vector.

The project is tested with the Term 2 Simulator, and evaluated by RMSE against
the ground truth data.

### Code
The main code lies in the following files:
* kalman_filter.cpp:
  * Predict() the next state.
  * Update() the state for lidar measurement,
    or UpdateEKF() for radar measurement.

* FusionEKF.cpp:
  * Initialize the noise covariance matrices.
  * ProcessMeasurement() by calling ekf_'s Predict(), Update(), and UpdateEKF().

* tools.cpp:
  * CalculateJacobian() matrix. This is the first order Taylor expansion of
    the measurement function z=h(x).
  * CalculateRMSE() between the estimated states and ground truth data.

### Project Rubic
* Compiling
 
  The code can compile and run
  ```
  mkdir -p build && cd build
  cmake .. && make
  ./ExtendedKF
  ```

* Accuracy

  * The RMSE for Dataset 1 in the simulator is [0.1073 0.0954 0.4769 0.4898].
    This meets the project criteria.

  * When using the lidar measurements alone, the RMSE is
    [0.1915 0.1593 0.6307 0.5242].

  * When using the radar measurements alone, the RMSE is
    [0.2527 0.3815 0.6504 0.7475].
  
  The sensor fusion does improve the accuracy.
 
* Follows the correct algorithm

  * The code follows what has been taught in the lessons. In particular, it
    fixed the angle into -pi and pi in the measurement function for radar.

  * The code initializes the state vector and covariance matrices with the
    first measuremnt.

  * The code first predicts and then updates.

  * The code handles both radar and lidar measurements.

* Code efficiency
  * The code avoids duplicate calculations and initializations.
