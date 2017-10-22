#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


constexpr double pi() { return std::atan(1)*4; }
const float EPSILON = 0.0001;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  // Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // initial state transition matrix F
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<  1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;

  // initialize state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<  1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;


  // process noise covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);

  // set the acceleration noise components
  noise_ax_ = 9;
  noise_ay_ = 9;


  // laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;  
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) 
  {
    // Initialize the state ekf_.x_ with the first measurement.
    cout << "EKF: ";
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      auto rho = measurement_pack.raw_measurements_[0];
      auto theta = measurement_pack.raw_measurements_[1];

      // we can't determine vx and vy from rho_dot. initializing inital velocity to zero
      ekf_.x_ << rho*std::cos(theta), rho*std::sin(theta), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }
    cout << ekf_.x_ << endl;

    // update timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // // debug: ignore radar data for now
  // if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
  //   return;

  // elapsed time (seconds)
  auto dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // update state transition matrix F
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // update process noise covariance matrix Q
  ekf_.Q_<< pow(dt, 4)/4*noise_ax_, 0, pow(dt, 3)/2*noise_ax_, 0,
              0, pow(dt, 4)/4*noise_ay_, 0, pow(dt, 3)/2*noise_ay_,
              pow(dt,3)/2*noise_ax_, 0, pow(dt,2)*noise_ax_, 0,
              0, pow(dt,3)/2*noise_ay_, 0, pow(dt,2)*noise_ay_;



  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
  {
    // update measurement function
    float px = ekf_.x_[0];
    float py = ekf_.x_[1];
    float vx = ekf_.x_[2];
    float vy = ekf_.x_[3];

    // bound px py to EPSILON
    if (fabs(px) < EPSILON && fabs(py) < EPSILON)
    {
      px = EPSILON;
      py = EPSILON;
    }
    float rho_pred = std::sqrt(px*px + py*py);
    if (rho_pred < EPSILON)
      std::cout << "WARNING: rho_pred close to zero " << rho_pred << std::endl;

    float theta_pred = std::atan2(py, px);
    if (fabs(theta_pred) > pi())
      std::cout << "ERROR: theta pred out of range " << theta_pred << std::endl;

    float rho_dot_pred = (px*vx + py*vy) / rho_pred;

    ekf_.hx_ = VectorXd(3);
    ekf_.hx_ << rho_pred, theta_pred, rho_dot_pred;


    // update measurement matrix
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

    // update measurement noise covariance matrix
    ekf_.R_ = R_radar_;

    // ekf update
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }

  // laser measurement
  else 
  {
    // update measurement matrix
    ekf_.H_ = H_laser_;

    // update measurement noise covariance matrix
    ekf_.R_ = R_laser_;

    // kf update
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
