#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  // It isn't really necessary to have persistent
  // storage for these in FusionEKF.
  // They can just as easily be constructed at local scope
  // the first time ProcessMeasurement() is called,
  // then used to initialize persistent containers in
  // KalmanFilter.  I do use them to initialize containers 
  // in KalmanFilter, but I'm keeping them here as well because
  // they are small and if it ain't broke, don't fix it.
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  // 1measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
  
  // Finish initializing the FusionEKF.

  // Set the process and measurement noises
  noise_ax = 9.;
  noise_ay = 9.;
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
    // Create the covariance matrix.

    cout << "Initializing FusionEKF" << endl;

    VectorXd x(4);
    if( measurement_pack.sensor_type_ == MeasurementPackage::RADAR ) 
    {
      // Convert radar from polar to cartesian coordinates and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      x << rho*cos(phi), rho*sin(phi), 0.f, 0.f;
    }
    else if( measurement_pack.sensor_type_ == MeasurementPackage::LASER ) 
    {
      // Initialize state.
      x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.f, 0.f;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // Estimate the initial state covariance matrix
    // with a moderate covariance of 1 for the x and y position values.
    // 
    // The initial x and y velocities are unknown (we initialized them
    // as 0 above).  By associating a large covariance of 1000 with them
    // we are telling the Kalman filter equations that these (totally fabricated) 
    // initial values of 0 for vx and vy are highly uncertain, and should not be
    // given very much weight in subsequent measurement updates.
    MatrixXd P(4, 4);
    P << 1, 0, 0, 0,
	 0, 1, 0, 0,
         0, 0, 1000, 0,
         0, 0, 0, 1000;

    // Initialize transition matrix
    MatrixXd F(4, 4);
    F << 1, 0, 0, 0,
	 0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    // Initialize measurement matrix for laser measurements
    H_laser_ << 1, 0, 0, 0,
               0, 1, 0, 0;

    // Initialize ekf_ with the first state vector, 
    // estimated initial state covariance matrix,
    // and an empty matrix for Q
    MatrixXd Q(4,4);
    ekf_.Init( x, /*x_in*/ 
        P, /*P_in*/ 
        F, /*F_in*/
        H_laser_, /*H_in*/ 
        Hj_, /*Hj_in*/ 
        R_laser_, /*R_in*/ 
        R_radar_, /*R_ekf_in*/ 
        Q ); /*Q_in*/

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  
  // Update the state transition matrix F according to the new elapsed time.

  // Compute the time from the previous measurement in seconds.
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
  
  if( dt > 0. ) 
  { 
    // Update the motion model matrix for a timestep dt.
    // We use a motion model with a constant velocity.
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;
  
    // Update the process noise covariance matrix for a timestep dt.
    // Our motion model uses Gaussian random accelerations in the x and y directions.
    float dt2 = dt*dt;
    float dt3 = dt2*dt;
    float dt4 = dt3*dt;
    float dt4over4 = dt4/4.;
    float dt3over2 = dt3/2.;
    ekf_.Q_ << dt4over4*noise_ax,                 0, dt3over2*noise_ax,                0,
			     0, dt4over4*noise_ay,                 0, dt3over2*noise_ay,
	     dt3over2*noise_ax,                 0,      dt2*noise_ax,                 0,
			     0, dt3over2*noise_ay,                 0,      dt2*noise_ay;
    ekf_.Predict();
  }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
  {
    // Radar updates
    ekf_.UpdateEKF( measurement_pack.raw_measurements_ );
  } 
  else 
  {
    // Laser updates
    ekf_.Update( measurement_pack.raw_measurements_ );
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
